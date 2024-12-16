#include <Arduino.h>
#include <Eigen.h>
#include <vector>
#include <random>

using namespace Eigen;

// Structure for a single polynomial
struct Polynomial {
  VectorXd coefficients;
  unsigned long startTime; // Start time of this polynomial's validity
};

// Structure for the polynomial buffer
struct PolynomialBuffer {
  std::vector<Polynomial> polynomials;
  int currentIndex = 0;
  int size;

  PolynomialBuffer(int s) : size(s) {
      polynomials.resize(size);
  }

  void addPolynomial(const Polynomial& poly) {
    polynomials[currentIndex] = poly;
    currentIndex = (currentIndex + 1) % size;
  }

    bool isFull(){
        return currentIndex == 0 && polynomials[size-1].coefficients.size() != 0;
    }

    void clear(){
        currentIndex = 0;
        for(int i = 0; i < size; i++) polynomials[i].coefficients.resize(0);
    }
};

// Function to evaluate a polynomial at a given x
double evaluatePolynomial(const VectorXd& coefficients, double x) {
  double result = 0.0;
  for (int i = 0; i < coefficients.size(); i++) {
    result += coefficients(i) * pow(x, i);
  }
  return result;
}

// Function to fit an 8th order polynomial using Monte Carlo
Polynomial monteCarloFit(const PolynomialBuffer& buffer, unsigned long totalDuration, int iterations = 10000) {
    if(!buffer.isFull()){
        Serial.println("Buffer not full, cannot perform fit.");
        return {VectorXd::Zero(0), 0};
    }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distrib(-10.0, 10.0); // Adjust range as needed

  Polynomial bestPolynomial;
  bestPolynomial.coefficients = VectorXd::Zero(9); // 9 coefficients for 8th order
  double bestError = std::numeric_limits<double>::max();

  for (int iter = 0; iter < iterations; iter++) {
    VectorXd currentCoefficients(9);
    for (int i = 0; i < 9; i++) {
      currentCoefficients(i) = distrib(gen);
    }

    double currentError = 0.0;
    for (unsigned long t = 0; t <= totalDuration; t+= totalDuration/1000) { // Evaluate at many points
        double trueValue = 0;
        for(const auto& poly : buffer.polynomials){
            if(t >= poly.startTime && t < poly.startTime + totalDuration/buffer.size) {
                trueValue = evaluatePolynomial(poly.coefficients, t - poly.startTime);
                break;
            }
        }
      double fittedValue = evaluatePolynomial(currentCoefficients, t);
      currentError += pow(fittedValue - trueValue, 2); // Sum of squared errors
    }

    if (currentError < bestError) {
      bestError = currentError;
      bestPolynomial.coefficients = currentCoefficients;
    }
  }
    bestPolynomial.startTime = buffer.polynomials[0].startTime;
  return bestPolynomial;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
    static PolynomialBuffer polyBuffer(16);
    static PolynomialBuffer fittedPolyBuffer(16);
    static unsigned long startTime = 0;
    static int polynomialCounter = 0;

    if(polynomialCounter < 16){
        Polynomial newPoly;
        newPoly.coefficients = VectorXd::Random(3); //Example, random coefficients
        newPoly.startTime = startTime;
        polyBuffer.addPolynomial(newPoly);
        startTime += 1000; //Example duration
        polynomialCounter++;
        delay(500);
    }

    if(polyBuffer.isFull()){
        unsigned long totalDuration = polyBuffer.polynomials[polyBuffer.size-1].startTime + 1000 - polyBuffer.polynomials[0].startTime;
        Polynomial fittedPoly = monteCarloFit(polyBuffer, totalDuration);
        if(fittedPoly.coefficients.size() > 0){
            fittedPolyBuffer.addPolynomial(fittedPoly);
            Serial.println("Fitted polynomial added to buffer.");
            polyBuffer.clear();
            polynomialCounter = 0;
        }
    }

    if(fittedPolyBuffer.isFull()){
        Serial.println("Fitted Polynomial Buffer is full:");
        for(const auto& poly : fittedPolyBuffer.polynomials){
            Serial.print("Start Time: ");
            Serial.println(poly.startTime);
            Serial.print("Coefficients: ");
            for(int i = 0; i < poly.coefficients.size(); i++){
                Serial.print(poly.coefficients(i));
                Serial.print(" ");
            }
            Serial.println();
        }
        fittedPolyBuffer.clear();
    }

  delay(100);
}
