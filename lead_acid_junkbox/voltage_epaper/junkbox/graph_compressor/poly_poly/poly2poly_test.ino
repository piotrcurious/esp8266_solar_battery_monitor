#include <Arduino.h>
#include <Eigen.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace Eigen;

// ... (Polynomial struct, evaluatePolynomial, findCriticalPoints, fitPolynomialConstrained functions from previous response)

void plotFunction(const std::vector<Polynomial>& polynomials, unsigned long totalDuration, int numPoints = 200) {
    Serial.println("FUNCTION"); // Start of function data for plotter
    for (int i = 0; i < numPoints; ++i) {
        double t = (totalDuration * i) / (numPoints - 1.0);
        double trueValue = 0;
        for (const auto& poly : polynomials) {
            if (t >= poly.startTime && t < poly.startTime + totalDuration / polynomials.size()) {
                trueValue = evaluatePolynomial(poly.coefficients, t - poly.startTime);
                break;
            }
        }
        Serial.print(t);
        Serial.print(",");
        Serial.println(trueValue, 6);
    }
    Serial.println("END_FUNCTION"); // End of function data
}

void plotPolynomial(const Polynomial& poly, unsigned long startTime, unsigned long endTime, int numPoints = 200) {
    Serial.println("POLYNOMIAL"); // Start of polynomial data for plotter
    for (int i = 0; i < numPoints; ++i) {
        double t = startTime + ((endTime - startTime) * i) / (numPoints - 1.0);
        double fittedValue = evaluatePolynomial(poly.coefficients, t - startTime);
        Serial.print(t);
        Serial.print(",");
        Serial.println(fittedValue, 6);
    }
    Serial.println("END_POLYNOMIAL"); // End of polynomial data
}

void runTest(int numPolynomials, int polynomialOrder, int fittedPolynomialOrder, int pointsPerSegment) {
    PolynomialBuffer polyBuffer(numPolynomials);
    unsigned long startTime = 0;

    Serial.print("Running Test with ");
    Serial.print(numPolynomials);
    Serial.print(" polynomials of order ");
    Serial.print(polynomialOrder);
    Serial.print(", fitting to order ");
    Serial.println(fittedPolynomialOrder);

    std::vector<Polynomial> testPolynomials;

    for (int i = 0; i < numPolynomials; i++) {
        Polynomial newPoly;
        newPoly.coefficients = VectorXd::Random(polynomialOrder + 1); // Random coefficients
        newPoly.startTime = startTime;
        polyBuffer.addPolynomial(newPoly);
        testPolynomials.push_back(newPoly);
        startTime += 1000;
    }

    unsigned long totalDuration = polyBuffer.polynomials[polyBuffer.size - 1].startTime + 1000 - polyBuffer.polynomials[0].startTime;

    plotFunction(testPolynomials, totalDuration);

    Polynomial fittedPoly;
    fittedPoly.coefficients = fitPolynomialConstrained(polyBuffer.polynomials, totalDuration, fittedPolynomialOrder, pointsPerSegment);
    fittedPoly.startTime = polyBuffer.polynomials[0].startTime;

    if (fittedPoly.coefficients.size() > 0) {
        plotPolynomial(fittedPoly, fittedPoly.startTime, fittedPoly.startTime + totalDuration);
    } else {
        Serial.println("Fitting failed.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial Monitor to open
}

void loop() {
    // Test cases (adjust parameters as needed)
    runTest(3, 2, 8, 5);   // 3 quadratic polynomials, fit to 8th order
    delay(5000);
    runTest(4, 3, 8, 5);   // 4 cubic polynomials, fit to 8th order
    delay(5000);
    runTest(2, 1, 8, 5); //2 linear functions, fit to 8th order
    delay(5000);
    while(true); //stop after tests
}
