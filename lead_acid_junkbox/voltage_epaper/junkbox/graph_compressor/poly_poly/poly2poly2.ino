#include <Arduino.h>
#include <Eigen.h>
#include <vector>
#include <cmath>
#include <algorithm>

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

    bool isFull() {
        return currentIndex == 0 && polynomials[size - 1].coefficients.size() != 0;
    }

    void clear() {
        currentIndex = 0;
        for (int i = 0; i < size; i++) polynomials[i].coefficients.resize(0);
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

std::vector<double> findCriticalPoints(const Polynomial& poly, double startTime, double endTime) {
    std::vector<double> criticalPoints;
    if (poly.coefficients.size() < 2) return criticalPoints;

    // Find local extrema (first derivative = 0)
    std::vector<double> derivativeCoeffs(poly.coefficients.size() - 1);
    for (int i = 1; i < poly.coefficients.size(); i++) {
        derivativeCoeffs[i - 1] = i * poly.coefficients(i);
    }

    // Solve for roots of the derivative (up to quadratic)
    if (derivativeCoeffs.size() == 2) {
        double x = -derivativeCoeffs[0] / derivativeCoeffs[1];
        if (x >= 0 && x <= endTime - startTime) criticalPoints.push_back(x + startTime);
    } else if (derivativeCoeffs.size() == 3) {
        double a = derivativeCoeffs[2];
        double b = derivativeCoeffs[1];
        double c = derivativeCoeffs[0];
        double discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            double x1 = (-b + sqrt(discriminant)) / (2 * a);
            double x2 = (-b - sqrt(discriminant)) / (2 * a);
            if (x1 >= 0 && x1 <= endTime - startTime) criticalPoints.push_back(x1 + startTime);
            if (x2 >= 0 && x2 <= endTime - startTime) criticalPoints.push_back(x2 + startTime);
        }
    }

    std::sort(criticalPoints.begin(), criticalPoints.end());
    return criticalPoints;
}

VectorXd fitPolynomialConstrained(const std::vector<Polynomial>& polynomials, unsigned long totalDuration, int order, int pointsPerSegment = 5) {
    if (polynomials.empty()) return VectorXd::Zero(0);

    std::vector<std::pair<double, double>> dataPoints;

    for (const auto& poly : polynomials) {
        double segmentDuration = totalDuration / polynomials.size();
        double startTime = poly.startTime;
        double endTime = startTime + segmentDuration;

        std::vector<double> criticalPoints = findCriticalPoints(poly, startTime, endTime);
        for (double point : criticalPoints) {
            dataPoints.push_back({point, evaluatePolynomial(poly.coefficients, point - startTime)});
        }

        // Add evenly spaced points within segment for better fit
        for (int i = 0; i < pointsPerSegment; ++i) {
            double t = startTime + (segmentDuration * i) / (pointsPerSegment - 1.0);
            dataPoints.push_back({t, evaluatePolynomial(poly.coefficients, t - startTime)});
        }
    }

    //add junction points
    for (int i = 1; i < polynomials.size(); i++) {
        dataPoints.push_back({(double)polynomials[i].startTime, evaluatePolynomial(polynomials[i - 1].coefficients, polynomials[i].startTime - polynomials[i - 1].startTime)});
    }

    std::sort(dataPoints.begin(), dataPoints.end());
    dataPoints.erase(std::unique(dataPoints.begin(), dataPoints.end()), dataPoints.end());

    int numDataPoints = dataPoints.size();
    int numConstraints = (polynomials.size() - 1) * 2; // Continuity and derivative for each junction
    int numCoefficients = order + 1;

    MatrixXd A(numDataPoints + numConstraints, numCoefficients);
    VectorXd b(numDataPoints + numConstraints);
    int rowIndex = 0;

    for (const auto& point : dataPoints) {
        for (int j = 0; j <= order; j++) {
            A(rowIndex, j) = pow(point.first, j);
        }
        b(rowIndex) = point.second;
        rowIndex++;
    }

    // Continuity and derivative constraints:
    for (int i = 0; i < polynomials.size() - 1; ++i) {
        double junctionTime = polynomials[i + 1].startTime;

        // Continuity constraint
        for (int j = 0; j <= order; j++) {
            A(rowIndex, j) = pow(junctionTime, j);
        }
        b(rowIndex) = evaluatePolynomial(polynomials[i].coefficients, junctionTime - polynomials[i].startTime);
        rowIndex++;

        // First derivative constraint
        for (int j = 1; j <= order; j++) {
            A(rowIndex, j) = j * pow(junctionTime, j - 1);
        }
        double derivativeValue = 0;
        for (int j = 1; j < polynomials[i].coefficients.size(); ++j) {
            derivativeValue += j * polynomials[i].coefficients(j) * pow(junctionTime - polynomials[i].startTime, j - 1);
        }
        b(rowIndex) = derivativeValue;
        rowIndex++;
    }

    if (A.rows() == 0 || A.cols() == 0 || b.size() == 0) return VectorXd::Zero(0);

    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    static PolynomialBuffer polyBuffer(16);
    static PolynomialBuffer fittedPolyBuffer(16);
    static unsigned long startTime = 0;
    static int polynomialCounter = 0;

    if (polynomialCounter < 16) {
        Polynomial newPoly;
        newPoly.coefficients = VectorXd::Random(3); // Example, random coefficients (adjust as needed)
        newPoly.startTime = startTime;
        polyBuffer.addPolynomial(newPoly);
        startTime += 1000; // Example duration
        polynomialCounter++;
        delay(500);
    }

    if (polyBuffer.isFull()) {
        unsigned long totalDuration = polyBuffer.polynomials[polyBuffer.size - 1].startTime + 1000 - polyBuffer.polynomials[0].startTime;
        Polynomial fittedPoly;
        fittedPoly.coefficients = fitPolynomialConstrained(polyBuffer.polynomials, totalDuration, 8);
        fittedPoly.startTime = polyBuffer.polynomials[0].startTime;
        if (fittedPoly.coefficients.size() > 0) {
            fittedPolyBuffer.addPolynomial(fittedPoly);
            Serial.println("Fitted polynomial added to buffer.");
            polyBuffer.clear();
            polynomialCounter = 0;
        }
    }

    if (fittedPolyBuffer.isFull()) {
        Serial.println("Fitted Polynomial Buffer is full:");
        for (const auto& poly : fittedPolyBuffer.polynomials) {
            Serial.print("Start Time: ");
            Serial.println(poly.startTime);
            Serial.print("Coefficients: ");
            for (int i = 0; i < poly.coefficients.size(); i++) {
                Serial.print(poly.coefficients(i), 6);
                Serial.print(" ");
            }
            Serial.println();
        }
        fittedPolyBuffer.clear();
    }

    delay
