#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

// Numerical parameters
const double TOLERANCE = 1e-6;  // Convergence tolerance for Newton's method
const int MAX_ITERATIONS = 100; // Maximum iterations for Newton's method

// Helper function to compute the derivative coefficients
std::vector<double> computeDerivative(const std::vector<double>& poly) {
    std::vector<double> derivative;
    for (size_t i = 1; i < poly.size(); ++i) {
        derivative.push_back(poly[i] * i);
    }
    return derivative;
}

// Evaluate polynomial at a given point
double evaluatePoly(const std::vector<double>& poly, double x) {
    double result = 0;
    double power = 1;
    for (double coef : poly) {
        result += coef * power;
        power *= x;
    }
    return result;
}

// Newton's method to find critical points (derivative = 0)
std::vector<double> findCriticalPoints(const std::vector<double>& poly, double start, double end) {
    std::vector<double> criticalPoints;
    std::vector<double> derivative = computeDerivative(poly);
    std::vector<double> secondDerivative = computeDerivative(derivative);

    // Start sampling within the interval
    for (double x = start; x <= end; x += (end - start) / 10) {
        double x0 = x; // Initial guess
        int iterations = 0;

        while (iterations < MAX_ITERATIONS) {
            double f = evaluatePoly(derivative, x0);            // f'(x)
            double fPrime = evaluatePoly(secondDerivative, x0); // f''(x)

            if (std::fabs(f) < TOLERANCE) break; // Converged to a critical point
            if (std::fabs(fPrime) < TOLERANCE) break; // Avoid division by zero

            x0 -= f / fPrime; // Newton's step
            iterations++;
        }

        // Check if x0 is a valid critical point within bounds
        if (x0 >= start && x0 <= end) {
            bool isUnique = true;
            for (double cp : criticalPoints) {
                if (std::fabs(cp - x0) < TOLERANCE) {
                    isUnique = false;
                    break;
                }
            }
            if (isUnique) criticalPoints.push_back(x0);
        }
    }

    return criticalPoints;
}

// Find min-max of a single polynomial
std::pair<double, double> findMinMax(const std::vector<double>& poly, double start, double end) {
    if (poly.size() <= 1) return {0, 0}; // Constant or invalid polynomial

    // Find critical points
    std::vector<double> criticalPoints = findCriticalPoints(poly, start, end);

    // Evaluate polynomial at critical points and endpoints
    double minVal = evaluatePoly(poly, start);
    double maxVal = minVal;

    for (double point : criticalPoints) {
        double value = evaluatePoly(poly, point);
        minVal = std::min(minVal, value);
        maxVal = std::max(maxVal, value);
    }

    // Evaluate endpoints
    double endVal = evaluatePoly(poly, end);
    minVal = std::min(minVal, endVal);
    maxVal = std::max(maxVal, endVal);

    return {minVal, maxVal};
}

int main() {
    // Dataset: Polynomials represented as coefficient vectors
    std::vector<std::vector<double>> dataset = {
        {1, -3, 2},              // f(x) = 2x^2 - 3x + 1
        {-2, 0, 1},              // g(x) = x^2 - 2
        {0, 5, -1, 1},           // h(x) = x^3 - x^2 + 5x
        {0, -1, 0, 1, 0, -0.1}   // i(x) = -0.1x^5 + x^3 - x
    };

    double globalMin = std::numeric_limits<double>::infinity();
    double globalMax = -std::numeric_limits<double>::infinity();

    // Domain to evaluate polynomials
    double start = -10.0, end = 10.0;

    for (const auto& poly : dataset) {
        auto [minVal, maxVal] = findMinMax(poly, start, end);
        globalMin = std::min(globalMin, minVal);
        globalMax = std::max(globalMax, maxVal);
        std::cout << "Polynomial Min: " << minVal << ", Max: " << maxVal << "\n";
    }

    std::cout << "Global Min: " << globalMin << ", Global Max: " << globalMax << "\n";

    return 0;
}
