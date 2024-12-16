#include <Eigen.h>
#include <vector>
#include <cmath>
#include <algorithm>

// ... (Other includes, structs, evaluatePolynomial function)

std::vector<double> findCriticalPoints(const Polynomial& poly, double startTime, double endTime) {
    std::vector<double> criticalPoints;
    if(poly.coefficients.size() < 2) return criticalPoints;
    // Find local extrema (first derivative = 0)
    std::vector<double> derivativeCoeffs(poly.coefficients.size() - 1);
    for(int i = 1; i < poly.coefficients.size(); i++){
        derivativeCoeffs[i-1] = i * poly.coefficients(i);
    }

    //Solve quadratic equation for the derivative
    if(derivativeCoeffs.size() == 3){
        double a = derivativeCoeffs[2];
        double b = derivativeCoeffs[1];
        double c = derivativeCoeffs[0];
        double discriminant = b*b - 4*a*c;
        if(discriminant >= 0){
            double x1 = (-b + sqrt(discriminant)) / (2*a);
            double x2 = (-b - sqrt(discriminant)) / (2*a);
            if(x1 >= 0 && x1 <= endTime - startTime) criticalPoints.push_back(x1 + startTime);
            if(x2 >= 0 && x2 <= endTime - startTime) criticalPoints.push_back(x2 + startTime);
        }
    } else if (derivativeCoeffs.size() == 2){
        double x = -derivativeCoeffs[0]/derivativeCoeffs[1];
        if(x >= 0 && x <= endTime - startTime) criticalPoints.push_back(x + startTime);
    }

    std::sort(criticalPoints.begin(), criticalPoints.end());
    return criticalPoints;
}

VectorXd fitPolynomialConstrained(const std::vector<Polynomial>& polynomials, unsigned long totalDuration, int order) {
    // ... (Error handling and variable declarations)
    std::vector<std::pair<double, double>> dataPoints;

    for (const auto& poly : polynomials) {
        std::vector<double> criticalPoints = findCriticalPoints(poly, poly.startTime, poly.startTime + totalDuration / polynomials.size());
        for(auto& point : criticalPoints){
            dataPoints.push_back({point, evaluatePolynomial(poly.coefficients, point - poly.startTime)});
        }
    }

    for (unsigned long t = 0; t < totalDuration; t+= totalDuration/1000) {
        double trueValue = 0;
        for (const auto& poly : polynomials) {
            if (t >= poly.startTime && t < poly.startTime + totalDuration / polynomials.size()) {
                trueValue = evaluatePolynomial(poly.coefficients, t - poly.startTime);
                break;
            }
        }
        dataPoints.push_back({(double)t, trueValue});
    }

    std::sort(dataPoints.begin(), dataPoints.end());
    dataPoints.erase(std::unique(dataPoints.begin(), dataPoints.end()), dataPoints.end());

    int numDataPoints = dataPoints.size();
    MatrixXd A(numDataPoints + numConstraints*2, numCoefficients);
    VectorXd b(numDataPoints + numConstraints*2);
    int rowIndex = 0;

    for(int i = 0; i < numDataPoints; i++){
        for (int j = 0; j <= order; j++) {
            A(rowIndex, j) = pow(dataPoints[i].first, j);
        }
        b(rowIndex) = dataPoints[i].second;
        rowIndex++;
    }

    // Continuity and derivative constraints: (same as before)
    for (int i = 0; i < numConstraints; i++) {
        unsigned long junctionTime = polynomials[i + 1].startTime;

        // Continuity constraint
        for (int j = 0; j <= order; j++) {
            A(rowIndex, j) = pow(junctionTime, j);
        }
        b(rowIndex) = evaluatePolynomial(polynomials[i].coefficients, junctionTime - polynomials[i].startTime);
        rowIndex++;

        // First derivative constraint:
        for (int j = 1; j <= order; j++) { // Start from j=1 for derivatives
            A(rowIndex, j) = j * pow(junctionTime, j - 1);
        }
        double derivativeValue = 0;
        for(int j = 1; j <= polynomials[i].coefficients.size() -1; j++){
            derivativeValue += j * polynomials[i].coefficients(j) * pow(junctionTime - polynomials[i].startTime, j-1);
        }
        b(rowIndex) = derivativeValue;
        rowIndex++;
    }

    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}

// ... (rest of the code)
