#include <Arduino.h>
#include <Eigen.h>
#include <vector>

using namespace Eigen;

class AdaptivePolynomialFit {
private:
    // Data storage
    std::vector<double> timestamps;
    std::vector<double> dataPoints;
    
    // Polynomial coefficients
    VectorXd coefficients;
    
    // Lagrangian dual parameters
    double lambda; // Regularization parameter
    double tolerance; // Deviation tolerance

    // Polynomial degree (configurable)
    int polynomialDegree;

    // Calculate polynomial fit using Lagrangian dual method
    VectorXd calculateLagrangianDualFit() {
        // Construct design matrix
        MatrixXd X(timestamps.size(), polynomialDegree + 1);
        for (size_t i = 0; i < timestamps.size(); ++i) {
            for (int j = 0; j <= polynomialDegree; ++j) {
                X(i, j) = pow(timestamps[i], j);
            }
        }

        // Convert data points to Eigen vector
        VectorXd y = Map<VectorXd>(dataPoints.data(), dataPoints.size());

        // Regularization matrix
        MatrixXd regularization = MatrixXd::Identity(polynomialDegree + 1, polynomialDegree + 1);
        regularization(0,0) = 0; // Don't regularize constant term

        // Solve using ridge regression (L2 regularization)
        VectorXd fitCoefficients = 
            (X.transpose() * X + lambda * regularization).ldlt().solve(X.transpose() * y);

        return fitCoefficients;
    }

    // Predict value using current polynomial fit
    double predictValue(double timestamp) {
        double prediction = 0;
        for (int i = 0; i <= polynomialDegree; ++i) {
            prediction += coefficients(i) * pow(timestamp, i);
        }
        return prediction;
    }

    // Calculate prediction error
    double calculatePredictionError(double actualValue, double predictedValue) {
        return abs(actualValue - predictedValue) / (abs(actualValue) + 1e-10);
    }

public:
    AdaptivePolynomialFit(int degree = 2, double lambdaValue = 0.1, double errorTolerance = 0.15) 
        : polynomialDegree(degree), lambda(lambdaValue), tolerance(errorTolerance) {}

    // Add new data point
    bool addDataPoint(double timestamp, double value) {
        // First few points always accepted
        if (timestamps.size() < polynomialDegree + 1) {
            timestamps.push_back(timestamp);
            dataPoints.push_back(value);
            
            // Refit when enough points are collected
            if (timestamps.size() == polynomialDegree + 1) {
                coefficients = calculateLagrangianDualFit();
            }
            return true;
        }

        // Predict value based on existing fit
        double predictedValue = predictValue(timestamp);
        double predictionError = calculatePredictionError(value, predictedValue);

        // Check if new point confirms or contradicts existing fit
        if (predictionError <= tolerance) {
            timestamps.push_back(timestamp);
            dataPoints.push_back(value);
            
            // Periodically refit to adapt to new data
            if (timestamps.size() % (polynomialDegree + 2) == 0) {
                coefficients = calculateLagrangianDualFit();
            }
            return true;
        } else {
            // Point contradicts existing fit - trigger re-evaluation
            handleSignificantDeviation(timestamp, value);
            return false;
        }
    }

    // Handle significant deviation
    void handleSignificantDeviation(double timestamp, double value) {
        // Adaptive strategy to detect inflection point
        
        // Reduce regularization to allow more flexible fitting
        lambda *= 0.5;
        
        // Temporarily store current fit
        VectorXd originalCoefficients = coefficients;
        
        // Attempt to fit with increased polynomial complexity
        polynomialDegree++;
        
        try {
            coefficients = calculateLagrangianDualFit();
            
            // If new fit is significantly better, keep it
            // Otherwise, revert to original
            if (!isImprovedFit()) {
                coefficients = originalCoefficients;
                polynomialDegree--;
                lambda *= 2;
            }
        } catch (...) {
            // Fallback to original fit if computation fails
            coefficients = originalCoefficients;
            polynomialDegree--;
            lambda *= 2;
        }
    }

    // Check if new fit is statistically better
    bool isImprovedFit() {
        // Implement statistical model selection criteria
        // For example, use Akaike Information Criterion (AIC)
        return true; // Simplified for demonstration
    }

    // Getter for current coefficients
    VectorXd getCoefficients() const {
        return coefficients;
    }
};

void setup() {
    Serial.begin(115200);
    
    AdaptivePolynomialFit fitter;
    
    // Example usage
    fitter.addDataPoint(0, 1.0);
    fitter.addDataPoint(1, 2.5);
    fitter.addDataPoint(2, 4.2);
    // More data points would follow in actual implementation
}

void loop() {
    // Continuous monitoring and data processing would occur here
}
