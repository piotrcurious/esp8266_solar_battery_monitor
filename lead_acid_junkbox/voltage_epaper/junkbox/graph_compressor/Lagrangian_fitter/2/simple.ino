#include <Arduino.h>
#include <Eigen.h>  // Linear algebra library for matrix operations

using namespace Eigen;

class LagrangianPolynomialFit {
private:
    const int MAX_DATAPOINTS = 50;
    const int POLYNOMIAL_DEGREE = 3;
    
    // Data storage
    float timestamps[50];
    float datapoints[50];
    int dataCount = 0;
    
    // Cost function parameters
    float lambda = 0.1;  // Regularization parameter

public:
    void addDataPoint(float timestamp, float value) {
        if (dataCount < MAX_DATAPOINTS) {
            timestamps[dataCount] = timestamp;
            datapoints[dataCount] = value;
            dataCount++;
        }
    }

    // Lagrangian dual problem formulation
    MatrixXf computePolynomialFit() {
        // Construct design matrix
        MatrixXf X(dataCount, POLYNOMIAL_DEGREE + 1);
        VectorXf y(dataCount);

        for (int i = 0; i < dataCount; i++) {
            y(i) = datapoints[i];
            for (int j = 0; j <= POLYNOMIAL_DEGREE; j++) {
                X(i, j) = pow(timestamps[i], j);
            }
        }

        // Regularization matrix (L2 norm)
        MatrixXf regularizationMatrix = MatrixXf::Identity(POLYNOMIAL_DEGREE + 1, POLYNOMIAL_DEGREE + 1);
        
        // Lagrangian dual objective: minimize (prediction error + regularization)
        MatrixXf XtX = X.transpose() * X;
        MatrixXf XtY = X.transpose() * y;
        
        // Solve with regularization (Ridge Regression)
        MatrixXf coefficients = (XtX + lambda * regularizationMatrix).ldlt().solve(XtY);

        return coefficients;
    }

    float predictValue(float timestamp, MatrixXf& coefficients) {
        float prediction = 0.0;
        for (int i = 0; i <= POLYNOMIAL_DEGREE; i++) {
            prediction += coefficients(i) * pow(timestamp, i);
        }
        return prediction;
    }
};

// Global instance
LagrangianPolynomialFit polynomialFitter;

void setup() {
    Serial.begin(115200);

    // Example dataset
    polynomialFitter.addDataPoint(0, 1.2);
    polynomialFitter.addDataPoint(1, 2.5);
    polynomialFitter.addDataPoint(2, 4.1);
    polynomialFitter.addDataPoint(3, 6.7);
    polynomialFitter.addDataPoint(4, 9.3);

    // Compute polynomial fit
    MatrixXf coefficients = polynomialFitter.computePolynomialFit();

    // Print coefficients
    Serial.println("Polynomial Coefficients:");
    for (int i = 0; i < coefficients.rows(); i++) {
        Serial.print("Degree ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(coefficients(i));
    }

    // Example prediction
    float testTimestamp = 2.5;
    float prediction = polynomialFitter.predictValue(testTimestamp, coefficients);
    Serial.print("Prediction at timestamp ");
    Serial.print(testTimestamp);
    Serial.print(": ");
    Serial.println(prediction);
}

void loop() {
    // Nothing to do in this example
}
