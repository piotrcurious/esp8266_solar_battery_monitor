#include <Arduino.h>
#include <Eigen.h>
using namespace Eigen;

class LagrangianUncertaintyFitter {
private:
    static const int MAX_DEGREE = 4;
    static const int MAX_DATAPOINTS = 50;
    
    // Data point structure
    struct DataPoint {
        double x;
        double y;
        double estimated_uncertainty;
    };
    
    DataPoint dataPoints[MAX_DATAPOINTS];
    int dataPointCount = 0;
    
    // Polynomial coefficients and estimated uncertainties
    VectorXd coefficients;
    VectorXd uncertainties;
    
    // Lagrangian optimization parameters
    double lambda_reg = 1.0;  // Regularization parameter
    int max_iterations = 50;
    double convergence_threshold = 1e-6;

public:
    // Lagrangian Dual Problem Formulation
    bool fitWithAdaptiveUncertainty(int degree) {
        if (degree > MAX_DEGREE || dataPointCount <= degree) return false;
        
        // Initialize coefficients and uncertainties
        coefficients = VectorXd::Zero(degree + 1);
        uncertainties = VectorXd::Ones(dataPointCount);
        
        // Iterative optimization
        for (int iter = 0; iter < max_iterations; iter++) {
            // Step 1: Optimize coefficients (fixing uncertainties)
            VectorXd prev_coefficients = coefficients;
            optimizeCoefficients(degree);
            
            // Step 2: Update measurement uncertainties
            updateUncertainties(degree);
            
            // Check convergence
            double coeff_change = (coefficients - prev_coefficients).norm();
            if (coeff_change < convergence_threshold) {
                break;
            }
        }
        
        return true;
    }
    
private:
    // Optimize coefficients using weighted least squares
    void optimizeCoefficients(int degree) {
        // Construct weighted design matrix
        MatrixXd A(dataPointCount, degree + 1);
        VectorXd b(dataPointCount);
        VectorXd weights(dataPointCount);
        
        for (int i = 0; i < dataPointCount; i++) {
            // Compute inverse variance weights
            weights(i) = 1.0 / (uncertainties(i) * uncertainties(i));
            
            // Populate design matrix
            for (int j = 0; j <= degree; j++) {
                A(i, j) = pow(dataPoints[i].x, j);
            }
            b(i) = dataPoints[i].y;
        }
        
        // Add regularization term to the Lagrangian
        MatrixXd AtW = A.transpose() * weights.asDiagonal();
        MatrixXd regularization = lambda_reg * MatrixXd::Identity(degree + 1, degree + 1);
        
        // Solve weighted least squares with regularization
        coefficients = (AtW * A + regularization).ldlt().solve(AtW * b);
    }
    
    // Update uncertainty estimates based on residuals
    void updateUncertainties(int degree) {
        for (int i = 0; i < dataPointCount; i++) {
            // Compute predicted value
            double predicted = 0;
            for (int j = 0; j <= degree; j++) {
                predicted += coefficients(j) * pow(dataPoints[i].x, j);
            }
            
            // Compute residual
            double residual = abs(dataPoints[i].y - predicted);
            
            // Adaptive uncertainty estimation
            // Uses a combination of current residual and prior uncertainty
            uncertainties(i) = sqrt(
                0.5 * residual +  // Adaptive component
                0.5 * uncertainties(i)  // Prior information
            );
        }
    }
    
public:
    // Add data point
    void addDataPoint(double x, double y) {
        if (dataPointCount < MAX_DATAPOINTS) {
            dataPoints[dataPointCount] = {x, y, 1.0};  // Initial uniform uncertainty
            dataPointCount++;
        }
    }
    
    // Compute total Lagrangian objective
    double computeLagrangianObjective(int degree) {
        double residual_term = 0;
        double regularization_term = 0;
        
        // Residual term
        for (int i = 0; i < dataPointCount; i++) {
            double predicted = 0;
            for (int j = 0; j <= degree; j++) {
                predicted += coefficients(j) * pow(dataPoints[i].x, j);
            }
            
            double weighted_residual = (dataPoints[i].y - predicted) / uncertainties(i);
            residual_term += weighted_residual * weighted_residual;
        }
        
        // Regularization term (L2 norm of coefficients)
        for (int j = 0; j <= degree; j++) {
            regularization_term += coefficients(j) * coefficients(j);
        }
        
        return residual_term + lambda_reg * regularization_term;
    }
    
    // Print results
    void printResults(int degree) {
        Serial.println("\nLagrangian Dual Problem Results:");
        
        // Print coefficients
        Serial.println("Polynomial Coefficients:");
        for (int i = 0; i <= degree; i++) {
            Serial.printf("a%d: %.4f\n", i, coefficients(i));
        }
        
        // Print estimated point uncertainties
        Serial.println("\nEstimated Point Uncertainties:");
        for (int i = 0; i < dataPointCount; i++) {
            Serial.printf("Point %d: %.4f\n", i, uncertainties(i));
        }
        
        // Compute and print Lagrangian objective
        double objective = computeLagrangianObjective(degree);
        Serial.printf("\nLagrangian Objective: %.4f\n", objective);
    }
};

// Global instance
LagrangianUncertaintyFitter fitter;

void setup() {
    Serial.begin(115200);
    
    // Simulate noisy sensor data
    randomSeed(analogRead(0));
    
    // Example: Quadratic function with complex noise
    double trueCoeffs[] = {2.0, 0.5, -0.1};  // y = 2 + 0.5x - 0.1x^2
    
    // Generate data with varying noise characteristics
    for (int i = 0; i < 30; i++) {
        double x = i * 0.5;
        
        // True function value
        double trueY = trueCoeffs[0] + 
                       trueCoeffs[1] * x + 
                       trueCoeffs[2] * x * x;
        
        // Non-uniform noise generation
        double noise = random(-100, 100) / 100.0 * (1 + 0.1 * x);
        
        fitter.addDataPoint(x, trueY + noise);
    }
    
    // Fit polynomials of different degrees
    for (int degree = 1; degree <= 3; degree++) {
        Serial.printf("\n--- Degree %d Polynomial Fit ---\n", degree);
        
        if (fitter.fitWithAdaptiveUncertainty(degree)) {
            fitter.printResults(degree);
        }
    }
}

void loop() {
    // No additional processing needed
    delay(10000);  // Wait 10 seconds before repeating
}
