#include <Arduino.h>
#include <Eigen.h>  // Linear algebra library for matrix operations
using namespace Eigen;

class PolynomialFitter {
private:
    static const int MAX_DEGREE = 4;
    static const int MAX_DATAPOINTS = 50;
    
    // Coefficients and their uncertainties
    double coefficients[MAX_DEGREE + 1];
    double coeffUncertainties[MAX_DEGREE + 1];
    
    // Data storage with individual point uncertainties
    struct DataPoint {
        double x;
        double y;
        double uncertainty;  // Measurement uncertainty for each point
    };
    
    DataPoint dataPoints[MAX_DATAPOINTS];
    int dataPointCount = 0;
    
public:
    void addDataPoint(double x, double y, double uncertainty = 1.0) {
        if (dataPointCount < MAX_DATAPOINTS) {
            dataPoints[dataPointCount] = {x, y, uncertainty};
            dataPointCount++;
        }
    }
    
    // Weighted least squares polynomial fitting
    bool fitPolynomial(int degree) {
        if (degree > MAX_DEGREE || dataPointCount <= degree) return false;
        
        // Create design matrix and weight matrix
        MatrixXd A(dataPointCount, degree + 1);
        VectorXd b(dataPointCount);
        VectorXd weights(dataPointCount);
        
        // Populate design matrix and weight vector
        for (int i = 0; i < dataPointCount; i++) {
            weights(i) = 1.0 / (dataPoints[i].uncertainty * dataPoints[i].uncertainty);
            
            for (int j = 0; j <= degree; j++) {
                A(i, j) = pow(dataPoints[i].x, j);
            }
            b(i) = dataPoints[i].y;
        }
        
        // Weighted normal equations solution
        MatrixXd AtW = A.transpose() * weights.asDiagonal();
        VectorXd solution = (AtW * A).ldlt().solve(AtW * b);
        
        // Store coefficients
        for (int i = 0; i <= degree; i++) {
            coefficients[i] = solution(i);
        }
        
        // Compute coefficient uncertainties
        computeCoefficientUncertainties(A, b, weights, degree);
        
        return true;
    }
    
    // Compute coefficient and fit uncertainties
    void computeCoefficientUncertainties(const MatrixXd& A, 
                                         const VectorXd& b, 
                                         const VectorXd& weights,
                                         int degree) {
        // Compute covariance matrix
        MatrixXd AtWA = (A.transpose() * weights.asDiagonal() * A);
        MatrixXd covarianceMatrix = AtWA.inverse();
        
        // Extract diagonal (variance of coefficients)
        for (int i = 0; i <= degree; i++) {
            coeffUncertainties[i] = sqrt(covarianceMatrix(i, i));
        }
    }
    
    // Evaluate polynomial at a given x
    double evaluate(double x, int degree) {
        double result = 0;
        for (int i = 0; i <= degree; i++) {
            result += coefficients[i] * pow(x, i);
        }
        return result;
    }
    
    // Compute total fitting error (weighted residual sum of squares)
    double computeFittingError(int degree) {
        double totalError = 0;
        for (int i = 0; i < dataPointCount; i++) {
            double predicted = evaluate(dataPoints[i].x, degree);
            double residual = dataPoints[i].y - predicted;
            
            // Weighted residual
            double weightedResidual = residual / dataPoints[i].uncertainty;
            totalError += weightedResidual * weightedResidual;
        }
        return sqrt(totalError / (dataPointCount - degree - 1));
    }
    
    // Print coefficients and their uncertainties
    void printResults(int degree) {
        Serial.println("\nPolynomial Fitting Results:");
        for (int i = 0; i <= degree; i++) {
            Serial.printf("a%d: %.4f ± %.4f\n", 
                          i, 
                          coefficients[i], 
                          coeffUncertainties[i]);
        }
        
        double fittingError = computeFittingError(degree);
        Serial.printf("\nFitting Error (Weighted RMS): %.4f\n", fittingError);
    }
};

// Global instance
PolynomialFitter fitter;

void setup() {
    Serial.begin(115200);
    
    // Simulate noisy sensor data with varying uncertainties
    randomSeed(analogRead(0));
    
    // Example: Quadratic function with added noise
    double trueCoeffs[] = {2.0, 0.5, -0.1};  // y = 2 + 0.5x - 0.1x^2
    
    for (int i = 0; i < 30; i++) {
        double x = i * 0.5;
        
        // True function value
        double trueY = trueCoeffs[0] + 
                       trueCoeffs[1] * x + 
                       trueCoeffs[2] * x * x;
        
        // Introduce noise with position-dependent uncertainty
        // Uncertainty increases with x to simulate sensor drift
        double noise = random(-100, 100) / 100.0;
        double uncertainty = 0.1 + 0.05 * x;
        
        fitter.addDataPoint(x, trueY + noise, uncertainty);
    }
    
    // Fit polynomials of different degrees
    for (int degree = 1; degree <= 3; degree++) {
        Serial.printf("\n--- Degree %d Polynomial Fit ---\n", degree);
        
        if (fitter.fitPolynomial(degree)) {
            fitter.printResults(degree);
        }
    }
}

void loop() {
    // No additional processing needed
    delay(10000);  // Wait 10 seconds before repeating
}
