#include <Arduino.h>
#include <Eigen.h>
using namespace Eigen;

class AdaptiveMeasurementSystem {
private:
    // System configuration
    static const int STATE_DIM = 6;  // Extended state vector
    static const int MEASUREMENT_DIM = 1;
    static const int MAX_DATAPOINTS = 50;
    static const int MAX_DEGREE = 3;

    // Extended state vector components
    enum StateIndex {
        NOISE_FLOOR = 0,
        MEASUREMENT_BIAS = 1,
        SYSTEM_GAIN = 2,
        TREND_COMPONENT_1 = 3,
        TREND_COMPONENT_2 = 4,
        UNCERTAINTY_EST = 5
    };

    // Kalman Filter Matrices
    MatrixXd F;  // State transition matrix
    MatrixXd H;  // Measurement mapping matrix
    MatrixXd Q;  // Process noise covariance
    MatrixXd R;  // Measurement noise covariance
    MatrixXd P;  // Error covariance matrix

    // State estimation
    VectorXd x;  // Current state estimate

    // Data collection
    struct DataPoint {
        double timestamp;
        double value;
        double estimated_uncertainty;
    };
    DataPoint dataPoints[MAX_DATAPOINTS];
    int dataPointCount = 0;

    // Polynomial fitting parameters
    VectorXd coefficients;

    // Measurement system parameters
    double samplingPeriod;
    unsigned long lastMeasurementTime;

    // Adaptive measurement configuration
    struct SystemConfig {
        double initialNoiseFloor;
        double processNoiseFactor;
        double measurementNoiseFactor;
        bool adaptiveNoiseEstimation;
    } config;

public:
    AdaptiveMeasurementSystem(double sampling_period = 0.1) : 
        samplingPeriod(sampling_period), 
        lastMeasurementTime(0) {
        
        // Initialize matrices
        initializeKalmanFilter();
        
        // Default configuration
        config = {
            .initialNoiseFloor = 0.1,
            .processNoiseFactor = 0.01,
            .measurementNoiseFactor = 0.1,
            .adaptiveNoiseEstimation = true
        };
    }

    void initializeKalmanFilter() {
        // Initialize Extended State Vector
        x = VectorXd::Zero(STATE_DIM);
        x(NOISE_FLOOR) = config.initialNoiseFloor;
        x(SYSTEM_GAIN) = 1.0;

        // State Transition Matrix (first-order system model)
        F = MatrixXd::Identity(STATE_DIM, STATE_DIM);
        F(NOISE_FLOOR, NOISE_FLOOR) = 0.99;  // Slow drift
        F(MEASUREMENT_BIAS, MEASUREMENT_BIAS) = 0.95;  // Bias decay
        F(SYSTEM_GAIN, SYSTEM_GAIN) = 0.99;  // Gain stability

        // Measurement Mapping Matrix
        H = MatrixXd::Zero(MEASUREMENT_DIM, STATE_DIM);
        H(0, NOISE_FLOOR) = 1.0;
        H(0, MEASUREMENT_BIAS) = 1.0;
        H(0, SYSTEM_GAIN) = 1.0;

        // Initial Error Covariance
        P = MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1.0;

        // Process Noise Covariance
        Q = MatrixXd::Identity(STATE_DIM, STATE_DIM) * config.processNoiseFactor;

        // Measurement Noise Covariance
        R = MatrixXd::Identity(MEASUREMENT_DIM, MEASUREMENT_DIM) * config.measurementNoiseFactor;
    }

    // Extended Kalman Filter Prediction Step
    void predict() {
        // Predict state
        x = F * x;

        // Predict error covariance
        P = F * P * F.transpose() + Q;
    }

    // Extended Kalman Filter Update Step
    void update(double measurement) {
        // Compute Kalman Gain
        MatrixXd K = P * H.transpose() * 
            (H * P * H.transpose() + R).inverse();

        // Measurement residual
        double y_residual = measurement - 
            (x(NOISE_FLOOR) + x(MEASUREMENT_BIAS) + x(SYSTEM_GAIN) * measurement);

        // Update state estimate
        x += K * y_residual;

        // Update error covariance
        P = (MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P;

        // Adaptive noise estimation
        if (config.adaptiveNoiseEstimation) {
            updateNoiseEstimation(y_residual);
        }
    }

    void updateNoiseEstimation(double residual) {
        // Adaptive noise floor and uncertainty estimation
        x(NOISE_FLOOR) = 0.9 * x(NOISE_FLOOR) + 0.1 * abs(residual);
        x(UNCERTAINTY_EST) = sqrt(x(NOISE_FLOOR));

        // Update measurement noise covariance
        R(0,0) = x(NOISE_FLOOR) * x(NOISE_FLOOR);
    }

    // Add measurement with Kalman filtering
    void addMeasurement(double value) {
        predict();
        update(value);

        // Store data point with estimated uncertainty
        if (dataPointCount < MAX_DATAPOINTS) {
            dataPoints[dataPointCount] = {
                (double)millis() / 1000.0,  // timestamp
                value,
                x(UNCERTAINTY_EST)  // uncertainty estimate
            };
            dataPointCount++;
        }
    }

    // Lagrangian polynomial fitting with uncertainty-aware approach
    bool performLagrangianFit(int degree) {
        if (dataPointCount <= degree) return false;

        // Construct design matrix with uncertainty weighting
        MatrixXd A(dataPointCount, degree + 1);
        VectorXd b(dataPointCount);
        VectorXd weights(dataPointCount);

        for (int i = 0; i < dataPointCount; i++) {
            // Compute inverse variance weights
            weights(i) = 1.0 / (dataPoints[i].estimated_uncertainty * 
                                dataPoints[i].estimated_uncertainty);

            // Populate design matrix
            for (int j = 0; j <= degree; j++) {
                A(i, j) = pow(dataPoints[i].timestamp, j);
            }
            b(i) = dataPoints[i].value;
        }

        // Regularization
        double lambda_reg = 1.0;
        MatrixXd AtW = A.transpose() * weights.asDiagonal();
        MatrixXd regularization = lambda_reg * MatrixXd::Identity(degree + 1, degree + 1);

        // Solve weighted least squares with regularization
        coefficients = VectorXd(degree + 1);
        coefficients = (AtW * A + regularization).ldlt().solve(AtW * b);

        return true;
    }

    // Print system state and fitting results
    void printSystemState(int degree) {
        Serial.println("\n--- Adaptive Measurement System State ---");
        
        // System state estimates
        Serial.printf("Noise Floor: %.4f\n", x(NOISE_FLOOR));
        Serial.printf("Measurement Bias: %.4f\n", x(MEASUREMENT_BIAS));
        Serial.printf("System Gain: %.4f\n", x(SYSTEM_GAIN));
        Serial.printf("Estimated Uncertainty: %.4f\n", x(UNCERTAINTY_EST));

        // Polynomial coefficients
        Serial.println("\nPolynomial Coefficients:");
        for (int i = 0; i <= degree; i++) {
            Serial.printf("a%d: %.4f\n", i, coefficients(i));
        }

        // Data points with uncertainties
        Serial.println("\nMeasurement Points:");
        for (int i = 0; i < dataPointCount; i++) {
            Serial.printf("t: %.2f, value: %.4f, uncertainty: %.4f\n", 
                          dataPoints[i].timestamp, 
                          dataPoints[i].value, 
                          dataPoints[i].estimated_uncertainty);
        }
    }

    // Reset for next measurement cycle
    void reset() {
        dataPointCount = 0;
        // Retain Kalman filter state and parameters
    }
};

// Global instance
AdaptiveMeasurementSystem measurementSystem;

void setup() {
    Serial.begin(115200);
    
    // Simulate multiple measurement cycles
    for (int cycle = 0; cycle < 3; cycle++) {
        Serial.printf("\n--- Measurement Cycle %d ---\n", cycle);
        
        // Simulate sensor data with complex characteristics
        randomSeed(analogRead(0));
        
        // Generate measurements with varying characteristics
        for (int i = 0; i < 30; i++) {
            // Simulate non-linear, noisy sensor signal
            double timestamp = i * 0.1;
            double baseSignal = 2.0 + 0.5 * timestamp - 0.1 * timestamp * timestamp;
            double noise = random(-100, 100) / 100.0 * (1 + 0.05 * timestamp);
            
            measurementSystem.addMeasurement(baseSignal + noise);
            
            // Small delay to simulate sampling period
            delay(50);
        }
        
        // Perform polynomial fitting
        int degree = 2;
        if (measurementSystem.performLagrangianFit(degree)) {
            measurementSystem.printSystemState(degree);
        }
        
        // Prepare for next cycle
        measurementSystem.reset();
    }
}

void loop() {
    // No additional processing needed
    delay(10000);  // Wait 10 seconds before repeating
}
