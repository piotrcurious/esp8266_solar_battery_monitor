#include <Arduino.h>
#include <Eigen.h>
#include <vector>

using namespace Eigen;

class AdaptiveRandomIntervalMeasurementSystem {
private:
    // Enhanced system configuration
    static const int STATE_DIM = 8;  // Extended state vector
    static const int MEASUREMENT_DIM = 1;
    static const int MAX_DATAPOINTS = 100;
    static const int MAX_FREQUENCY_HISTORY = 20;

    // Expanded state vector indices
    enum StateIndex {
        NOISE_FLOOR = 0,
        MEASUREMENT_BIAS = 1,
        SYSTEM_GAIN = 2,
        TREND_COMPONENT_1 = 3,
        TREND_COMPONENT_2 = 4,
        SAMPLING_FREQUENCY = 5,
        FREQUENCY_VARIANCE = 6,
        SYSTEM_UNCERTAINTY = 7
    };

    // Advanced sampling interval tracking
    struct SamplingInterval {
        unsigned long timestamp;
        double interval;
        double frequency;
    };

    // Circular buffer for sampling intervals
    std::vector<SamplingInterval> samplingIntervals;
    
    // Kalman Filter Enhanced Matrices
    MatrixXd F;  // State transition matrix
    MatrixXd H;  // Measurement mapping matrix
    MatrixXd Q;  // Process noise covariance
    MatrixXd R;  // Measurement noise covariance
    MatrixXd P;  // Error covariance matrix

    // State estimation
    VectorXd x;  // Current state estimate

    // Measurement data storage
    struct MeasurementPoint {
        unsigned long timestamp;
        double value;
        double estimated_uncertainty;
        double sampling_frequency;
    };
    MeasurementPoint dataPoints[MAX_DATAPOINTS];
    int dataPointCount = 0;

    // Polynomial fitting parameters
    VectorXd coefficients;

    // System configuration and adaptation parameters
    struct SystemConfig {
        bool adaptiveFrequencyEstimation;
        bool dynamicNoiseModeling;
        double baseProcessNoise;
        double baseMeasurementNoise;
    } config;

    // Timing and frequency tracking
    unsigned long lastMeasurementTime;
    double instantFrequency;

public:
    AdaptiveRandomIntervalMeasurementSystem() : 
        lastMeasurementTime(0),
        instantFrequency(0) {
        
        // Initialize system
        initializeKalmanFilter();
        
        // Default configuration
        config = {
            .adaptiveFrequencyEstimation = true,
            .dynamicNoiseModeling = true,
            .baseProcessNoise = 0.01,
            .baseMeasurementNoise = 0.1
        };

        // Preallocate sampling intervals vector
        samplingIntervals.reserve(MAX_FREQUENCY_HISTORY);
    }

    void initializeKalmanFilter() {
        // Extended State Vector Initialization
        x = VectorXd::Zero(STATE_DIM);
        x(NOISE_FLOOR) = 0.1;
        x(SYSTEM_GAIN) = 1.0;
        x(SAMPLING_FREQUENCY) = 10.0;  // Initial frequency estimate
        x(FREQUENCY_VARIANCE) = 1.0;

        // Dynamic State Transition Matrix
        F = MatrixXd::Identity(STATE_DIM, STATE_DIM);
        F(NOISE_FLOOR, NOISE_FLOOR) = 0.95;  // Noise floor decay
        F(MEASUREMENT_BIAS, MEASUREMENT_BIAS) = 0.90;  // Bias decay
        F(SYSTEM_GAIN, SYSTEM_GAIN) = 0.98;  // Gain stability
        F(SAMPLING_FREQUENCY, SAMPLING_FREQUENCY) = 0.90;  // Frequency adaptation
        F(FREQUENCY_VARIANCE, FREQUENCY_VARIANCE) = 0.95;  // Variance decay

        // Measurement Mapping Matrix
        H = MatrixXd::Zero(MEASUREMENT_DIM, STATE_DIM);
        H(0, NOISE_FLOOR) = 1.0;
        H(0, MEASUREMENT_BIAS) = 1.0;
        H(0, SYSTEM_GAIN) = 1.0;
        H(0, SAMPLING_FREQUENCY) = 0.1;  // Frequency influence

        // Initial Error Covariance
        P = MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1.0;
    }

    void updateSamplingCharacteristics(unsigned long currentTime) {
        if (lastMeasurementTime > 0) {
            double interval = (currentTime - lastMeasurementTime) / 1000.0;  // seconds
            instantFrequency = 1.0 / interval;

            // Store sampling interval
            if (samplingIntervals.size() >= MAX_FREQUENCY_HISTORY) {
                samplingIntervals.erase(samplingIntervals.begin());
            }
            samplingIntervals.push_back({currentTime, interval, instantFrequency});

            // Update frequency-related state
            updateFrequencyEstimate();
        }
        
        lastMeasurementTime = currentTime;
    }

    void updateFrequencyEstimate() {
        if (samplingIntervals.size() > 1) {
            // Compute frequency statistics
            double meanFreq = 0, varFreq = 0;
            for (const auto& interval : samplingIntervals) {
                meanFreq += interval.frequency;
            }
            meanFreq /= samplingIntervals.size();

            for (const auto& interval : samplingIntervals) {
                varFreq += pow(interval.frequency - meanFreq, 2);
            }
            varFreq /= samplingIntervals.size();

            // Update Kalman state
            x(SAMPLING_FREQUENCY) = 0.8 * x(SAMPLING_FREQUENCY) + 0.2 * meanFreq;
            x(FREQUENCY_VARIANCE) = 0.8 * x(FREQUENCY_VARIANCE) + 0.2 * varFreq;
        }
    }

    void adaptNoiseCharacteristics() {
        // Dynamic noise modeling based on sampling characteristics
        double frequencyAdaptiveFactor = 1.0 + 0.1 * log(x(SAMPLING_FREQUENCY));
        double varianceAdaptiveFactor = 1.0 + 0.5 * x(FREQUENCY_VARIANCE);

        // Update process and measurement noise
        Q = MatrixXd::Identity(STATE_DIM, STATE_DIM) * 
            (config.baseProcessNoise * frequencyAdaptiveFactor * varianceAdaptiveFactor);
        
        R = MatrixXd::Identity(MEASUREMENT_DIM, MEASUREMENT_DIM) * 
            (config.baseMeasurementNoise * frequencyAdaptiveFactor);
    }

    void predict() {
        // Adaptive noise characteristics
        adaptNoiseCharacteristics();

        // Predict state
        x = F * x;

        // Predict error covariance
        P = F * P * F.transpose() + Q;
    }

    void update(double measurement, unsigned long currentTime) {
        // Update sampling characteristics first
        updateSamplingCharacteristics(currentTime);

        // Compute Kalman Gain
        MatrixXd K = P * H.transpose() * 
            (H * P * H.transpose() + R).inverse();

        // Measurement residual with frequency-aware model
        double predicted_measurement = 
            x(NOISE_FLOOR) + 
            x(MEASUREMENT_BIAS) + 
            x(SYSTEM_GAIN) * measurement +
            0.1 * x(SAMPLING_FREQUENCY);

        double y_residual = measurement - predicted_measurement;

        // Update state estimate
        x += K * y_residual;

        // Update error covariance
        P = (MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P;

        // Store measurement with additional metadata
        if (dataPointCount < MAX_DATAPOINTS) {
            dataPoints[dataPointCount] = {
                currentTime,
                measurement,
                sqrt(P(SYSTEM_UNCERTAINTY, SYSTEM_UNCERTAINTY)),
                instantFrequency
            };
            dataPointCount++;
        }
    }

    bool performAdaptiveFitting(int degree) {
        if (dataPointCount <= degree) return false;

        // Prepare design matrix with frequency-aware weighting
        MatrixXd A(dataPointCount, degree + 1);
        VectorXd b(dataPointCount);
        VectorXd weights(dataPointCount);

        for (int i = 0; i < dataPointCount; i++) {
            // Compute inverse variance weights with frequency influence
            double frequencyWeight = 1.0 / (1.0 + 0.1 * abs(dataPoints[i].sampling_frequency - x(SAMPLING_FREQUENCY)));
            weights(i) = frequencyWeight / (dataPoints[i].estimated_uncertainty * dataPoints[i].estimated_uncertainty);

            // Populate design matrix using normalized timestamps
            double normalizedTime = (dataPoints[i].timestamp - dataPoints[0].timestamp) / 1000.0;
            for (int j = 0; j <= degree; j++) {
                A(i, j) = pow(normalizedTime, j);
            }
            b(i) = dataPoints[i].value;
        }

        // Regularization with frequency-adaptive lambda
        double lambda_reg = 1.0 / (1.0 + 0.1 * x(SAMPLING_FREQUENCY));
        MatrixXd AtW = A.transpose() * weights.asDiagonal();
        MatrixXd regularization = lambda_reg * MatrixXd::Identity(degree + 1, degree + 1);

        // Solve weighted least squares
        coefficients = VectorXd(degree + 1);
        coefficients = (AtW * A + regularization).ldlt().solve(AtW * b);

        return true;
    }

    void printSystemState(int degree) {
        Serial.println("\n--- Adaptive Random Interval Measurement System ---");
        
        // Detailed system state
        Serial.printf("Noise Floor: %.4f\n", x(NOISE_FLOOR));
        Serial.printf("Measurement Bias: %.4f\n", x(MEASUREMENT_BIAS));
        Serial.printf("System Gain: %.4f\n", x(SYSTEM_GAIN));
        
        Serial.printf("\nSampling Frequency: %.2f Hz\n", x(SAMPLING_FREQUENCY));
        Serial.printf("Frequency Variance: %.4f\n", x(FREQUENCY_VARIANCE));

        // Polynomial coefficients
        Serial.println("\nPolynomial Coefficients:");
        for (int i = 0; i <= degree; i++) {
            Serial.printf("a%d: %.4f\n", i, coefficients(i));
        }
    }

    void reset() {
        dataPointCount = 0;
        samplingIntervals.clear();
        lastMeasurementTime = 0;
        // Retain Kalman filter state for continuity
    }
};

AdaptiveRandomIntervalMeasurementSystem measurementSystem;

void setup() {
    Serial.begin(115200);
    
    // Simulate multiple measurement cycles with random intervals
    for (int cycle = 0; cycle < 3; cycle++) {
        Serial.printf("\n--- Measurement Cycle %d ---\n", cycle);
        
        randomSeed(analogRead(0));
        
        // Generate measurements with random, non-uniform intervals
        for (int i = 0; i < 50; i++) {
            // Random interval between measurements (50-500 ms)
            delay(random(50, 500));
            
            // Simulate non-linear, noisy sensor signal
            unsigned long currentTime = millis();
            double timestamp = currentTime / 1000.0;
            
            // Complex signal with noise
            double baseSignal = 2.0 + 0.5 * timestamp - 0.1 * timestamp * timestamp;
            double noise = random(-100, 100) / 100.0 * (1 + 0.05 * timestamp);
            
            measurementSystem.update(baseSignal + noise, currentTime);
        }
        
        // Perform adaptive polynomial fitting
        int degree = 2;
        if (measurementSystem.performAdaptiveFitting(degree)) {
            measurementSystem.printSystemState(degree);
        }
        
        // Prepare for next cycle
        measurementSystem.reset();
    }
}

void loop() {
    delay(10000);  // Wait before potential repeats
}
