#include <cmath>
#include <limits>

struct MeasurementMonitor {
    float values[4];          // Rolling buffer for measured values
    unsigned long times[4];   // Corresponding timestamps
    float derivatives[3];     // Rolling buffer for derivatives
    float noiseModel;         // Noise model estimate
    float driftModel;         // Drift model estimate
    float threshold;          // Threshold for deviation
    int count;                // Number of samples currently stored
};

// Initialize the monitor
void initMonitor(MeasurementMonitor &monitor, float threshold) {
    for (int i = 0; i < 4; i++) {
        monitor.values[i] = 0;
        monitor.times[i] = 0;
        if (i < 3) monitor.derivatives[i] = 0;
    }
    monitor.noiseModel = 0;
    monitor.driftModel = 0;
    monitor.threshold = threshold;
    monitor.count = 0;
}

// Add a new sample to the monitor
void addSample(float measurement, unsigned long currentTime, MeasurementMonitor &monitor) {
    // Shift rolling buffer
    for (int i = 3; i > 0; i--) {
        monitor.values[i] = monitor.values[i - 1];
        monitor.times[i] = monitor.times[i - 1];
    }

    // Add the new sample
    monitor.values[0] = measurement;
    monitor.times[0] = currentTime;

    // Update count (max 4)
    if (monitor.count < 4) monitor.count++;
}

// Calculate derivatives based on the rolling buffer
void calculateDerivatives(MeasurementMonitor &monitor) {
    if (monitor.count < 2) return; // Not enough data for derivatives

    for (int i = 0; i < monitor.count - 1; i++) {
        float deltaTime = (monitor.times[i] - monitor.times[i + 1]) / 1000.0; // Convert ms to seconds
        if (deltaTime > 1e-6) {
            monitor.derivatives[i] = (monitor.values[i] - monitor.values[i + 1]) / deltaTime;
        } else {
            monitor.derivatives[i] = 0; // Prevent division by zero
        }
    }
}

// Predict signal value using higher-order derivatives
float weightedPrediction(const MeasurementMonitor &monitor) {
    if (monitor.count < 2) return monitor.values[0]; // Not enough data to predict

    float deltaTime = (monitor.times[0] - monitor.times[1]) / 1000.0; // Time difference between last two samples
    float prediction = monitor.values[0];

    // Apply first-order derivative (drift)
    prediction += deltaTime * monitor.derivatives[0];

    // Apply second-order derivative (acceleration)
    if (monitor.count >= 3) {
        float secondDerivative = (monitor.derivatives[0] - monitor.derivatives[1]) / deltaTime;
        prediction += 0.5 * secondDerivative * deltaTime * deltaTime;
    }

    return prediction;
}

// Monitor measurement and compute deviation and time within bounds
float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds, MeasurementMonitor &monitor) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = std::numeric_limits<float>::infinity();
        return 0.0;
    }

    // Calculate predicted value
    float predictedValue = weightedPrediction(monitor);

    // Deviation rate calculation
    float deviationRate;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation for near-zero measurement
    }

    // Update noise model
    if (!std::isnan(deviationRate) && !std::isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * fabs(predictedValue - measurement);
    }

    // Update drift model
    if (monitor.count >= 2) {
        monitor.driftModel = 0.9 * monitor.driftModel + 0.1 * monitor.derivatives[0];
    }

    // Calculate time within bounds
    float upperBound = monitor.values[0] * (1.0 + monitor.threshold + monitor.noiseModel);
    float lowerBound = monitor.values[0] * (1.0 - monitor.threshold - monitor.noiseModel);
    float timeToUpper = std::numeric_limits<float>::infinity();
    float timeToLower = std::numeric_limits<float>::infinity();

    if (fabs(monitor.driftModel) > 1e-6) {
        timeToUpper = (upperBound - monitor.values[0]) / monitor.driftModel;
        timeToLower = (lowerBound - monitor.values[0]) / monitor.driftModel;
    }

    // Adjust time-to-bounds based on drift direction
    if (monitor.driftModel > 0) {
        timeToLower = std::numeric_limits<float>::infinity();
    } else if (monitor.driftModel < 0) {
        timeToUpper = std::numeric_limits<float>::infinity();
    }

    timeWithinBounds = std::fmin(std::fmax(0, timeToUpper), std::fmax(0, timeToLower));
    if (std::isnan(timeWithinBounds) || std::isinf(timeWithinBounds)) {
        timeWithinBounds = std::numeric_limits<float>::infinity();
    }

    return deviationRate;
}
