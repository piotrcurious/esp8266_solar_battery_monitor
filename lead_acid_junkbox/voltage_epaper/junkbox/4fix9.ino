#include <limits>

// Structure for measurement monitoring
struct MeasurementMonitor {
    static constexpr int MAX_SAMPLES = 4;

    float values[MAX_SAMPLES] = {0};
    unsigned long times[MAX_SAMPLES] = {0};
    float derivatives[MAX_SAMPLES - 1] = {0}; // First-order derivatives
    int count = 0;                           // Number of valid samples
    float noiseModel = 0.0;                  // Exponential moving average of noise
    float driftModel = 0.0;                  // Exponential moving average of drift
    float threshold = 0.05;                  // Allowed deviation threshold (e.g., ±5%)
};

// Add a new sample to the rolling buffer
void addSample(float measurement, unsigned long currentTime, MeasurementMonitor &monitor) {
    // Shift data to make room for the new sample
    for (int i = monitor.count - 1; i >= 0; i--) {
        if (i + 1 < MeasurementMonitor::MAX_SAMPLES) {
            monitor.values[i + 1] = monitor.values[i];
            monitor.times[i + 1] = monitor.times[i];
        }
    }

    // Add the new sample
    monitor.values[0] = measurement;
    monitor.times[0] = currentTime;

    // Update sample count
    if (monitor.count < MeasurementMonitor::MAX_SAMPLES) {
        monitor.count++;
    }
}

// Calculate derivatives for the rolling buffer
void calculateDerivatives(MeasurementMonitor &monitor) {
    if (monitor.count < 2) return; // Not enough data for derivatives

    for (int i = 0; i < monitor.count - 1; i++) {
        unsigned long timeDiff = 
            (monitor.times[i] + (ULONG_MAX - monitor.times[i + 1] + 1)) % ULONG_MAX;
        float deltaTime = timeDiff / 1000.0; // Convert ms to seconds
        if (deltaTime > 1e-6) {
            monitor.derivatives[i] = (monitor.values[i] - monitor.values[i + 1]) / deltaTime;
        } else {
            monitor.derivatives[i] = 0; // Prevent division by zero
        }
    }

    // Clear unused derivative slots
    for (int i = monitor.count - 1; i < MeasurementMonitor::MAX_SAMPLES - 1; i++) {
        monitor.derivatives[i] = 0;
    }
}

// Predict the next value using weighted derivatives
float weightedPrediction(const MeasurementMonitor &monitor) {
    if (monitor.count < 2) return monitor.values[0];

    unsigned long deltaTime = monitor.times[0] - monitor.times[1];
    float deltaTimeSec = deltaTime / 1000.0;

    float prediction = monitor.values[0] + monitor.derivatives[0] * deltaTimeSec;

    if (monitor.count >= 3) {
        float deltaDerivative = monitor.derivatives[0] - monitor.derivatives[1];
        float secondDerivative = deltaDerivative / deltaTimeSec;

        // Smooth second derivative
        secondDerivative = 0.9 * secondDerivative +
                           0.1 * (monitor.derivatives[1] - monitor.derivatives[2]) / deltaTimeSec;

        prediction += 0.5 * secondDerivative * deltaTimeSec * deltaTimeSec;
    }

    return prediction;
}

// Monitor measurement and calculate deviation rate and time within bounds
float monitorMeasurement(float measurement, unsigned long currentTime, 
                         float &timeWithinBounds, MeasurementMonitor &monitor) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = std::numeric_limits<float>::infinity();
        return 0.0;
    }

    // Calculate predicted value
    float predictedValue = weightedPrediction(monitor);

    // Deviation rate calculation
    float deviationRate = 0.0;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation for near-zero measurement
    }

    // Update noise model (capped exponential moving average)
    if (!std::isnan(deviationRate) && !std::isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * std::min(fabs(predictedValue - measurement), 10.0f);
    }

    // Update drift model (significant changes only)
    if (monitor.count >= 2 && fabs(monitor.derivatives[0]) > 1e-3) {
        monitor.driftModel = 0.9 * monitor.driftModel + 0.1 * monitor.derivatives[0];
    }

    // Calculate time within bounds
    float upperBound = monitor.values[0] * (1.0 + monitor.threshold + monitor.noiseModel);
    float lowerBound = monitor.values[0] * (1.0 - monitor.threshold - monitor.noiseModel);
    float timeToUpper = std::numeric_limits<float>::infinity();
    float timeToLower = std::numeric_limits<float>::infinity();

    float effectiveDrift = monitor.driftModel + (monitor.driftModel == 0 ? 1e-6 : 0); // Add bias if zero
    if (effectiveDrift > 0) {
        timeToUpper = (upperBound - monitor.values[0]) / effectiveDrift;
    } else if (effectiveDrift < 0) {
        timeToLower = (lowerBound - monitor.values[0]) / effectiveDrift;
    }

    // Select the valid time-to-bound
    timeWithinBounds = std::fmin(
        std::fmax(0, timeToUpper),
        std::fmax(0, timeToLower)
    );

    // Correct invalid results
    if (std::isnan(timeWithinBounds) || std::isinf(timeWithinBounds)) {
        timeWithinBounds = std::numeric_limits<float>::infinity();
    }

    return deviationRate;
}
