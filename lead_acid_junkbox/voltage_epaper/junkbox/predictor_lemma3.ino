#include <math.h>

struct MeasurementMonitor {
    float values[4];              // Rolling buffer for the last four measurements
    unsigned long times[4];       // Corresponding timestamps
    float errors[4];              // Prediction errors
    int count;                    // Current number of stored samples (max 4)
    float threshold;              // Threshold for deviation rate
    float shadowValue;            // Shadowed (smoothed) value
    float noiseModel;             // Estimated noise level (variance of errors)
    float driftModel;             // Estimated drift (trend of differences)
    float weights[4];             // Weights for candidate models
    float confidence[4];          // Confidence levels for candidate models
};

// Initialize the monitor
MeasurementMonitor monitor = {
    {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
    0, 0.5, 0.0, 0.0, {0.25, 0.25, 0.25, 0.25}, {1.0, 1.0, 1.0, 1.0}
};

// Add a new measurement to the rolling buffer
void addToRollingBuffer(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    if (mon.count < 4) {
        mon.values[mon.count] = measurement;
        mon.times[mon.count] = currentTime;
        mon.count++;
    } else {
        for (int i = 1; i < 4; i++) {
            mon.values[i - 1] = mon.values[i];
            mon.times[i - 1] = mon.times[i];
        }
        mon.values[3] = measurement;
        mon.times[3] = currentTime;
    }
}

// Kalman filter-based smoothing
float kalmanPrediction(float measurement, MeasurementMonitor &mon, float processNoise = 1e-2, float measurementNoise = 1e-1) {
    static float state = 0.0; // State estimate
    static float uncertainty = 1.0; // State uncertainty

    // Prediction
    float predictedState = state;
    float predictedUncertainty = uncertainty + processNoise;

    // Update
    float gain = predictedUncertainty / (predictedUncertainty + measurementNoise);
    state = predictedState + gain * (measurement - predictedState);
    uncertainty = (1 - gain) * predictedUncertainty;

    return state;
}

// Weighted candidate combination with confidence
float weightedPrediction(MeasurementMonitor &mon) {
    float linearPred = linearRegressionPrediction(mon);
    float polyPred = polynomialRegressionPrediction(mon);
    float smoothPred = exponentialSmoothingPrediction(mon);
    float kalmanPred = kalmanPrediction(mon.values[mon.count - 1], mon);

    // Adjust weights dynamically based on errors and confidence
    float totalWeight = 0.0;
    for (int i = 0; i < 4; i++) {
        mon.weights[i] = exp(-mon.errors[i] / (mon.noiseModel + 1e-6)) * mon.confidence[i];
        totalWeight += mon.weights[i];
    }
    for (int i = 0; i < 4; i++) {
        mon.weights[i] /= totalWeight; // Normalize
    }

    // Combine predictions
    return mon.weights[0] * linearPred +
           mon.weights[1] * polyPred +
           mon.weights[2] * smoothPred +
           mon.weights[3] * kalmanPred;
}

// Main function to check measurements
float checkMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addToRollingBuffer(measurement, currentTime, monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY; // Insufficient data
        return 0.0;
    }

    float predictedValue = weightedPrediction(monitor);
    float deviationRate = fabs(predictedValue - measurement) / fabs(measurement);

    // Update noise model (variance of errors)
    for (int i = 0; i < monitor.count; i++) {
        float error = fabs(monitor.values[i] - predictedValue);
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * (error * error);
    }

    // Predict time within bounds using linear regression
    float drift = monitor.driftModel;
    if (fabs(drift) < 1e-6) {
        timeWithinBounds = INFINITY; // Stable
    } else {
        float currentValue = monitor.values[monitor.count - 1];
        float upperBound = currentValue * (1.0 + monitor.threshold);
        float lowerBound = currentValue * (1.0 - monitor.threshold);
        float timeToUpper = (upperBound - currentValue) / drift;
        float timeToLower = (lowerBound - currentValue) / drift;
        timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    }

    return deviationRate;
}
