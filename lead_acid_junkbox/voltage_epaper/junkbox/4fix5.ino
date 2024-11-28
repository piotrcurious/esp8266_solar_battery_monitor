float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0; // Not enough data to calculate
    }

    // Weighted prediction using higher-order derivatives
    float predictedValue = monitor.values[monitor.count - 1];
    float deltaTime = (monitor.timestamps[monitor.count - 1] - monitor.timestamps[monitor.count - 2]) / 1000.0;

    // First-order derivative (drift)
    if (monitor.count >= 2) {
        predictedValue += deltaTime * monitor.derivatives[monitor.count - 2];
    }

    // Second-order derivative (acceleration)
    if (monitor.count >= 3) {
        float secondDerivative = (monitor.derivatives[monitor.count - 2] - monitor.derivatives[monitor.count - 3]) / deltaTime;
        predictedValue += 0.5 * secondDerivative * deltaTime * deltaTime;
    }

    // Calculate deviation rate with fallback for zero measurement
    float deviationRate;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation if measurement is near zero
    }

    // Update noise model
    if (!isnan(deviationRate) && !isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * fabs(predictedValue - measurement);
    }

    // Drift estimation with directionality
    if (monitor.count >= 2) {
        float recentDrift = monitor.derivatives[monitor.count - 2];
        monitor.driftModel = 0.9 * monitor.driftModel + 0.1 * recentDrift;
    }

    // Time-to-bounds estimation
    float currentValue = monitor.values[monitor.count - 1];
    float upperBound = currentValue * (1.0 + monitor.threshold + monitor.noiseModel);
    float lowerBound = currentValue * (1.0 - monitor.threshold - monitor.noiseModel);
    float timeToUpper = INFINITY, timeToLower = INFINITY;

    if (fabs(monitor.driftModel) > 1e-6) {
        timeToUpper = (upperBound - currentValue) / monitor.driftModel;
        timeToLower = (lowerBound - currentValue) / monitor.driftModel;
    }

    // Correct time predictions based on drift direction
    if (monitor.driftModel > 0) {
        timeToLower = INFINITY; // Signal drifting upward, lower bound won't be reached
    } else if (monitor.driftModel < 0) {
        timeToUpper = INFINITY; // Signal drifting downward, upper bound won't be reached
    }

    timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    if (isnan(timeWithinBounds) || isinf(timeWithinBounds)) {
        timeWithinBounds = INFINITY; // Fallback for invalid predictions
    }

    return isnan(deviationRate) ? 0.0 : deviationRate; // Return valid deviation rate
}
