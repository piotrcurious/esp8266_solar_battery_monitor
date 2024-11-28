float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0;
    }

    // Calculate weighted predicted value
    float predictedValue = weightedPrediction(monitor);

    // Deviation rate calculation
    float deviationRate = 0.0;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation for zero measurement
    }

    // Update noise model (based on recent deviation rate)
    if (!isnan(deviationRate) && !isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * deviationRate;
    }

    // Update drift model based on recent derivatives
    if (monitor.count >= 2) {
        float recentDrift = monitor.derivatives[monitor.count - 2];
        monitor.driftModel = 0.9 * monitor.driftModel + 0.1 * recentDrift;
    }

    // Predict time within bounds
    float currentValue = monitor.values[monitor.count - 1];
    float upperBound = currentValue * (1.0 + monitor.threshold + monitor.noiseModel);
    float lowerBound = currentValue * (1.0 - monitor.threshold - monitor.noiseModel);

    float timeToUpper = INFINITY, timeToLower = INFINITY;
    if (fabs(monitor.driftModel) > 1e-6) {
        timeToUpper = (upperBound - currentValue) / monitor.driftModel;
        timeToLower = (lowerBound - currentValue) / monitor.driftModel;
    }

    // Ensure time-to-bounds is meaningful
    timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    if (isnan(timeWithinBounds) || isinf(timeWithinBounds)) {
        timeWithinBounds = INFINITY; // Fallback for invalid calculations
    }

    return deviationRate; // Return calculated deviation rate
}
