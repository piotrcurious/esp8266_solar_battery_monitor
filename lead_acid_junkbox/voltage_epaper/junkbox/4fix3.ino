float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0;
    }

    float predictedValue = weightedPrediction(monitor);

    // Updated deviation calculation to avoid NaN
    float deviationRate = 0.0;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation for zero measurement
    }

    // Noise model update with NaN safeguard
    if (!isnan(deviationRate) && !isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * (deviationRate * deviationRate);
    }

    // Drift estimation with safeguard
    monitor.driftModel = (monitor.count >= 2 && fabs(monitor.derivatives[monitor.count - 2]) > 1e-6)
                             ? monitor.derivatives[monitor.count - 2]
                             : 0.0;

    // Time within bounds prediction with safeguards
    float currentValue = monitor.values[monitor.count - 1];
    float upperBound = currentValue * (1.0 + monitor.threshold);
    float lowerBound = currentValue * (1.0 - monitor.threshold);
    float timeToUpper = (fabs(monitor.driftModel) > 1e-6) ? (upperBound - currentValue) / monitor.driftModel : INFINITY;
    float timeToLower = (fabs(monitor.driftModel) > 1e-6) ? (lowerBound - currentValue) / monitor.driftModel : INFINITY;

    if (!isnan(timeToUpper) && !isnan(timeToLower)) {
        timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    } else {
        timeWithinBounds = INFINITY;
    }

    return isnan(deviationRate) ? 0.0 : deviationRate; // Return 0 if deviation rate is invalid
}
