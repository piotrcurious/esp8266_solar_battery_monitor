float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0;
    }

    float predictedValue = weightedPrediction(monitor);
    
    // Updated deviation calculation
    float deviationRate;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement);
    }

    // Noise model update
    monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * (deviationRate * deviationRate);

    // Drift estimation
    monitor.driftModel = monitor.derivatives[monitor.count - 2];

    // Time within bounds prediction
    float currentValue = monitor.values[monitor.count - 1];
    float upperBound = currentValue * (1.0 + monitor.threshold);
    float lowerBound = currentValue * (1.0 - monitor.threshold);
    float timeToUpper = (upperBound - currentValue) / monitor.driftModel;
    float timeToLower = (lowerBound - currentValue) / monitor.driftModel;

    timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    return deviationRate;
}
