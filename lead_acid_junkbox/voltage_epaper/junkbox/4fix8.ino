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
    float deviationRate = 0.0;
    if (fabs(measurement) > 1e-6) {
        deviationRate = fabs(predictedValue - measurement) / fabs(measurement);
    } else {
        deviationRate = fabs(predictedValue - measurement); // Absolute deviation for near-zero measurement
    }

    // Update noise model (exponential moving average)
    if (!std::isnan(deviationRate) && !std::isinf(deviationRate)) {
        monitor.noiseModel = 0.9 * monitor.noiseModel + 0.1 * fabs(predictedValue - measurement);
    }

    // Update drift model (exponential moving average)
    if (monitor.count >= 2) {
        monitor.driftModel = 0.9 * monitor.driftModel + 0.1 * monitor.derivatives[0];
    }

    // Calculate time within bounds
    float upperBound = monitor.values[0] * (1.0 + monitor.threshold + monitor.noiseModel);
    float lowerBound = monitor.values[0] * (1.0 - monitor.threshold - monitor.noiseModel);
    float timeToUpper = std::numeric_limits<float>::infinity();
    float timeToLower = std::numeric_limits<float>::infinity();

    // Handle positive drift
    if (monitor.driftModel > 0) {
        if (upperBound > monitor.values[0]) {
            timeToUpper = (upperBound - monitor.values[0]) / monitor.driftModel;
        }
        timeToLower = std::numeric_limits<float>::infinity(); // Signal cannot reach lower bound when drifting up
    }
    // Handle negative drift
    else if (monitor.driftModel < 0) {
        if (lowerBound < monitor.values[0]) {
            timeToLower = (lowerBound - monitor.values[0]) / monitor.driftModel;
        }
        timeToUpper = std::numeric_limits<float>::infinity(); // Signal cannot reach upper bound when drifting down
    }

    // Handle stable signal (zero drift)
    if (fabs(monitor.driftModel) <= 1e-6) {
        timeToUpper = std::numeric_limits<float>::infinity();
        timeToLower = std::numeric_limits<float>::infinity();
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
