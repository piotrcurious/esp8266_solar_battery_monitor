// Predict the evolution of the system using weighted models
float weightedEvolution(float currentValue, unsigned long currentTime, float horizon, MeasurementMonitor &mon) {
    float weightedPrediction = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < 4; i++) {
        // Evolution based on each model's predicted behavior
        float modelPrediction = modelEvolution(currentValue, currentTime, horizon, i, mon);
        weightedPrediction += modelPrediction * mon.weights[i];
        totalWeight += mon.weights[i];
    }

    return (totalWeight > 0) ? (weightedPrediction / totalWeight) : currentValue;
}

// Evolution function for individual models
float modelEvolution(float currentValue, unsigned long currentTime, float horizon, int modelIndex, MeasurementMonitor &mon) {
    float predictedValue = currentValue;

    // Model-specific dynamics (simplified for illustration)
    switch (modelIndex) {
        case 0: // Constant drift model
            predictedValue += mon.driftModel * horizon;
            break;
        case 1: // Linear trend model
            predictedValue += mon.driftModel * horizon * 1.1;
            break;
        case 2: // Exponential growth model
            predictedValue *= pow(1.0 + mon.driftModel, horizon / 1000.0);
            break;
        case 3: // Custom noise-adjusted model
            predictedValue += (mon.driftModel + mon.noiseModel) * horizon;
            break;
    }

    return predictedValue;
}

// Monitor measurement with weighted predicted evolution
float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    // Add the new sample and update the monitor state
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0f; // Not enough data to calculate
    }

    // Refine measurement with local iterations for noise reduction
    float refined = refinedMeasurement(monitor, 10);
    addSample(refined, currentTime, monitor); // Add refined measurement to improve history

    // Predict the system's evolution using weighted models
    float predictionHorizon = 5000; // 5 seconds into the future
    float predictedValue = weightedEvolution(refined, currentTime, predictionHorizon, monitor);

    // Update noise model for enhanced accuracy
    updateNoiseModel(refined, predictedValue, monitor);

    // Calculate deviation rate for monitoring purposes
    float deviationRate = fabs(predictedValue - refined) / fabs(refined + 1e-6f);

    // Time-to-bounds estimation based on evolved predictions
    float currentValue = refined;
    float predictedDrift = predictedValue - currentValue;

    // Upper and lower bounds
    float upperBound = currentValue * (1.0f + monitor.threshold);
    float lowerBound = currentValue * (1.0f - monitor.threshold);

    // Time-to-bound estimations based on the predicted evolution
    float timeToUpper = INFINITY;
    float timeToLower = INFINITY;

    if (fabs(predictedDrift) > 1e-6f) {
        timeToUpper = (predictedDrift > 0) ? (upperBound - currentValue) / predictedDrift : INFINITY;
        timeToLower = (predictedDrift < 0) ? (lowerBound - currentValue) / predictedDrift : INFINITY;
    }

    // Compute the time within bounds as the minimum valid time
    timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));

    return deviationRate;
}
