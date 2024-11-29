float modelEvolution(float currentValue, unsigned long currentTime, float horizon, int modelIndex, MeasurementMonitor &mon) {
    float predictedValue = currentValue;
    float weightedVelocity = 0.0f;
    float weightedAcceleration = 0.0f;
    float totalWeight = 0.0f;

    // Calculate weighted velocity (first derivative) and acceleration (second derivative)
    for (int i = 0; i < mon.count; i++) {
        float weight = 1.0f / (1.0f + fabs(currentTime - mon.timestamps[i])); // Time-based weighting
        weightedVelocity += mon.derivatives[i] * weight;
        weightedAcceleration += mon.secondDerivatives[i] * weight;
        totalWeight += weight;
    }

    if (totalWeight > 0) {
        weightedVelocity /= totalWeight;
        weightedAcceleration /= totalWeight;
    }

    // Model-specific evolution
    switch (modelIndex) {
        case 0: // Constant drift model
            predictedValue += weightedVelocity * horizon;
            break;

        case 1: // Linear trend model
            predictedValue += (weightedVelocity + 0.5f * weightedAcceleration * horizon) * horizon;
            break;

        case 2: // Exponential growth model
            if (weightedVelocity > 0) {
                float growthRate = weightedVelocity / currentValue;
                predictedValue *= exp(growthRate * horizon / 1000.0f);
            }
            break;

        case 3: // Noise-adjusted model
            float noiseAdjustment = mon.noiseModel; // Incorporate noise model adjustment
            predictedValue += (weightedVelocity + noiseAdjustment) * horizon;
            break;

        default:
            break;
    }

    return predictedValue;
}

float weightedEvolution(float currentValue, unsigned long currentTime, float horizon, MeasurementMonitor &mon) {
    float weightedPrediction = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < 4; i++) {
        float modelPrediction = modelEvolution(currentValue, currentTime, horizon, i, mon);
        weightedPrediction += modelPrediction * mon.weights[i];
        totalWeight += mon.weights[i];
    }

    return (totalWeight > 0) ? (weightedPrediction / totalWeight) : currentValue;
}
