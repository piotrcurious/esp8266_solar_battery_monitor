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
};

// Initialize the monitor
MeasurementMonitor monitor = {
    {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
    0, 0.5, 0.0, 0.0, {0.25, 0.25, 0.25, 0.25}
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

// Linear regression candidate
float linearRegressionPrediction(MeasurementMonitor &mon) {
    if (mon.count < 2) return mon.values[mon.count - 1];

    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    float deltaTime = (mon.times[mon.count - 1] - mon.times[0]) / 1000.0;

    for (int i = 0; i < mon.count; i++) {
        float x = (mon.times[i] - mon.times[0]) / 1000.0;
        sumX += x;
        sumY += mon.values[i];
        sumXY += x * mon.values[i];
        sumX2 += x * x;
    }

    float slope = (mon.count * sumXY - sumX * sumY) / (mon.count * sumX2 - sumX * sumX);
    float intercept = (sumY - slope * sumX) / mon.count;

    return intercept + slope * deltaTime;
}

// Polynomial regression candidate (quadratic)
float polynomialRegressionPrediction(MeasurementMonitor &mon) {
    if (mon.count < 3) return linearRegressionPrediction(mon);

    float x[4], y[4], sumX[5] = {0}, sumXY[3] = {0};
    for (int i = 0; i < mon.count; i++) {
        x[i] = (mon.times[i] - mon.times[0]) / 1000.0;
        y[i] = mon.values[i];
        for (int p = 0; p <= 4; p++) sumX[p] += pow(x[i], p);
        for (int p = 0; p <= 2; p++) sumXY[p] += y[i] * pow(x[i], p);
    }

    float denom = sumX[4] * (sumX[2] * mon.count - sumX[1] * sumX[1]) -
                  sumX[3] * (sumX[3] * mon.count - sumX[1] * sumX[2]) +
                  sumX[2] * (sumX[3] * sumX[1] - sumX[2] * sumX[2]);

    if (fabs(denom) < 1e-6) return linearRegressionPrediction(mon);

    float a = (sumXY[2] * (sumX[2] * mon.count - sumX[1] * sumX[1]) -
               sumXY[1] * (sumX[3] * mon.count - sumX[1] * sumX[2]) +
               sumXY[0] * (sumX[3] * sumX[1] - sumX[2] * sumX[2])) /
              denom;

    float b = (sumX[4] * (sumXY[1] * mon.count - sumXY[0] * sumX[1]) -
               sumX[3] * (sumXY[2] * mon.count - sumXY[0] * sumX[2]) +
               sumX[2] * (sumXY[2] * sumX[1] - sumXY[1] * sumX[2])) /
              denom;

    float c = (sumX[4] * (sumX[2] * sumXY[0] - sumX[1] * sumXY[1]) -
               sumX[3] * (sumX[3] * sumXY[0] - sumX[1] * sumXY[2]) +
               sumX[2] * (sumX[3] * sumXY[1] - sumX[2] * sumXY[2])) /
              denom;

    float xLast = (mon.times[mon.count - 1] - mon.times[0]) / 1000.0;
    return a * xLast * xLast + b * xLast + c;
}

// Exponential smoothing candidate
float exponentialSmoothingPrediction(MeasurementMonitor &mon, float alpha = 0.5) {
    if (mon.count < 2) return mon.values[mon.count - 1];

    float smoothedValue = mon.values[0];
    for (int i = 1; i < mon.count; i++) {
        smoothedValue = alpha * mon.values[i] + (1 - alpha) * smoothedValue;
    }
    return smoothedValue;
}

// Weighted model combination
float weightedPrediction(MeasurementMonitor &mon) {
    float linearPred = linearRegressionPrediction(mon);
    float polyPred = polynomialRegressionPrediction(mon);
    float smoothPred = exponentialSmoothingPrediction(mon);

    return mon.weights[0] * linearPred + mon.weights[1] * polyPred + mon.weights[2] * smoothPred;
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

    // Update weights based on prediction accuracy
    for (int i = 0; i < 3; i++) {
        monitor.weights[i] *= exp(-fabs(predictedValue - measurement));
    }
    float sumWeights = monitor.weights[0] + monitor.weights[1] + monitor.weights[2];
    for (int i = 0; i < 3; i++) {
        monitor.weights[i] /= sumWeights;
    }

    // Predict time within bounds using linear regression
    float drift = monitor.driftModel;
    if (fabs(drift) < 1e-6) {
        timeWithinBounds = INFINITY; // Stable
    } else {
        float currentValue = mon.values[mon.count - 1];
        float upperBound = currentValue * (1.0 + monitor.threshold);
        float lowerBound = currentValue * (1.0 - monitor.threshold);
        float timeToUpper = (upperBound - currentValue) / drift;
        float timeToLower = (lowerBound - currentValue) / drift;
        timeWithinBounds = fmin(fmax(0, timeToUpper), fmax(0, timeToLower));
    }

    return deviationRate;
}
void loop() {
    float measurement = analogRead(A0) * (5.0 / 1023.0); // Convert ADC to voltage
    unsigned long currentTime = millis();

    float timeWithinBounds;
    float deviationRate = checkMeasurement(measurement, currentTime, timeWithinBounds);

    Serial.print("Deviation Rate: ");
    Serial.println(deviationRate);
    Serial.print("Time Within Bounds: ");
    if (isinf(timeWithinBounds)) {
        Serial.println("Infinite (Stable)");
    } else {
        Serial.print(timeWithinBounds);
        Serial.println(" seconds");
    }

    delay(random(50, 150)); // Simulate variable sampling rate
}
