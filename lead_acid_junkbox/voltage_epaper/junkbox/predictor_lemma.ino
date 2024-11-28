#include <math.h>

struct MeasurementMonitor {
    float values[4];             // Rolling buffer for the last four measurements
    unsigned long times[4];      // Corresponding timestamps
    float predictions[4];        // Predicted values
    float errors[4];             // Prediction errors
    int count;                   // Current number of stored samples (max 4)
    float threshold;             // Threshold for deviation rate
    float shadowValue;           // Shadowed (smoothed) value
    float noiseModel;            // Estimated noise level (variance of errors)
    float driftModel;            // Estimated drift (trend of differences)
};

// Initialize the monitor
MeasurementMonitor monitor = {
    {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
    0, 0.5, 0.0, 0.0, 0.0
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

// Calculate noise and drift models
void updateNoiseAndDriftModels(MeasurementMonitor &mon) {
    if (mon.count < 2) {
        mon.noiseModel = 0.0;
        mon.driftModel = 0.0;
        return;
    }

    // Calculate drift (average delta) and noise (variance of errors)
    float sumDeltas = 0.0, sumSquaredErrors = 0.0;
    int n = mon.count - 1;

    for (int i = 0; i < n; i++) {
        float delta = mon.values[i + 1] - mon.values[i];
        float error = mon.errors[i];
        sumDeltas += delta;
        sumSquaredErrors += error * error;
    }

    mon.driftModel = sumDeltas / n; // Mean drift
    mon.noiseModel = sqrt(sumSquaredErrors / n); // Noise as standard deviation of errors
}

// Update shadow value
void updateShadowValue(MeasurementMonitor &mon) {
    if (mon.count < 2) {
        mon.shadowValue = mon.values[mon.count - 1];
        return;
    }

    // Combine drift, noise, and previous shadow value
    float driftAdjustment = mon.driftModel;
    float noiseAdjustment = mon.noiseModel;

    mon.shadowValue = mon.shadowValue + driftAdjustment + noiseAdjustment;
}

// Predict the next value based on shadow model
float predictNextValue(MeasurementMonitor &mon) {
    if (mon.count < 2) {
        return mon.values[mon.count - 1];
    }

    float predictedValue = mon.shadowValue + mon.driftModel;
    return predictedValue;
}

// Calculate deviation rate and update errors
float calculateDeviationRate(float measurement, MeasurementMonitor &mon) {
    float predictedValue = predictNextValue(mon);
    float error = fabs(predictedValue - measurement);

    // Update error history
    for (int i = 1; i < mon.count; i++) {
        mon.errors[i - 1] = mon.errors[i];
    }
    mon.errors[mon.count - 1] = error;

    return error / fabs(measurement);
}

// Predict the time within bounds
float predictTimeWithinBounds(MeasurementMonitor &mon, float deviationRate) {
    if (fabs(mon.driftModel) < 1e-6) {
        return INFINITY; // Signal is stable
    }

    float currentValue = mon.values[mon.count - 1];
    float upperBound = currentValue * (1.0 + mon.threshold);
    float lowerBound = currentValue * (1.0 - mon.threshold);

    float timeToUpper = (upperBound - currentValue) / mon.driftModel;
    float timeToLower = (lowerBound - currentValue) / mon.driftModel;

    if (timeToUpper < 0 && timeToLower < 0) {
        return 0; // Signal already out of bounds
    }

    return min(timeToUpper > 0 ? timeToUpper : INFINITY, timeToLower > 0 ? timeToLower : INFINITY);
}

// Main function to check and update measurements
float checkMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addToRollingBuffer(measurement, currentTime, monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY; // Insufficient data
        return 0.0;
    }

    updateNoiseAndDriftModels(monitor);
    updateShadowValue(monitor);

    float deviationRate = calculateDeviationRate(measurement, monitor);
    timeWithinBounds = predictTimeWithinBounds(monitor, deviationRate);

    return deviationRate;
}

void setup() {
    Serial.begin(9600);
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
