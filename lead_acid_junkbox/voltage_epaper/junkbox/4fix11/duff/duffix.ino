#include <math.h>

#define BUFFER_SIZE 4

struct MeasurementMonitor {
    float values[BUFFER_SIZE];          // Rolling buffer for measurements
    float timestamps[BUFFER_SIZE];      // Precise time points in seconds
    float errors[4];                    // Errors for each candidate model
    int count;                          // Number of samples in the buffer
    float threshold;                    // Allowed deviation threshold
    float shadowValue;                  // Shadowed (smoothed) value
    float noiseModel;                   // Estimated noise level
    float driftModel;                   // Estimated drift rate
    float weights[4];                   // Dynamic weights for candidate models
    float confidence[4];                // Confidence for candidate models
    float derivatives[BUFFER_SIZE - 1];  // Stores calculated derivatives
    float secondDerivatives[BUFFER_SIZE - 2];  // Stores second derivatives
};

MeasurementMonitor monitor;

// Initialize the monitor
void initializeMonitor(MeasurementMonitor &mon) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mon.values[i] = 0.0f;
        mon.timestamps[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        mon.errors[i] = 0.001f;
        mon.weights[i] = 0.001f;
        mon.confidence[i] = 1.0f;
    }
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        mon.derivatives[i] = 0.0f;
    }
    for (int i = 0; i < BUFFER_SIZE - 2; i++) {
        mon.secondDerivatives[i] = 0.0f;
    }
    mon.count = 0;
    mon.threshold = 0.01f;
    mon.shadowValue = 0.0f;
    mon.noiseModel = 0.1f;
    mon.driftModel = 0.0f;
}

// Add a new sample to the rolling buffer
void addSample(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    float timeInSeconds = (float)currentTime / 1000.0f;
    if (mon.count < BUFFER_SIZE) {
        mon.values[mon.count] = measurement;
        mon.timestamps[mon.count] = timeInSeconds;
        mon.count++;
    } else {
        for (int i = 1; i < BUFFER_SIZE; i++) {
            mon.values[i - 1] = mon.values[i];
            mon.timestamps[i - 1] = mon.timestamps[i];
        }
        mon.values[BUFFER_SIZE - 1] = measurement;
        mon.timestamps[BUFFER_SIZE - 1] = timeInSeconds;
    }
}

// Calculate first and second derivatives
void calculateDerivatives(MeasurementMonitor &mon) {
    if (mon.count < 2) return; // Need at least 2 points for first derivatives
    for (int i = 0; i < mon.count - 1; i++) {
        float dt = mon.timestamps[i + 1] - mon.timestamps[i];
        mon.derivatives[i] = (dt > 1e-6) ? (mon.values[i + 1] - mon.values[i]) / dt : 0.0f;
    }
    if (mon.count < 3) return; // Need at least 3 points for second derivatives
    for (int i = 0; i < mon.count - 2; i++) {
        float dt = mon.timestamps[i + 2] - mon.timestamps[i];
        mon.secondDerivatives[i] = (dt > 1e-6) ? 
            (mon.derivatives[i + 1] - mon.derivatives[i]) / dt : 0.0f;
    }
}

// Linear trend prediction
float linearModelPrediction(const MeasurementMonitor &mon) {
    if (mon.count < 2) return mon.shadowValue;
    float dt = mon.timestamps[mon.count - 1] - mon.timestamps[mon.count - 2];
    return mon.values[mon.count - 1] + mon.derivatives[mon.count - 2] * dt;
}

// Polynomial model prediction
float polynomialModelPrediction(const MeasurementMonitor &mon) {
    if (mon.count < 3) return linearModelPrediction(mon);
    float x1 = mon.timestamps[mon.count - 3], x2 = mon.timestamps[mon.count - 2], x3 = mon.timestamps[mon.count - 1];
    float y1 = mon.values[mon.count - 3], y2 = mon.values[mon.count - 2], y3 = mon.values[mon.count - 1];
    float a = (y3 - (x3 * (y2 - y1) + x2 * y1 - x1 * y2) / (x2 - x1)) / ((x3 - x2) * (x3 - x1));
    float b = (y2 - y1) / (x2 - x1) - a * (x2 + x1);
    float c = y1 - a * x1 * x1 - b * x1;
    return a * x3 * x3 + b * x3 + c;
}

// Kalman filter prediction
float kalmanModelPrediction(float measurement, MeasurementMonitor &mon) {
    static float state = 0.0f, uncertainty = 1.0f;
    float processNoise = 0.01f, measurementNoise = mon.noiseModel;

    float predictedUncertainty = uncertainty + processNoise;
    float gain = predictedUncertainty / (predictedUncertainty + measurementNoise);
    state += gain * (measurement - state);
    uncertainty = (1 - gain) * predictedUncertainty;

    return state;
}

// Weighted model prediction
float weightedPrediction(const MeasurementMonitor &mon) {
    float prediction = 0.0f, totalWeight = 0.0f;
    float models[3] = {
        linearModelPrediction(mon),
        polynomialModelPrediction(mon),
        kalmanModelPrediction(mon.values[mon.count - 1], const_cast<MeasurementMonitor&>(mon))
    };
    for (int i = 0; i < 3; i++) {
        prediction += models[i] * mon.weights[i];
        totalWeight += mon.weights[i];
    }
    return (totalWeight > 0) ? prediction / totalWeight : mon.shadowValue;
}

// Monitor measurement
float monitorMeasurement(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    addSample(measurement, currentTime, mon);
    calculateDerivatives(mon);
    return weightedPrediction(mon);
}

void setup() {
    Serial.begin(9600);
    initializeMonitor(monitor);
}

void loop() {
    unsigned long currentTime = millis();
    float measurement = analogRead(A0) * (5.0 / 1023.0); // Example: convert ADC to voltage
    float prediction = monitorMeasurement(measurement, currentTime, monitor);
    Serial.println(prediction);
    delay(100);
}
