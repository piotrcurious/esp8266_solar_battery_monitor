#include <math.h>

#define BUFFER_SIZE 4

struct MeasurementMonitor {
    float values[BUFFER_SIZE];      // Rolling buffer for measurements
    unsigned long times[BUFFER_SIZE]; // Rolling buffer for timestamps
    float errors[4];               // Errors for each candidate model
    int count;                     // Number of samples in the buffer
    float threshold;               // Allowed deviation threshold
    float shadowValue;             // Shadowed (smoothed) value
    float noiseModel;              // Estimated noise level
    float driftModel;              // Estimated drift rate
    float weights[4];              // Dynamic weights for candidate models
    float confidence[4];           // Confidence for candidate models
    float derivatives[BUFFER_SIZE - 1]; // Stores calculated derivatives
};

// Initialize the monitor
MeasurementMonitor monitor = {
    {0}, {0}, {0}, 0, 0.1, 0.0, 0.0, {0.25, 0.25, 0.25, 0.25}, {1.0, 1.0, 1.0, 1.0}, {0}
};

// Add a new sample to the rolling buffer
void addSample(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    if (mon.count < BUFFER_SIZE) {
        mon.values[mon.count] = measurement;
        mon.times[mon.count] = currentTime;
        mon.count++;
    } else {
        // Shift values and times to make space for the new sample
        for (int i = 1; i < BUFFER_SIZE; i++) {
            mon.values[i - 1] = mon.values[i];
            mon.times[i - 1] = mon.times[i];
        }
        mon.values[BUFFER_SIZE - 1] = measurement;
        mon.times[BUFFER_SIZE - 1] = currentTime;
    }
}

// Calculate derivatives using nonlinear regression
void calculateDerivatives(MeasurementMonitor &mon) {
    if (mon.count < 2) return; // Not enough samples
    for (int i = 0; i < mon.count - 1; i++) {
        unsigned long dt = mon.times[i + 1] - mon.times[i];
        if (dt > 0) {
            mon.derivatives[i] = (mon.values[i + 1] - mon.values[i]) / dt;
        } else {
            mon.derivatives[i] = 0.0; // Avoid division by zero
        }
    }
}

// Candidate models for prediction
float linearModelPrediction(MeasurementMonitor &mon) {
    if (mon.count < 2) return mon.values[mon.count - 1];
    float slope = mon.derivatives[mon.count - 2];
    unsigned long dt = mon.times[mon.count - 1] - mon.times[mon.count - 2];
    return mon.values[mon.count - 1] + slope * dt;
}

float polynomialModelPrediction(MeasurementMonitor &mon) {
    if (mon.count < 3) return linearModelPrediction(mon);
    // Quadratic fit using the last three points
    float x1 = mon.times[mon.count - 3], x2 = mon.times[mon.count - 2], x3 = mon.times[mon.count - 1];
    float y1 = mon.values[mon.count - 3], y2 = mon.values[mon.count - 2], y3 = mon.values[mon.count - 1];
    float a = (y3 - (x3 * (y2 - y1) + x2 * y1 - x1 * y2) / (x2 - x1)) / ((x3 - x2) * (x3 - x1));
    float b = (y2 - y1) / (x2 - x1) - a * (x2 + x1);
    float c = y1 - a * x1 * x1 - b * x1;
    unsigned long dt = mon.times[mon.count - 1] - x3;
    return a * dt * dt + b * dt + c;
}

float kalmanModelPrediction(float measurement, MeasurementMonitor &mon) {
    static float state = 0.0; // Kalman state
    static float uncertainty = 1.0; // Kalman uncertainty
    float processNoise = 1e-2;
    float measurementNoise = mon.noiseModel > 0 ? mon.noiseModel : 1e-1;

    // Prediction
    float predictedState = state;
    float predictedUncertainty = uncertainty + processNoise;

    // Update
    float gain = predictedUncertainty / (predictedUncertainty + measurementNoise);
    state = predictedState + gain * (measurement - predictedState);
    uncertainty = (1 - gain) * predictedUncertainty;

    return state;
}

// Weighted prediction combining models
float weightedPrediction(MeasurementMonitor &mon) {
    float linearPred = linearModelPrediction(mon);
    float polyPred = polynomialModelPrediction(mon);
    float smoothPred = kalmanModelPrediction(mon.values[mon.count - 1], mon);
    float avgNoise = sqrt(mon.noiseModel);

    float predictions[] = {linearPred, polyPred, smoothPred};
    for (int i = 0; i < 3; i++) {
        float error = fabs(mon.values[mon.count - 1] - predictions[i]);
        mon.errors[i] = error;
        mon.weights[i] = exp(-error / (avgNoise + 1e-6)) * mon.confidence[i];
    }
    float totalWeight = mon.weights[0] + mon.weights[1] + mon.weights[2];
    return (mon.weights[0] * linearPred + mon.weights[1] * polyPred + mon.weights[2] * smoothPred) / totalWeight;
}

// Main function to monitor measurements
float monitorMeasurement(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addSample(measurement, currentTime, monitor);
    calculateDerivatives(monitor);

    if (monitor.count < 2) {
        timeWithinBounds = INFINITY;
        return 0.0;
    }

    float predictedValue = weightedPrediction(monitor);
    float deviationRate = fabs(predictedValue - measurement) / fabs(measurement);

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

