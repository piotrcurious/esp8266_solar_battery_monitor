
#include <math.h>

#define BUFFER_SIZE 4

struct MeasurementMonitor {
    float values[BUFFER_SIZE];          // Rolling buffer for measurements
    unsigned long times[BUFFER_SIZE];   // Rolling buffer for timestamps
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
    float secondDerivatives[BUFFER_SIZE - 1];  // Stores second derivatives
};

// Initialize the monitor
MeasurementMonitor monitor;

// Simulate a measurement
float performMeasurement() {  
    float measurement = analogRead(A0) * (17.0 / 1023.0); // Convert ADC to voltage
    return measurement; 
}

// Initialize the monitor with default values
void initializeMonitor(MeasurementMonitor &mon) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mon.values[i] = mon.times[i] = 0;
        mon.timestamps[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        mon.errors[i] = mon.weights[i] = 0.001f;
        mon.confidence[i] = 1.0f;
    }
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        mon.derivatives[i] = 0.0f;
        mon.secondDerivatives[i] = 0.0f;
    }
    mon.count = 0;
    mon.threshold = 0.0001f;
    mon.shadowValue = 0.0f;
    mon.noiseModel = 0.0f;
    mon.driftModel = 0.0f;
}

// Add a new sample to the rolling buffer
void addSample(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    if (mon.count < BUFFER_SIZE) {
        mon.values[mon.count] = measurement;
        mon.times[mon.count] = currentTime;
        mon.timestamps[mon.count] = (float)currentTime / 1000.0f;  // Convert to seconds
        mon.count++;
    } else {
        // Shift values, times, and timestamps to make space for the new sample
        for (int i = 1; i < BUFFER_SIZE; i++) {
            mon.values[i - 1] = mon.values[i];
            mon.times[i - 1] = mon.times[i];
            mon.timestamps[i - 1] = mon.timestamps[i];
        }
        mon.values[BUFFER_SIZE - 1] = measurement;
        mon.times[BUFFER_SIZE - 1] = currentTime;
        mon.timestamps[BUFFER_SIZE - 1] = (float)currentTime / 1000.0f;
    }
}

// Calculate derivatives using nonlinear regression
void calculateDerivatives(MeasurementMonitor &mon) {
    if (mon.count < 3) return; // Need at least 3 points for second derivatives

    // First derivatives
    for (int i = 0; i < mon.count - 1; i++) {
        float dt = mon.timestamps[i + 1] - mon.timestamps[i];
        mon.derivatives[i] = (dt > 0) ? (mon.values[i + 1] - mon.values[i]) / dt : 0.0f;
    }

    // Second derivatives
    for (int i = 0; i < mon.count - 2; i++) {
        float dt = mon.timestamps[i + 2] - mon.timestamps[i];
        mon.secondDerivatives[i] = (dt > 0) ? 
            (mon.derivatives[i + 1] - mon.derivatives[i]) / dt : 0.0f;
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
    float x1 = mon.times[mon.count - 3], x2 = mon.times[mon.count - 2], x3 = mon.times[mon.count - 1];
    float y1 = mon.values[mon.count - 3], y2 = mon.values[mon.count - 2], y3 = mon.values[mon.count - 1];
    float a = (y3 - (x3 * (y2 - y1) + x2 * y1 - x1 * y2) / (x2 - x1)) / ((x3 - x2) * (x3 - x1));
    float b = (y2 - y1) / (x2 - x1) - a * (x2 + x1);
    float c = y1 - a * x1 * x1 - b * x1;
    unsigned long dt = mon.times[mon.count - 1] - x3;
    return a * dt * dt + b * dt + c;
}

float kalmanModelPrediction(float measurement, MeasurementMonitor &mon) {
    static float state = 0.0f;
    static float uncertainty = 1.0f;
    float processNoise = 1e-2f;
    float measurementNoise = mon.noiseModel > 0 ? mon.noiseModel : 1e-1f;

    float predictedState = state;
    float predictedUncertainty = uncertainty + processNoise;

    float gain = predictedUncertainty / (predictedUncertainty + measurementNoise);
    state = predictedState + gain * (measurement - predictedState);
    uncertainty = (1 - gain) * predictedUncertainty;

    return state;
}

// Refine measurement using local iterations
float refinedMeasurement(MeasurementMonitor &mon, int iterations) {
    float sum = 0.0f;
    for (int i = 0; i < iterations; i++) {
        sum += performMeasurement();
        delay(1);
    }
    return sum / iterations;
}

// Update noise model with average and drift
void updateNoiseModel(float measurement, float predictedValue, MeasurementMonitor &mon) {
    float instantaneousNoise = fabs(predictedValue - measurement);
    float noiseDrift = (instantaneousNoise - mon.noiseModel) / (mon.times[mon.count - 1] - mon.times[mon.count - 2]);

    mon.noiseModel = 0.9f * mon.noiseModel + 0.1f * instantaneousNoise;
    mon.driftModel = 0.9f * mon.driftModel + 0.1f * noiseDrift;
}

// Model-specific evolution

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

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Initialize the measurement monitor
    initializeMonitor(monitor);
}

void loop() {
    // Current time
    unsigned long currentTime = millis();
    
    // Perform measurement
    float measurement = performMeasurement();
    
    // Time within bounds placeholder
    float timeWithinBounds;
    
    // Monitor the measurement
    float deviationRate = monitorMeasurement(measurement, currentTime, timeWithinBounds);
    
    // Optional: Print results for debugging
    Serial.print("Measurement: ");
    Serial.print(measurement);
    Serial.print(" | Deviation Rate: ");
    Serial.print(deviationRate);
    Serial.print(" | Time Within Bounds: ");
    Serial.println(timeWithinBounds);
    
    // Optional: Add a delay to control sampling rate
    delay(100);
}
/*

Key improvements in this integrated version:
1. Enhanced `MeasurementMonitor` struct with new fields
2. Updated `initializeMonitor` to handle new fields
3. Modified `addSample` to capture precise timestamps
4. Enhanced `calculateDerivatives` to compute both first and second derivatives
5. Added `modelEvolution` and `weightedEvolution` functions for more sophisticated prediction
6. Improved `monitorMeasurement` with advanced prediction and bounds estimation
7. Added `setup()` and `loop()` functions for Arduino compatibility

The code provides a robust framework for monitoring measurements with:
- Multiple predictive models
- Adaptive noise and drift estimation
- Weighted model evolution
- Time-to-bounds estimation

Note: This code assumes an Arduino-like environment with `analogRead()`, `millis()`, and `delay()` functions. You may need to adjust it slightly depending on your specific microcontroller or embedded system.

Would you like me to explain any part of the code in more detail?
  */
