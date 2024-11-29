#include <math.h>
#include <Arduino.h>

#define BUFFER_SIZE 10
#define NUM_MODELS 3 // Polynomial, Kalman, Constant

struct MeasurementMonitor {
    float values[BUFFER_SIZE];          // Buffer for measurements
    float timestamps[BUFFER_SIZE];      // Corresponding timestamps
    float weights[NUM_MODELS];          // Weights for different models
    float errors[NUM_MODELS];           // Cumulative errors for model evaluation
    float predictions[NUM_MODELS];      // Store predictions from each model
    int count;                          // Number of samples in the buffer
    float threshold;                    // Allowed deviation threshold
    float noise;                        // Estimated noise level
    float minBound;                     // Minimum predicted value
    float maxBound;                     // Maximum predicted value
};

// Initialize the measurement monitor
MeasurementMonitor monitor;

// Function to simulate a measurement (replace with actual sensor reading)
float performMeasurement() {
    return analogRead(A0) * (17.0 / 1023.0); // Convert ADC to voltage
}

// Initialize the monitor with default values
void initializeMonitor(MeasurementMonitor &mon) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mon.values[i] = 0.0f;
        mon.timestamps[i] = 0.0f;
    }
    for (int i = 0; i < NUM_MODELS; i++) {
        mon.weights[i] = 1.0f / NUM_MODELS; // Equal initial weight
        mon.errors[i] = 0.0f;
        mon.predictions[i] = 0.0f;
    }
    mon.count = 0;
    mon.threshold = 0.01f; // Set a default threshold for anomaly detection
    mon.noise = 0.0f; // Initialize noise model
    mon.minBound = 0.0f; // Set minimum bound for predictions
    mon.maxBound = 5.0f; // Set maximum bound for predictions
}

// Add a new measurement sample to the buffer
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

// Calculate the derivative (rate of change) for the last two measurements
float calculateDerivative(MeasurementMonitor &mon) {
    if (mon.count < 2) return 0.0f; // Not enough data
    float deltaY = mon.values[mon.count - 1] - mon.values[mon.count - 2];
    float deltaT = mon.timestamps[mon.count - 1] - mon.timestamps[mon.count - 2];
    return (deltaT > 0) ? (deltaY / deltaT) : 0.0f; // Avoid division by zero
}

// Predict time to exit bounds based on the current value and derivative
float predictTimeToBounds(float currentValue, float derivative, float minBound, float maxBound) {
    if (derivative == 0) return INFINITY; // No change, will never exit bounds

    float timeToMin = (currentValue - minBound) / derivative;
    float timeToMax = (maxBound - currentValue) / derivative;

    // If derivative is negative, check time to min bound; if positive, check time to max bound
    return (derivative < 0) ? (timeToMin > 0 ? timeToMin : INFINITY) : (timeToMax > 0 ? timeToMax : INFINITY);
}

// Polynomial fitting for prediction (degree 2 for quadratic)
float polynomialFitPrediction(MeasurementMonitor &mon, float timeIncrement) {
    if (mon.count < 3) return mon.values[mon.count - 1]; // Not enough data

    // Calculate coefficients for a quadratic polynomial
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0, sumX2Y = 0;
    for (int i = 0; i < mon.count; i++) {
        float x = mon.timestamps[i];
        float y = mon.values[i];
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        sumX3 += x * x * x;
        sumX4 += x * x * x * x;
        sumX2Y += x * x * y;
    }

    // Calculate the coefficients using Cramer's Rule
    float A = (mon.count * sumX2Y - sumY * sumX2) / (mon.count * sumX4 - sumX2 * sumX2);
    float B = (sumY - A * sumX2) / sumX;
    float C = (sumXY - B * sumX - A * sumX2) / sumX2;

    // Predict the next value based on the polynomial
    float nextTime = mon.timestamps[mon.count - 1] + timeIncrement;
    return A * nextTime * nextTime + B * nextTime + C;
}

// Kalman filter implementation for noise adjustment
float kalmanFilter(float measurement, float &estimation, float &errorEstimation, float processNoise, float measurementNoise) {
    // Prediction update
    float prediction = estimation;
    errorEstimation += processNoise;

    // Measurement update
    float kalmanGain = errorEstimation / (errorEstimation + measurementNoise);
    estimation = prediction + kalmanGain * (measurement - prediction);
    errorEstimation *= (1 - kalmanGain); // Update error estimation

    return estimation;
}

// Constant model prediction
float constantModelPrediction(float currentValue) {
    return currentValue; // No change
}

// Update model weights based on cumulative error history
void updateWeights(MeasurementMonitor &mon) {
    float totalError = 0.0f;
    for (int i = 0; i < NUM_MODELS; i++) {
        totalError += mon.errors[i];
    }
    for (int i = 0; i < NUM_MODELS; i++) {
        if (totalError > 0) {
            mon.weights[i] = (1.0f - (mon.errors[i] / totalError)); // Inverse weighting
        } else {
            mon.weights[i] = 1.0f / NUM_MODELS; // Fallback to equal weight
        }
    }
}

// Combine predictions from all models based on their weights
float combinedPrediction(MeasurementMonitor &mon, float timeIncrement) {
    float predictions[NUM_MODELS];
    float timeToBounds[NUM_MODELS];

    // Get predictions from each model
    predictions[0] = polynomialFitPrediction(mon, timeIncrement);
    predictions[1] = kalmanFilter(mon.values[mon.count - 1], mon.predictions[1], mon.noise, 0.01f, 0.1f); // Example noise parameters
    predictions[2] = constantModelPrediction(mon.values[mon.count - 1]);
    Serial.print("model0: ");
    Serial.println(predictions[0]);
    Serial.print("model1: ");
    Serial.println(predictions[1]);
    Serial.print("model2: ");
    Serial.println(predictions[2]);
    
    // Calculate derivatives for time-to-bounds prediction
    float derivatives[NUM_MODELS];
    derivatives[0] = calculateDerivative(mon); // For polynomial
    derivatives[1] = (predictions[1] - mon.values[mon.count - 1]); // For Kalman (simple derivative)
    derivatives[2] = 0.0f; // Constant model has no derivative

    // Predict time to bounds for each model
    for (int i = 0; i < NUM_MODELS; i++) {
        timeToBounds[i] = predictTimeToBounds(predictions[i], derivatives[i], mon.minBound, mon.maxBound);
    }
    float totalWeight = 0.0f;

    // Combine predictions based on weights and time to bounds
    float combinedValue = 0.0f;
    for (int i = 0; i < NUM_MODELS; i++) {
        totalWeight += mon.weights[i];
        combinedValue += mon.weights[i] * predictions[i];
        mon.errors[i] += fabs(predictions[i] - mon.values[mon.count - 1]); // Update cumulative error
    }
      combinedValue = (totalWeight > 0) ? (combinedValue / totalWeight) : mon.values[mon.count - 1];
    
        // Optionally, you can apply a limit to the combined value to ensure it stays within bounds
    if (combinedValue < mon.minBound) {
        combinedValue = mon.minBound;
    } else if (combinedValue > mon.maxBound) {
        combinedValue = mon.maxBound;
    }

    // Update weights based on cumulative errors after making predictions
    updateWeights(mon);
    Serial.print("model0: ");
    Serial.print(predictions[0]);
    Serial.print(":derivative:");
    Serial.print(derivatives[0]);
    Serial.print(":timeToBounds:");
    Serial.print(timeToBounds[0]);
    Serial.print(":errors:");
    Serial.print(mon.errors[0]);
    Serial.print(":weights:");
    Serial.print(mon.weights[0]);
    Serial.println(".");

    Serial.print("model1: ");
    Serial.print(predictions[1]);
    Serial.print(":derivative:");
    Serial.print(derivatives[1]);
    Serial.print(":timeToBounds:");
    Serial.print(timeToBounds[1]);
    Serial.print(":errors:");
    Serial.print(mon.errors[1]);
    Serial.print(":weights:");
    Serial.print(mon.weights[1]);
    Serial.println(".");

    Serial.print("model2: ");
    Serial.print(predictions[2]);
    Serial.print(":derivative:");
    Serial.print(derivatives[2]);
    Serial.print(":timeToBounds:");
    Serial.print(timeToBounds[2]);
    Serial.print(":errors:");
    Serial.print(mon.errors[2]);
    Serial.print(":weights:");
    Serial.print(mon.weights[2]);
    Serial.println(".");


    return combinedValue;
}

// Main loop to gather measurements and perform predictions
void loop() {
    unsigned long currentTime = millis();
    
    // Simulate a new measurement
    float measurement = performMeasurement();
    addSample(measurement, currentTime, monitor);
    
    // Perform predictions
    float timeIncrement = 1.0f; // Time increment for prediction (e.g., 1 second)
    float predictedValue = combinedPrediction(monitor, timeIncrement);
    
    // Print results for debugging
    Serial.print("Current Measurement: ");
    Serial.println(measurement);
    Serial.print("Predicted Value: ");
    Serial.println(predictedValue);
    Serial.print("Time to Min Bound: ");
    Serial.println(predictTimeToBounds(predictedValue, calculateDerivative(monitor), monitor.minBound, monitor.maxBound));
    Serial.print("Time to Max Bound: ");
    Serial.println(predictTimeToBounds(predictedValue, calculateDerivative(monitor), monitor.maxBound, monitor.minBound));
    
    // Delay to simulate sampling period
    delay((timeIncrement/2.0)*1000); // Adjust as necessary for your application
}

// Setup function to initialize the monitor and serial communication
void setup() {
    Serial.begin(115200); // Initialize serial communication
    initializeMonitor(monitor); // Initialize the measurement monitor
}
