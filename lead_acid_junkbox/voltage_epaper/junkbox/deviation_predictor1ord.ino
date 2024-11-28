struct MeasurementMonitor {
    float values[4];           // Rolling buffer for the last four measurements
    unsigned long times[4];    // Corresponding timestamps
    int count;                 // Current number of stored samples (max 4)
    float deviationRate;       // Deviation rate between predicted and measured values
    float threshold;           // Threshold for deviation rate
};

// Initialize the monitor
MeasurementMonitor monitor = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0, 0.0, 0.5};

// Function to add a new measurement to the rolling buffer
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

// Function to calculate the derivative using dynamic time intervals
float calculateDynamicDerivative(MeasurementMonitor &mon) {
    int count = mon.count;
    if (count < 2) {
        return 0.0;
    }

    float numerator = 0, denominator = 0;
    for (int i = 1; i < count; i++) {
        float deltaTime = (mon.times[i] - mon.times[i - 1]) / 1000.0;
        if (deltaTime > 0) {
            float deltaValue = mon.values[i] - mon.values[i - 1];
            numerator += deltaValue / deltaTime;
            denominator += 1.0;
        }
    }

    return (denominator > 0) ? numerator / denominator : 0.0;
}

// Function to predict the next value based on the derivative
float predictNextValue(MeasurementMonitor &mon, float measuredDerivative, unsigned long currentTime) {
    float deltaTime = (currentTime - mon.times[mon.count - 1]) / 1000.0;
    return mon.values[mon.count - 1] + measuredDerivative * deltaTime;
}

// Function to predict the time remaining before the signal exits the threshold bounds
float predictTimeWithinBounds(MeasurementMonitor &mon, float measuredDerivative) {
    if (abs(measuredDerivative) < 1e-6) {
        return INFINITY; // Signal is stable and won't leave bounds
    }

    float currentValue = mon.values[mon.count - 1];
    float upperBound = currentValue * (1.0 + mon.threshold);
    float lowerBound = currentValue * (1.0 - mon.threshold);

    float timeToUpper = (upperBound - currentValue) / measuredDerivative;
    float timeToLower = (lowerBound - currentValue) / measuredDerivative;

    // Take the minimum positive time (if both are positive)
    float predictedTime = INFINITY;
    if (timeToUpper > 0) predictedTime = timeToUpper;
    if (timeToLower > 0 && timeToLower < predictedTime) predictedTime = timeToLower;

    return predictedTime;
}

// Function to check average change and calculate deviation rate
float checkAverageChange(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addToRollingBuffer(measurement, currentTime, monitor);

    if (monitor.count < 4) {
        timeWithinBounds = INFINITY; // Insufficient data
        return 0.0;
    }

    float measuredDerivative = calculateDynamicDerivative(monitor);
    float predictedValue = predictNextValue(monitor, measuredDerivative, currentTime);
    monitor.deviationRate = abs(predictedValue - measurement) / abs(measurement);

    // Predict how long the signal will stay within the threshold bounds
    timeWithinBounds = predictTimeWithinBounds(monitor, measuredDerivative);

    return monitor.deviationRate;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Simulated measurement input
    float measurement = analogRead(A0) * (5.0 / 1023.0); // Convert ADC to voltage
    unsigned long currentTime = millis();

    // Check average change and get the deviation rate and time within bounds
    float timeWithinBounds;
    float deviationRate = checkAverageChange(measurement, currentTime, timeWithinBounds);

    // Print results
    Serial.print("Deviation Rate: ");
    Serial.println(deviationRate);
    Serial.print("Time Within Bounds: ");
    if (isinf(timeWithinBounds)) {
        Serial.println("Infinite (Stable)");
    } else {
        Serial.print(timeWithinBounds);
        Serial.println(" seconds");
    }

    if (deviationRate > monitor.threshold) {
        Serial.println("Deviation rate exceeded the threshold!");
    }

    delay(random(50, 150)); // Simulate variable sampling rate
}
