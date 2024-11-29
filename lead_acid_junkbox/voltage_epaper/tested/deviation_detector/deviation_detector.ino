struct MeasurementMonitor {
    float lastAverage;     // Stores the last calculated average
    float sum;             // Running sum of measurements
    unsigned long count;   // Number of measurements
    unsigned long lastTime; // Last time the average was checked
    float threshold;       // Threshold for average change
};

// Initialize a MeasurementMonitor structure
MeasurementMonitor monitor = {0, 0, 0, 0, 0.01}; // Example threshold set to 5.0

// Function to check if average change exceeds the threshold
bool checkAverageChange(float measurement, unsigned long currentTime) {
    // Update the sum and count
    monitor.sum += measurement;
    monitor.count++;

    // Only check average change at 1-second intervals
    if (currentTime - monitor.lastTime >= 1000) {
        // Calculate the new average
        float newAverage = monitor.sum / monitor.count;

        // Calculate the absolute change in average
        float averageChange = abs(newAverage - monitor.lastAverage);

        // Update lastAverage and lastTime
        monitor.lastAverage = newAverage;
        monitor.lastTime = currentTime;

        // Reset sum and count for the next interval
        monitor.sum = 0;
        monitor.count = 0;

        // Check if the average change exceeds the threshold
        return averageChange > monitor.threshold;
    }

    // Return false if not enough time has passed to check
    return false;
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Example: Simulated measurement input
    float measurement = analogRead(A0) * (17.0 / 1023.0); // Convert ADC to voltage
    unsigned long currentTime = millis();

    // Check if the average change exceeds the threshold
    if (checkAverageChange(measurement, currentTime)) {
        Serial.println("Average change exceeded the threshold!");
        Serial.print("Voltage:");
        Serial.print(measurement);
        Serial.println("V");                
    }
    delay(1000);
}
