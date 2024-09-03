// Main Arduino Sketch
#include <Arduino.h>
#include "SensorFilter.h"

// Create a SensorFilter object with 16 entries, outlier threshold of 2.0, 
// base Kalman gain of 0.1, process noise of 0.1, and measurement noise of 1.0
SensorFilter sensorFilter(16, 2.0, 0.1, 0.1, 1.0);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Simulate sensor reading
    float sensorValue = analogRead(A0); // Replace with your sensor reading method

    // Scale sensor reading for demonstration (e.g., 0 to 5V)
    sensorValue = (sensorValue / 1023.0) * 5.0;

    // Update sensor reading with filtering
    float filteredValue = sensorFilter.updateSensorReading(sensorValue);

    // Print the filtered sensor value
    Serial.println(filteredValue);

    delay(100); // Delay to simulate sensor update rate
}
