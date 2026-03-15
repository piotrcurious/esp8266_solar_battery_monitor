// Main Arduino Sketch
#include <Arduino.h>
#include "SensorFilter.h"

// Create a SensorFilter object (v3.0.0 2D Kinematic Filter)
// Params: window size (0 to disable), outlier threshold (std dev),
//         q_pos, q_vel, r_noise
SensorFilter sensorFilter(16, 2.5f, 0.01f, 0.001f, 0.1f);

void setup() {
    Serial.begin(115200);
}

void loop() {
    // 1. Get raw sensor reading
    float rawValue = (float)analogRead(A0) * (5.0f / 1023.0f);

    // 2. Update filter (automatically adapts to noise and detects steps)
    float filteredValue = sensorFilter.updateSensorReading(rawValue);

    // 3. Output results
    // You can also get the estimated velocity (rate of change)
    float velocity = sensorFilter.getVelocity();

    Serial.print("Raw:"); Serial.print(rawValue);
    Serial.print(" Filtered:"); Serial.print(filteredValue);
    Serial.print(" Velocity:"); Serial.println(velocity);

    delay(20);
}

// Example of persisting state for Deep Sleep (ESP32)
/*
RTC_DATA_ATTR SensorFilter::State savedState;

void goToSleep() {
    savedState = sensorFilter.getState();
    esp_deep_sleep_start();
}

void wakeUp() {
    sensorFilter.setState(savedState);
}
*/
