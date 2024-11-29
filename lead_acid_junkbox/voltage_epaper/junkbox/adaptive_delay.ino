#define MIN_DELAY 50    // Minimum delay in milliseconds
#define MAX_DELAY 5000  // Maximum delay in milliseconds

void loop() {
    // Current time
    unsigned long currentTime = millis();
    
    // Perform measurement
    float measurement = performMeasurement();
    
    // Time within bounds placeholder
    float timeWithinBounds;
    
    // Monitor the measurement
    float deviationRate = monitorMeasurement(measurement, currentTime, timeWithinBounds);
    
    // Compute adaptive delay based on time within bounds
    unsigned long adaptiveDelay = (unsigned long)timeWithinBounds;

    // Constrain the delay within defined limits
    if (adaptiveDelay < MIN_DELAY) {
        adaptiveDelay = MIN_DELAY;
    } else if (adaptiveDelay > MAX_DELAY) {
        adaptiveDelay = MAX_DELAY;
    }

    // Optional: Print results for debugging
    Serial.print("Measurement: ");
    Serial.print(measurement);
    Serial.print(" | Deviation Rate: ");
    Serial.print(deviationRate);
    Serial.print(" | Time Within Bounds: ");
    Serial.print(timeWithinBounds);
    Serial.print(" | Adaptive Delay: ");
    Serial.println(adaptiveDelay);
    
    // Delay for the computed time
    delay(adaptiveDelay);
}
