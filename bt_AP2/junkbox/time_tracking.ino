#include <TimeLib.h> // Library to handle time operations

unsigned long lastUpdateTime = 0; // Store the Unix time of the last update
unsigned long updateInterval = 60; // The interval between updates, in seconds (adjust to your needs)

void setup() {
    // Setup your serial, WiFi, etc.
    Serial.begin(115200);
    // Initialize the time library
    setTime(0); // This initializes time to zero. You will need to sync time from an NTP server or RTC module.

    // Retrieve stored last update time if available, otherwise set it to current time
    lastUpdateTime = now();  // Assuming now() returns the current Unix timestamp
}

void loop() {
    unsigned long currentTime = now(); // Get the current Unix time
    unsigned long timeElapsed = currentTime - lastUpdateTime;

    if (timeElapsed >= updateInterval) {
        // Calculate how many intervals have passed
        int missedIntervals = timeElapsed / updateInterval;

        // Update the graph for each missed interval
        for (int i = 0; i < missedIntervals; i++) {
            updateGraph();
            lastUpdateTime += updateInterval; // Increment last update time by one interval
        }
    }

    // Other operations can go here...

    // Optional: Put the ESP32 into deep sleep if no further updates or operations are needed
}

void updateGraph() {
    // Your existing code to update the graph
    Serial.println("Graph updated");
}
