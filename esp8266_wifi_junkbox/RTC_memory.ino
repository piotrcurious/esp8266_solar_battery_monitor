#include <ESP8266WiFi.h>

// Define a struct to store data
struct RTCData {
  uint32_t bootCount; // Example: Incremental boot counter
  float lastTemperature; // Example: Last recorded temperature
  char status[16]; // Example: A short status message
};

// Create an instance of the struct
RTCData rtcData;

// Define the RTC memory range
#define RTC_START_ADDRESS 64 // Start address (avoid first 64 bytes used by system)

// Function to write struct to RTC memory
bool saveRTCData() {
  ESP.rtcUserMemoryWrite(RTC_START_ADDRESS, (uint32_t*)&rtcData, sizeof(rtcData));
  return true; // You can add error checking if needed
}

// Function to read struct from RTC memory
bool loadRTCData() {
  return ESP.rtcUserMemoryRead(RTC_START_ADDRESS, (uint32_t*)&rtcData, sizeof(rtcData));
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize rtcData to default values
  rtcData.bootCount = 0;
  rtcData.lastTemperature = 0.0f;
  strncpy(rtcData.status, "OK", sizeof(rtcData.status));

  // Read from RTC memory
  if (loadRTCData()) {
    Serial.println("RTC data loaded successfully!");
    Serial.printf("Boot count: %d\n", rtcData.bootCount);
    Serial.printf("Last temperature: %.2f°C\n", rtcData.lastTemperature);
    Serial.printf("Status: %s\n", rtcData.status);

    // Increment boot count
    rtcData.bootCount++;

    // Update other data (example)
    rtcData.lastTemperature = 25.5; // Mock temperature value
    strncpy(rtcData.status, "Running", sizeof(rtcData.status));
  } else {
    Serial.println("Failed to load RTC data. Initializing defaults.");
  }

  // Save updated data back to RTC memory
  if (saveRTCData()) {
    Serial.println("RTC data saved successfully!");
  } else {
    Serial.println("Failed to save RTC data.");
  }

  // Simulate a deep sleep cycle (uncomment to test)
  // ESP.deepSleep(10e6); // Sleep for 10 seconds
}

void loop() {
  // Main loop - keep it empty for this example
}
