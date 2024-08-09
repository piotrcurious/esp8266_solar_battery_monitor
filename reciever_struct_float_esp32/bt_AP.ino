#include <WiFi.h>
#include <BlueDisplay.h>

// Define the SSID and Password for the AP
const char *ssid = "ESP32_SoftAP";
const char *password = "12345678";

// BlueDisplay object
BlueDisplay bluedisplay;

// This is a flag to keep track of the AP state
bool apStarted = false;

// Function to handle events related to the SoftAP
void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_AP_START:
            Serial.println("SoftAP started");
            apStarted = true;
            break;
        case SYSTEM_EVENT_AP_STOP:
            Serial.println("SoftAP stopped");
            apStarted = false;
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Serial.println("Device connected to SoftAP");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Serial.println("Device disconnected from SoftAP");
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize BlueDisplay
    bluedisplay.begin("ESP32 BlueDisplay");
    
    // Register the WiFi event handler
    WiFi.onEvent(onWiFiEvent);

    // Start the SoftAP
    Serial.println("Starting SoftAP...");
    WiFi.softAP(ssid, password);

    // Display initial message
    bluedisplay.clearDisplay();
    bluedisplay.println("ESP32 SoftAP");
    bluedisplay.println("SSID: " + String(ssid));
    bluedisplay.println("Status: Starting...");
    bluedisplay.flushDisplay();
}

void loop() {
    // Update display based on AP status
    if (apStarted) {
        bluedisplay.clearDisplay();
        bluedisplay.println("ESP32 SoftAP");
        bluedisplay.println("SSID: " + String(ssid));
        bluedisplay.println("Status: Active");
        bluedisplay.println("IP: " + WiFi.softAPIP().toString());
        bluedisplay.flushDisplay();
    } else {
        bluedisplay.clearDisplay();
        bluedisplay.println("ESP32 SoftAP");
        bluedisplay.println("SSID: " + String(ssid));
        bluedisplay.println("Status: Inactive");
        bluedisplay.flushDisplay();
    }

    delay(5000); // Update every 5 seconds
}
