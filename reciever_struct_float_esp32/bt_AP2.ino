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
    updateDisplay("Starting...");
}

void loop() {
    // Update display based on AP status
    if (apStarted) {
        updateDisplay("Active", WiFi.softAPIP().toString());
    } else {
        updateDisplay("Inactive");
    }

    delay(5000); // Update every 5 seconds
}

// Function to update the display with dynamic positioning
void updateDisplay(String status, String ip = "") {
    int displayWidth = bluedisplay.getDisplayWidth();
    int displayHeight = bluedisplay.getDisplayHeight();

    int x0 = 0;
    int y0 = displayHeight * 0.05; // 5% down from the top

    int y1 = displayHeight * 0.25; // 25% down from the top
    int y2 = displayHeight * 0.45; // 45% down from the top
    int y3 = displayHeight * 0.65; // 65% down from the top

    bluedisplay.clearDisplay();
    bluedisplay.drawText(x0, y0, "ESP32 SoftAP");
    bluedisplay.drawText(x0, y1, "SSID: " + String(ssid));
    bluedisplay.drawText(x0, y2, "Status: " + status);
    if (status == "Active" && ip != "") {
        bluedisplay.drawText(x0, y3, "IP: " + ip);
    }
    bluedisplay.flushDisplay();
}
