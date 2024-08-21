#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "VariableDefinitions.h"

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

AsyncWebServer server(80);

// Function to serve a float variable
void handleFloatVarRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "application/octet-stream", (const uint8_t*)&floatVar, sizeof(floatVar));
}

// Function to serve a float array
void handleFloatArrayRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "application/octet-stream", (const uint8_t*)floatArray, sizeof(floatArray));
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Serve the float variable
    server.on(floatVarPath, HTTP_GET, handleFloatVarRequest);

    // Serve the float array
    server.on(floatArrayPath, HTTP_GET, handleFloatArrayRequest);

    server.begin();
    Serial.println("Server started");
}

void loop() {
    // Your loop code here
}
