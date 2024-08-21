#include <WiFi.h>
#include <HTTPClient.h>
#include "VariableDefinitions.h"

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* serverIP = "ESP32_SERVER_IP";  // Replace with your ESP32 server's IP

// Function to retrieve a single float variable
float retrieveFloat(const char* path) {
    HTTPClient http;
    String url = String("http://") + serverIP + path;
    http.begin(url);
    int httpCode = http.GET();
    float value = 0.0;

    if (httpCode == HTTP_CODE_OK) {
        http.getStream().readBytes((uint8_t*)&value, sizeof(float));
        Serial.printf("Retrieved float from %s: %f\n", path, value);
    } else {
        Serial.printf("Failed to retrieve float from %s\n", path);
    }
    http.end();
    return value;
}

// Function to retrieve a float array
void retrieveFloatArray(const char* path, float* array, size_t size) {
    HTTPClient http;
    String url = String("http://") + serverIP + path;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        http.getStream().readBytes((uint8_t*)array, sizeof(float) * size);
        Serial.printf("Retrieved float array from %s:\n", path);
        for (size_t i = 0; i < size; i++) {
            Serial.printf("  [%d] = %f\n", i, array[i]);
        }
    } else {
        Serial.printf("Failed to retrieve float array from %s\n", path);
    }
    http.end();
}

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Retrieve the float variable
    float retrievedFloat = retrieveFloat(floatVarPath);
    Serial.printf("Retrieved float value: %f\n", retrievedFloat);

    // Retrieve the float array
    float retrievedArray[floatArraySize];
    retrieveFloatArray(floatArrayPath, retrievedArray, floatArraySize);
}

void loop() {
    // Optionally, retrieve data periodically
    delay(5000);  // Delay before retrieving data again

    // Reuse the same retrieval logic to fetch data
    float retrievedFloat = retrieveFloat(floatVarPath);
    Serial.printf("Retrieved float value: %f\n", retrievedFloat);

    float retrievedArray[floatArraySize];
    retrieveFloatArray(floatArrayPath, retrievedArray, floatArraySize);
}
