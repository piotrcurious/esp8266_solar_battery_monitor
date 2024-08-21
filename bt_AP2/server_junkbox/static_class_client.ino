#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* serverIP = "ESP32_SERVER_IP";  // Replace with your ESP32 server's IP

// Base class for managing HTTP clients
class HttpClient {
protected:
    const char* serverIP;
    const char* path;
    HTTPClient http;

    HttpClient(const char* ip, const char* p) : serverIP(ip), path(p) {}

public:
    virtual void retrieve() = 0;
};

// Class for handling float variable retrieval
class FloatClient : public HttpClient {
private:
    float value;

    // Private constructor to be used by static method
    FloatClient(const char* ip, const char* p) : HttpClient(ip, p), value(0.0) {}

public:
    static FloatClient create(const char* ip, const char* p) {
        return FloatClient(ip, p);
    }

    void retrieve() override {
        String url = String("http://") + serverIP + path;
        http.begin(url);
        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            http.getStream().readBytes((uint8_t*)&value, sizeof(float));
            Serial.printf("Retrieved float from %s: %f\n", path, value);
        } else {
            Serial.printf("Failed to retrieve float from %s\n", path);
        }
        http.end();
    }

    float getValue() const { return value; }
};

// Class for handling float array retrieval
class FloatArrayClient : public HttpClient {
private:
    float* array;
    size_t size;

    // Private constructor to be used by static method
    FloatArrayClient(const char* ip, const char* p, float* arr, size_t s)
        : HttpClient(ip, p), array(arr), size(s) {}

public:
    static FloatArrayClient create(const char* ip, const char* p, float* arr, size_t s) {
        return FloatArrayClient(ip, p, arr, s);
    }

    void retrieve() override {
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

    const float* getArray() const { return array; }
    size_t getSize() const { return size; }
};

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Create clients using static constructors
    FloatClient floatClient = FloatClient::create(serverIP, "/float");
    floatClient.retrieve();
    float retrievedFloat = floatClient.getValue();
    Serial.printf("Retrieved float value: %f\n", retrievedFloat);

    float retrievedArray[5];
    FloatArrayClient arrayClient = FloatArrayClient::create(serverIP, "/array", retrievedArray, 5);
    arrayClient.retrieve();
    const float* array = arrayClient.getArray();
    Serial.println("Retrieved array values:");
    for (size_t i = 0; i < arrayClient.getSize(); i++) {
        Serial.printf("  [%d] = %f\n", i, array[i]);
    }
}

void loop() {
    // Example usage: periodic retrieval
    delay(5000);

    // Reuse the same client instances to retrieve data again
    // FloatClient and FloatArrayClient could be used as in setup()
}
