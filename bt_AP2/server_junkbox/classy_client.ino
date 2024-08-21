#include <WiFi.h>
#include <HTTPClient.h>
#include <StandardCplusplus.h> // Fake stdlib
#include <vector>


const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* serverIP = "ESP32_SERVER_IP";  // Replace with your ESP32 server's IP

// Base class for handling variable retrieval
class VariableClient {
protected:
    const char* path;
    const char* serverIP;
    HTTPClient http;
public:
    VariableClient(const char* p, const char* ip) : path(p), serverIP(ip) {}
    virtual void retrieveVariable() = 0;
    const char* getPath() const { return path; }
};

// Class to handle retrieval of a single float variable
class FloatVariableClient : public VariableClient {
private:
    float variable;
public:
    FloatVariableClient(const char* p, const char* ip) : VariableClient(p, ip), variable(0) {}
    void retrieveVariable() override {
        String url = String("http://") + serverIP + path;
        http.begin(url);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            http.getStream().readBytes((uint8_t*)&variable, sizeof(float));
            Serial.printf("Retrieved float from %s: %f\n", path, variable);
        } else {
            Serial.printf("Failed to retrieve float from %s\n", path);
        }
        http.end();
    }

    float getVariable() const { return variable; }
};

// Class to handle retrieval of a float array
class FloatArrayClient : public VariableClient {
private:
    float* variableArray;
    size_t arraySize;
public:
    FloatArrayClient(const char* p, const char* ip, float* array, size_t size) : VariableClient(p, ip), variableArray(array), arraySize(size) {}
    void retrieveVariable() override {
        String url = String("http://") + serverIP + path;
        http.begin(url);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            http.getStream().readBytes((uint8_t*)variableArray, sizeof(float) * arraySize);
            Serial.printf("Retrieved float array from %s:\n", path);
            for (size_t i = 0; i < arraySize; i++) {
                Serial.printf("  [%d] = %f\n", i, variableArray[i]);
            }
        } else {
            Serial.printf("Failed to retrieve float array from %s\n", path);
        }
        http.end();
    }

    float* getArray() const { return variableArray; }
    size_t getArraySize() const { return arraySize; }
};

// Wrapper class to manage all variable clients
class VariableClientManager {
private:
    std::vector<VariableClient*> clients;
public:
    void addClient(VariableClient* client) {
        clients.push_back(client);
    }

    void retrieveAll() {
        for (auto& vc : clients) {
            vc->retrieveVariable();
        }
    }

    // Method to retrieve a single variable by path
    void retrieveSingle(const char* path) {
        for (auto& vc : clients) {
            if (strcmp(vc->getPath(), path) == 0) {
                vc->retrieveVariable();
                return;
            }
        }
        Serial.printf("No client found for path: %s\n", path);
    }

    // Method to get a client by path (optional, to access retrieved data)
    VariableClient* getClient(const char* path) {
        for (auto& vc : clients) {
            if (strcmp(vc->getPath(), path) == 0) {
                return vc;
            }
        }
        Serial.printf("No client found for path: %s\n", path);
        return nullptr;
    }
};

float retrievedFloat;
float retrievedArray[5];

VariableClientManager clientManager;

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Setup clients
    clientManager.addClient(new FloatVariableClient("/float", serverIP));
    clientManager.addClient(new FloatArrayClient("/array", serverIP, retrievedArray, 5));
}

void loop() {
    // Retrieve all variables
    clientManager.retrieveAll();
    
    // Example usage: retrieve only the float variable
    clientManager.retrieveSingle("/float");

    // Access the retrieved float directly
    FloatVariableClient* floatClient = (FloatVariableClient*)clientManager.getClient("/float");
    if (floatClient) {
        float value = floatClient->getVariable();
        Serial.printf("Accessed float value after retrieval: %f\n", value);
    }

    // Example usage: retrieve only the float array
    clientManager.retrieveSingle("/array");

    // Access the retrieved array directly
    FloatArrayClient* arrayClient = (FloatArrayClient*)clientManager.getClient("/array");
    if (arrayClient) {
        float* array = arrayClient->getArray();
        size_t arraySize = arrayClient->getArraySize();
        Serial.println("Accessed float array values after retrieval:");
        for (size_t i = 0; i < arraySize; i++) {
            Serial.printf("  [%d] = %f\n", i, array[i]);
        }
    }

    delay(5000);  // Delay before retrieving data again
}
