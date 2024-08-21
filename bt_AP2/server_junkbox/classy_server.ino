#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Define a base class for serving variables
class VariableServer {
protected:
    const char* path;
public:
    VariableServer(const char* p) : path(p) {}
    virtual void handleRequest(AsyncWebServerRequest *request) = 0;
    const char* getPath() const { return path; }
};

// Template class to handle a single float variable
class FloatVariableServer : public VariableServer {
private:
    float* variable;
public:
    FloatVariableServer(const char* p, float* var) : VariableServer(p), variable(var) {}
    void handleRequest(AsyncWebServerRequest *request) override {
        request->send_P(200, "application/octet-stream", (uint8_t*)variable, sizeof(float));
    }
};

// Template class to handle a float array
class FloatArrayServer : public VariableServer {
private:
    float* variableArray;
    size_t arraySize;
public:
    FloatArrayServer(const char* p, float* array, size_t size) : VariableServer(p), variableArray(array), arraySize(size) {}
    void handleRequest(AsyncWebServerRequest *request) override {
        request->send_P(200, "application/octet-stream", (uint8_t*)variableArray, sizeof(float) * arraySize);
    }
};

// Wrapper class to manage all servers
class VariableManager {
private:
    std::vector<VariableServer*> servers;
public:
    void addServer(VariableServer* server) {
        servers.push_back(server);
    }

    void attachHandlers(AsyncWebServer& server) {
        for (auto& vs : servers) {
            server.on(vs->getPath(), HTTP_GET, [vs](AsyncWebServerRequest *request) {
                vs->handleRequest(request);
            });
        }
    }
};

// Declare global variables
float myFloat = 123.456f;
float myArray[5] = {1.1, 2.2, 3.3, 4.4, 5.5};

AsyncWebServer server(80);
VariableManager varManager;

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Setup servers
    varManager.addServer(new FloatVariableServer("/float", &myFloat));
    varManager.addServer(new FloatArrayServer("/array", myArray, 5));

    // Attach handlers to the web server
    varManager.attachHandlers(server);

    // Start server
    server.begin();
}

void loop() {
    // Nothing needed here
}
