// Include the libraries for BLE and WiFi
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>

// Define the service and characteristic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Define the WiFi credentials
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Define the variables to share
int var1 = 0; // A counter variable
float var2 = 0.0; // A sensor variable

// Create a BLE server and a characteristic
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

// A flag to indicate if the device is connected to another device
bool deviceConnected = false;

// A callback class to handle the BLE events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// A function to initialize the BLE server and the characteristic
void setupBLE() {
  // Create the BLE device and set its name
  BLEDevice::init("ESP32-BLE");
  // Create the BLE server
  pServer = BLEDevice::createServer();
  // Set the server callback
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create the BLE characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
}

// A function to initialize the WiFi connection
void setupWiFi() {
  // Connect to the WiFi network
  WiFi.begin(ssid, password);
  // Wait until the connection is established
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// A function to update the variables and send them to the connected device
void updateVariables() {
  // Increment the counter variable
  var1++;
  // Read the sensor variable from a hypothetical analog pin
  var2 = analogRead(34) * 0.1;
  // Create a buffer to store the variables
  uint8_t buffer[8];
  // Copy the variables to the buffer
  memcpy(buffer, &var1, 4);
  memcpy(buffer + 4, &var2, 4);
  // Send the buffer to the connected device
  pCharacteristic->setValue(buffer, 8);
  pCharacteristic->notify();
}

// The setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  // Initialize the BLE server and the characteristic
  setupBLE();
  // Initialize the WiFi connection
  setupWiFi();
}

// The loop function
void loop() {
  // If the device is connected to another device
  if (deviceConnected) {
    // Update the variables and send them to the connected device
    updateVariables();
    // Print the variables to the serial monitor
    Serial.print("var1: ");
    Serial.print(var1);
    Serial.print(", var2: ");
    Serial.println(var2);
  }
  // Wait for one second
  delay(1000);
}
