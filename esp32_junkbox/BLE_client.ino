// Include the libraries for BLE and WiFi
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <WiFi.h>

// Define the service and characteristic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Define the WiFi credentials
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Define the variables to receive
int var1 = 0; // A counter variable
float var2 = 0.0; // A sensor variable

// Create a BLE client and a characteristic
BLEClient *pClient  = NULL;
BLERemoteCharacteristic* pRemoteCharacteristic = NULL;

// A flag to indicate if the device is connected to another device
bool deviceConnected = false;

// A callback class to handle the BLE events
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    deviceConnected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    deviceConnected = false;
  }
};

// A callback class to handle the characteristic events
class MyCharacteristicCallback : public BLERemoteCharacteristicCallbacks {
  void onRead(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Copy the data from the buffer to the variables
    memcpy(&var1, pData, 4);
    memcpy(&var2, pData + 4, 4);
    // Print the variables to the serial monitor
    Serial.print("var1: ");
    Serial.print(var1);
    Serial.print(", var2: ");
    Serial.println(var2);
  }
};

// A function to initialize the BLE client and the characteristic
void setupBLE() {
  // Create the BLE device and set its name
  BLEDevice::init("ESP32-BLE");
  // Create the BLE client
  pClient  = BLEDevice::createClient();
  // Set the client callback
  pClient->setClientCallbacks(new MyClientCallback());
  // Connect to the BLE server
  pClient->connect(BLEAddress("24:0A:C4:00:00:01")); // Put here the address of the other ESP32
  // Get the service
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  // Get the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  // Set the characteristic callback
  pRemoteCharacteristic->registerForNotify(new MyCharacteristicCallback());
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

// The setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  // Initialize the BLE client and the characteristic
  setupBLE();
  // Initialize the WiFi connection
  setupWiFi();
}

// The loop function
void loop() {
  // If the device is connected to another device
  if (deviceConnected) {
    // Read the characteristic value
    pRemoteCharacteristic->readValue();
  }
  // Wait for one second
  delay(1000);
}
