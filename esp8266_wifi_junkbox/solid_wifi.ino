//Dreamed by BingAI
//creates wifi connection and keeps it alive
//reconnecting if it dies (no ping)
//part of junkbox to improve esp8266 sender code
#include <ESP8266WiFi.h>
#include <Ticker.h>

// WiFi credentials
const char* ssid = "your-ssid";
const char* password = "your-password";

// WiFi event handler
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

// Ticker object to test connection
Ticker wifiTest;

// WiFi connection status flag
bool wifiConnected = false;

// WiFi connection test function
void testWiFi() {
  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    // Ping Google DNS server
    if (WiFi.ping("8.8.8.8") > 0) {
      // Connection is functional
      Serial.println("WiFi is OK");
    } else {
      // Connection is not functional
      Serial.println("WiFi is not OK");
      // Disconnect and reconnect
      WiFi.disconnect();
    }
  }
}

// WiFi connected callback
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  // Set connection flag
  wifiConnected = true;
  // Print IP address
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Start connection test every 10 seconds
  wifiTest.attach(10, testWiFi);
}

// WiFi disconnected callback
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  // Clear connection flag
  wifiConnected = false;
  // Stop connection test
  wifiTest.detach();
  // Print reason
  Serial.print("Disconnected from ");
  Serial.print(event.ssid);
  Serial.print(" because ");
  Serial.println(event.reason);
  // Reconnect
  WiFi.reconnect();
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);
  // Register WiFi event handlers
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  // Connect to WiFi
  WiFi.begin(ssid, password);
}

void loop() {
  // User code goes here
}
