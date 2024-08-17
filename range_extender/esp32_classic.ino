#include <WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi credentials for STA mode
const char* sta_ssid = "Your_STA_SSID";
const char* sta_password = "Your_STA_Password";

// AP mode credentials
const char* ap_ssid = "ESP32_AP";
const char* ap_password = "12345678";

// UDP settings
const uint16_t udpPort = 5683;
WiFiUDP udp;

IPAddress multicastAddress(224, 0, 0, 1); // Multicast address
const int multicastTTL = 1;               // Time-To-Live for multicast packets

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  delay(1000);

  // Start Wi-Fi in AP mode
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("AP mode started");

  // Connect to STA Wi-Fi network
  WiFi.begin(sta_ssid, sta_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to STA network...");
  }
  Serial.println("Connected to STA network");

  // Setup UDP
  udp.beginMulticast(WiFi.softAPIP(), multicastAddress, udpPort);
  Serial.println("Listening for multicast packets...");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    
    // Print received packet info
    Serial.printf("Received UDP packet: %s\n", packetBuffer);

    // Send the received packet to the STA network
    udp.beginPacketMulticast(multicastAddress, udpPort, WiFi.localIP());
    udp.write(packetBuffer, len);
    udp.endPacket();

    Serial.println("Packet forwarded to STA network");
  }
}
