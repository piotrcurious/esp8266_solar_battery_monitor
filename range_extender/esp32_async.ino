#include <WiFi.h>
#include <AsyncUDP.h>

// Wi-Fi credentials for STA mode
const char* sta_ssid = "Your_STA_SSID";
const char* sta_password = "Your_STA_Password";

// AP mode credentials
const char* ap_ssid = "ESP32_AP";
const char* ap_password = "12345678";

// UDP settings
const uint16_t udpPort = 5683;
IPAddress multicastAddress(224, 0, 0, 1); // Multicast address

AsyncUDP udp;

// Function to handle received packets and forward them
void onPacketReceived(AsyncUDPPacket packet) {
  Serial.printf("Received UDP packet: %s\n", (char*)packet.data());

  // Forward the received packet to the STA network
  udp.writeTo(packet.data(), packet.length(), multicastAddress, udpPort, WiFi.localIP());
  Serial.println("Packet forwarded to STA network");
}

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

  // Setup AsyncUDP
  if (udp.listenMulticast(multicastAddress, udpPort)) {
    Serial.println("Listening for multicast packets...");
    udp.onPacket(onPacketReceived);
  }
}

void loop() {
  // Nothing to do here since packet handling is done asynchronously
}
