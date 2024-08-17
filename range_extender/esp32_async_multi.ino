#include <WiFi.h>
#include <AsyncUDP.h>

// Wi-Fi credentials for STA mode
const char* sta_ssid = "Your_STA_SSID";
const char* sta_password = "Your_STA_Password";

// AP mode credentials
const char* ap_ssid = "ESP32_AP";
const char* ap_password = "12345678";

// Multicast address
IPAddress multicastAddress(224, 0, 0, 1);

// UDP Relay class
class UDPRelay {
  public:
    UDPRelay(uint16_t port) : udpPort(port) {}
    
    void begin() {
      if (udp.listenMulticast(multicastAddress, udpPort)) {
        Serial.printf("Listening for multicast packets on port %u...\n", udpPort);
        udp.onPacket([this](AsyncUDPPacket packet) {
          this->onPacketReceived(packet);
        });
      }
    }
    
  private:
    uint16_t udpPort;
    AsyncUDP udp;

    void onPacketReceived(AsyncUDPPacket packet) {
      Serial.printf("Received UDP packet on port %u: %s\n", udpPort, (char*)packet.data());
      udp.writeTo(packet.data(), packet.length(), multicastAddress, udpPort, WiFi.localIP());
      Serial.printf("Packet forwarded to STA network on port %u\n", udpPort);
    }
};

UDPRelay relay1(5683);
UDPRelay relay2(5684); // Another instance for a different port

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

  // Start UDP relays
  relay1.begin();
  relay2.begin();
}

void loop() {
  // Nothing to do here since packet handling is done asynchronously
}
