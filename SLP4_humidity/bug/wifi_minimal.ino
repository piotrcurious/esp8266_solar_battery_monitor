#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>
#include <AsyncUDP.h>

const char* ssid = "YourSSID";
const char* password = "YourPassword";
const int channel = 1;  // Fixed channel

AsyncUDP udp;

void setup() {
  Serial.begin(115200);

  // Initialize WiFi with ultra-minimal configuration
  if (!wifiStartUltraMinimal()) {
    Serial.println("Failed to initialize WiFi");
    return;
  }

  // Set up AsyncUDP for multicast
  setupMinimalMulticastUDP();

  Serial.println("Ultra-Minimal WiFi SoftAP with AsyncUDP is ready");
}

void loop() {
  // Your main code here
}

bool wifiStartUltraMinimal() {
  // Initialize TCP/IP adapter
  tcpip_adapter_init();

  // Initialize WiFi with default config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&cfg) != ESP_OK) {
    return false;
  }

  // Set WiFi storage to RAM to prevent unnecessary flash writes
  if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) {
    return false;
  }

  // Set mode to AP only
  if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) {
    return false;
  }

  // Configure AP
  wifi_config_t ap_config = {};
  strcpy((char*)ap_config.ap.ssid, ssid);
  ap_config.ap.ssid_len = strlen(ssid);
  strcpy((char*)ap_config.ap.password, password);
  ap_config.ap.channel = channel;
  ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
  ap_config.ap.max_connection = 4;  // Adjust as needed
  ap_config.ap.beacon_interval = 100;  // Default beacon interval

  // Disable hidden SSID to reduce beacons
  ap_config.ap.ssid_hidden = 0;

  // Apply the configuration
  if (esp_wifi_set_config(WIFI_IF_AP, &ap_config) != ESP_OK) {
    return false;
  }

  // Start WiFi
  if (esp_wifi_start() != ESP_OK) {
    return false;
  }

  // Disable scanning
  if (esp_wifi_scan_stop() != ESP_OK) {
    return false;
  }

  // Disable power save mode
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    return false;
  }

  // Disable any automatic channel selection
  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 1,  // Only one channel
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  if (esp_wifi_set_country(&country) != ESP_OK) {
    return false;
  }

  // Disable AMPDU for TX and RX
  if (esp_wifi_set_ampdu_rx_disable(true) != ESP_OK ||
      esp_wifi_set_ampdu_tx_disable(true) != ESP_OK) {
    return false;
  }

  // Set WiFi protocol to 11b only for simplicity
  if (esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B) != ESP_OK) {
    return false;
  }

  // Disable WiFi NVS storage to prevent any background writing
  if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) {
    return false;
  }

  // Optionally, reduce TX power to minimize interference
  // if (esp_wifi_set_max_tx_power(8) != ESP_OK) {  // Range: 2-20 dBm
  //   return false;
  // }

  return true;
}

void setupMinimalMulticastUDP() {
  IPAddress multicastIP(239, 1, 2, 3);
  if (udp.listenMulticast(multicastIP, 1234)) {
    Serial.println("UDP Listening on IP: 239.1.2.3");
    udp.onPacket([](AsyncUDPPacket packet) {
      // Minimal packet handler - just print length
      Serial.printf("Received UDP packet, length: %d\n", packet.length());
    });
  }
}
