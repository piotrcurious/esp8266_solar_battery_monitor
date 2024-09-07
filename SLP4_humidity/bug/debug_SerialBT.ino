#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_main.h"
#include "esp_coexist.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth with minimal configuration
  if (!btStartMinimal()) {
    Serial.println("Failed to initialize Bluetooth");
    return;
  }

  // Start Bluetooth Serial
  SerialBT.begin("ESP32_BT_Serial");

  // Disable unnecessary Bluetooth features
  disableUnnecessaryBtFeatures();

  // Configure coexistence
  configureCoexistence();

  Serial.println("Minimal Bluetooth Serial is ready");
}

void loop() {
  // Your main code here
}

bool btStartMinimal() {
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  
  // Minimize memory usage
  bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
  bt_cfg.bt_max_acl_conn = 1;
  bt_cfg.bt_max_sync_conn = 0;
  
  if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
    return false;
  }
  
  if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
    return false;
  }
  
  if (esp_bluedroid_init() != ESP_OK) {
    return false;
  }
  
  if (esp_bluedroid_enable() != ESP_OK) {
    return false;
  }

  return true;
}

void disableUnnecessaryBtFeatures() {
  // Disable scanning
  esp_bt_gap_cancel_discovery();
  
  // Disable device discovery
  esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  
  // Disable sniff mode
  esp_bt_sleep_disable();
  
  // Disable Secure Simple Pairing
  esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &esp_bt_io_cap_none, sizeof(uint8_t));
}

void configureCoexistence() {
  // Set WiFi as the preferred mode in coexistence
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  
  // Optionally, adjust other coexistence parameters
  // esp_coex_set_bt_tx_power(ESP_COEX_BT_TX_PWR_LOW);
  // esp_coex_set_wifi_tx_power(ESP_COEX_WIFI_TX_PWR_HIGH);
}
