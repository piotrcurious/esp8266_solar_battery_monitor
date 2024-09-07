#include "esp_wifi.h"

void configureWiFiAuthSettings() {
    wifi_config_t wifi_config = {};
    esp_wifi_get_config(WIFI_IF_AP, &wifi_config);

    // 1. Authentication Timeout
    wifi_config.ap.authmode_threshold = 10000;  // milliseconds, default is 5000

    // 2. Beacon Interval
    wifi_config.ap.beacon_interval = 100;  // milliseconds, default is 100

    // 3. Probe Response Timeout
    wifi_config.ap.min_connection_time = 2000;  // milliseconds, default is 1000

    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

    // 4. Association ID Timeout
    uint16_t aid_timeout = 1000;  // seconds, default is 300
    esp_wifi_set_associd_timeout(aid_timeout);

    // 5. WiFi Management Frame Retransmission
    wifi_pmf_config_t pmf_config = {
        .capable = true,
        .required = false
    };
    esp_wifi_set_pmf_config(&pmf_config);

    // 6. WiFi Enterprise Settings (if applicable)
    // wifi_config.ap.wpa_auth = WPA2_AUTH_ENT;
    // esp_wifi_sta_wpa2_ent_enable();

    // 7. Maximum Retry Attempts
    esp_wifi_set_max_tx_power(8);  // dBm, range: 2 dBm to 20 dBm

    // Apply the configuration
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
}
