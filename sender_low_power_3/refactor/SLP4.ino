#include "wifi_settings.h"
#include "telemetry_frame.h"
#include "user_interface.h"

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T _settings;
#endif

bool initialConnectionEstablished = false;

void fpm_wakup_cb_func(void) {
  Serial.println(F("Light sleep is over"));
  Serial.flush();  
  wifi_fpm_close();
}

bool initialConnectAndStoreParams() {
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t timeout = millis() + 20000; // 20s timeout
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    _settings.ip_address = WiFi.localIP();
    _settings.ip_gateway = WiFi.gatewayIP();
    _settings.ip_mask = WiFi.subnetMask();
    _settings.ip_dns1 = WiFi.dnsIP(0);
    _settings.ip_dns2 = WiFi.dnsIP(1);
    memcpy(_settings.wifi_bssid, WiFi.BSSID(), 6);
    _settings.wifi_channel = WiFi.channel();
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT);

  initialConnectionEstablished = initialConnectAndStoreParams();
  udp.begin(port);
  wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func);

  if (initialConnectionEstablished) {
    Serial.println("Connected OK");
      } else {
    Serial.println("Failed to establish initial connection");
  }
}

void reconnectAfterSleep() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    return;
  }

  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.config(_settings.ip_address, _settings.ip_gateway, _settings.ip_mask);
  WiFi.begin(ssid, password, _settings.wifi_channel, _settings.wifi_bssid, true);

  uint32_t timeout = millis() + 20000; // 20s timeout
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to reconnect after sleep");
    //initialConnectionEstablished = false; // no need as settings will stay same
    // it could be useful for a mesh with changing bssid though. 
  }
}

void loop() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    if (!initialConnectionEstablished) {
      delay(1000);  // Wait before retrying
      //todo: sleep in case there is no connection. 
    }
  }

  telemetry_frame tframe;
  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);

  udp.beginPacketMulticast(broadcast, port, WiFi.localIP());
  udp.write((byte*)&tframe, sizeof(tframe));
  udp.endPacket();
  delay(20); // todo : find better way to check if packet is sent

  if (digitalRead(MODE_PIN) == HIGH) {
    WiFi.mode(WIFI_OFF);
    delay(100);
    Serial.flush();

    extern os_timer_t *timer_list;
    timer_list = nullptr;

    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();

    long sleepTimeMilliSeconds = 10 * 1000;
    wifi_fpm_do_sleep(sleepTimeMilliSeconds * 1000);
    esp_delay(sleepTimeMilliSeconds + 1);

    reconnectAfterSleep();
  } else {
    delay(10); // delay in continuous mode. 3ms minimum. 
  }
}
