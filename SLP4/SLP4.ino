#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "user_interface.h"

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

//#define PERSISTENT_WIFI
//
#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T _settings;
#endif

bool initialConnectionEstablished = false;

void yield_delay(uint32_t delay) {
        uint32_t startTime = millis();
        while (millis() - startTime < delay) { // Maximum wait time of 50ms
          yield(); // Allow the ESP8266 to handle background tasks
        }
        // exit after wait
        return ;
      }

void fpm_wakup_cb_func(void) {
  Serial.println(F("woke up"));
  Serial.flush();  
  wifi_fpm_close();
}

bool initialConnectAndStoreParams() {
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t timeout = millis() + 20000; // 20s timeout
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield_delay(200);
  }
    pinMode(LED_PIN, INPUT);


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

int sendUDPPacket(const telemetry_frame& tframe) {
  if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
    size_t bytesWritten = udp.write((uint8_t*)&tframe, sizeof(tframe));
    if (bytesWritten == sizeof(tframe)) {
      if (udp.endPacket()) {
        // Wait for the packet to be sent
        unsigned long startTime = millis();
        while (millis() - startTime < 50) { // Maximum wait time of 50ms
            yield();
            udp.flush();
            yield_delay(5); // just to be sure. not sure if it is needed, but it seems there is no other way.
            // No packets pending in the transmit buffer
            return millis()-startTime;
          
          yield(); // Allow the ESP8266 to handle background tasks
        }
        // If we've waited too long, assume there was an issue
        return 50;
      }
    }
  }
  return -1;
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT);
  Serial.println("Connecting...");
  uint32_t timing_connecting = millis();
  initialConnectionEstablished = initialConnectAndStoreParams();
  udp.begin(port);
  wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func);

  if (initialConnectionEstablished) {
    Serial.println("Connected OK");
    Serial.println(millis()-timing_connecting);
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

  uint32_t timeout = millis();
  pinMode(LED_PIN, OUTPUT);
//  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
  while ((WiFi.status() != WL_CONNECTED )&&  ((millis()-timeout) < ASSOCIATE_TIMEOUT_INTERVAL)  ) {

    digitalWrite(LED_PIN, HIGH);
    yield_delay(10);
    digitalWrite(LED_PIN,LOW);
    yield_delay(50);
  }
  pinMode(LED_PIN, INPUT);

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
      yield_delay(1000);  // Wait before retrying
      //todo: sleep in case there is no connection. 
    }
  }

  telemetry_frame tframe;
  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);
  if (WiFi.status() == WL_CONNECTED) {tframe.wifi_rssi = WiFi.RSSI();}

 // udp.beginPacketMulticast(broadcast, port, WiFi.localIP());
 // udp.write((byte*)&tframe, sizeof(tframe));
 // udp.endPacket();
 // yield_delay(20); // todo : find better way to check if packet is sent

   if (WiFi.status() == WL_CONNECTED) {Serial.println(sendUDPPacket(tframe));} // print how long it took to send a packet in ms. 

  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH) {
    WiFi.mode(WIFI_OFF);
    yield_delay(100);
    Serial.flush();

    extern os_timer_t *timer_list;
    timer_list = nullptr;

    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();

    uint32_t sleepTimeMilliSeconds = 10 * 1000;
    wifi_fpm_do_sleep(sleepTimeMilliSeconds * 1000);
    esp_delay(sleepTimeMilliSeconds + 1);

    reconnectAfterSleep();
  } else {
    yield_delay(10); // delay in continuous mode. 3ms minimum. 
  }
}
