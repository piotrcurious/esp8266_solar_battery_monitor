// Improved sketch: deep-sleep + RTC memory persistence for ESP8266
// Based on the code you provided, modified to:
//  - store necessary runtime/connection parameters in RTC memory
//  - use ESP.deepSleep(...) instead of wifi_fpm light sleep
//  - restore parameters on wake and perform fast reconnect using stored channel/BSSID

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP.h>
#include "user_interface.h"

#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "SensorFilter.h"
#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

// Fix define (removed stray semicolon). SLEEP_DURATION is in milliseconds.
#define SLEEP_DURATION_MS (1u * 1000u) // set to 1000 ms for example. adjust as needed.

#define ITERATIONS_PER_WAKEUP 16
#define ITERATIONS_PER_WAKEUP_RSSI 32
#define ITERATIONS_PER_WAKEUP_SHT4x 8

const bool idle_sensor_update = true;
const bool CONTINOUS_MODE = false;

// -----------------------------------------------------------------------------
// RTC storage layout
// -----------------------------------------------------------------------------
#define RTC_MAGIC 0xA5A55A5AUL
struct rtc_store_t {
  uint32_t magic;                     // magic marker to validate RTC contents
  uint32_t sleep_periods_elapsed;
  uint8_t initialConnectionEstablished; // boolean as byte
  uint8_t transmit_phase;             // boolean as byte
  uint8_t wifi_bssid[6];
  uint8_t wifi_channel;
  uint8_t ip_address[4];
  uint8_t ip_gateway[4];
  uint8_t ip_mask[4];
  uint8_t ip_dns1[4];
  uint8_t ip_dns2[4];
  // store ssid/password copies (null-terminated guaranteed)
  char ssid[32];
  char password[64];
  uint8_t reserved[8]; // room for future usage, keep struct 32/64 aligned-ish
};
STATIC_ASSERT(sizeof(rtc_store_t) <= 512); // sanity check: RTC memory small on ESP8266

// helper to read/write RTC user memory (wrapping ESP functions)
bool rtc_write(const rtc_store_t &s) {
  // ESP.rtcUserMemoryWrite expects pointer to uint32_t; pass size in bytes
  return ESP.rtcUserMemoryWrite(0, (uint32_t*)&s, sizeof(s));
}

bool rtc_read(rtc_store_t &s) {
  // If read returns false we treat as invalid
  if (!ESP.rtcUserMemoryRead(0, (uint32_t*)&s, sizeof(s))) return false;
  return true;
}

// -----------------------------------------------------------------------------
// Variables (preserve names used in your original code)
// -----------------------------------------------------------------------------
#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T _settings;
#else
// If _settings isn't declared elsewhere, create a local minimal structure for runtime usage.
// This will be populated from RTC or live WiFi after connect.
struct WIFI_SETTINGS_T {
  IPAddress ip_address;
  IPAddress ip_gateway;
  IPAddress ip_mask;
  IPAddress ip_dns1;
  IPAddress ip_dns2;
  uint8_t wifi_bssid[6];
  uint8_t wifi_channel;
};
WIFI_SETTINGS_T _settings;
#endif

bool initialConnectionEstablished = false;
volatile uint32_t sleep_periods_elapsed = 0;
volatile bool transmit_phase = true;

WiFiUDP udp;
IPAddress broadcast; // you already used broadcast & port earlier; ensure they exist in your headers
uint16_t port = 12345; // default port (override if you set it elsewhere)

void yield_delay(uint32_t delay_ms) {
  uint32_t startTime = millis();
  while (millis() - startTime < delay_ms) {
    yield();
  }
}

// Callback used earlier for promiscuous wake etc (keep as-is)
void fpm_wakup_cb_func(void) {
  Serial.println(F("woke up"));
  Serial.flush();
  wifi_fpm_close();
}

// Save runtime connection parameters to RTC memory (call after successful connection)
void persist_connection_params_to_rtc() {
  rtc_store_t r = {0};
  r.magic = RTC_MAGIC;
  r.sleep_periods_elapsed = sleep_periods_elapsed;
  r.initialConnectionEstablished = initialConnectionEstablished ? 1 : 0;
  r.transmit_phase = transmit_phase ? 1 : 0;
  // copy BSSID/channel
  memcpy(r.wifi_bssid, _settings.wifi_bssid, 6);
  r.wifi_channel = _settings.wifi_channel;
  // copy IPs
  IPAddress ip = WiFi.localIP();
  IPAddress gw = WiFi.gatewayIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress d1 = WiFi.dnsIP(0);
  IPAddress d2 = WiFi.dnsIP(1);
  for (int i = 0; i < 4; ++i) { r.ip_address[i] = ip[i]; r.ip_gateway[i] = gw[i]; r.ip_mask[i] = mask[i]; r.ip_dns1[i] = d1[i]; r.ip_dns2[i] = d2[i]; }
  // store a copy of SSID/password so we can reconnect quickly after deep sleep
  // prefer WiFi.SSID()/WiFi.psk() if available; fallback to global ssid/password if defined
  String sssid = WiFi.SSID();
  String ppass = WiFi.psk();
#ifdef ssid
  // nothing: keep runtime SSID first
#endif
  if (sssid.length() == 0) {
    // fallback: if you have global ssid/password in wifi_settings.h, copy them
    strncpy(r.ssid, ssid, sizeof(r.ssid) - 1);
    strncpy(r.password, password, sizeof(r.password) - 1);
  } else {
    sssid.toCharArray(r.ssid, sizeof(r.ssid));
    ppass.toCharArray(r.password, sizeof(r.password));
  }
  rtc_write(r);
}

// Restore RTC memory into runtime variables. Returns true if valid data available.
bool restore_rtc_to_runtime() {
  rtc_store_t r;
  if (!rtc_read(r)) return false;
  if (r.magic != RTC_MAGIC) return false;
  sleep_periods_elapsed = r.sleep_periods_elapsed;
  initialConnectionEstablished = (r.initialConnectionEstablished != 0);
  transmit_phase = (r.transmit_phase != 0);
  memcpy(_settings.wifi_bssid, r.wifi_bssid, 6);
  _settings.wifi_channel = r.wifi_channel;
  _settings.ip_address = IPAddress(r.ip_address[0], r.ip_address[1], r.ip_address[2], r.ip_address[3]);
  _settings.ip_gateway  = IPAddress(r.ip_gateway[0], r.ip_gateway[1], r.ip_gateway[2], r.ip_gateway[3]);
  _settings.ip_mask     = IPAddress(r.ip_mask[0], r.ip_mask[1], r.ip_mask[2], r.ip_mask[3]);
  _settings.ip_dns1     = IPAddress(r.ip_dns1[0], r.ip_dns1[1], r.ip_dns1[2], r.ip_dns1[3]);
  _settings.ip_dns2     = IPAddress(r.ip_dns2[0], r.ip_dns2[1], r.ip_dns2[2], r.ip_dns2[3]);

  // restore SSID/password into global variables if present or use them for fast connect
  // We will use r.ssid/r.password directly where needed.
  return true;
}

// -----------------------------------------------------------------------------
// initial connection function (adapted slightly to allow fast connect using stored BSSID/channel
// -----------------------------------------------------------------------------
bool initialConnectAndStoreParamsFast(const char *use_ssid = nullptr, const char *use_password = nullptr, const uint8_t *use_bssid = nullptr, uint8_t use_channel = 0xFF) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  // If we have a known BSSID+channel, attempt fast connect with it
  if (use_bssid && use_channel != 0xFF) {
    WiFi.begin(use_ssid ? use_ssid : ssid, use_password ? use_password : password, use_channel, use_bssid);
  } else {
    // normal begin
    WiFi.begin(use_ssid ? use_ssid : ssid, use_password ? use_password : password, channel);
  }

  // initial connect synchronization
  uint32_t timeout_ticks = 20 * 1000;
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && timeout_ticks > 0) {
    digitalWrite(LED_PIN, LOW);
    for (uint8_t i = 0; i < 10; ++i) { yield(); }
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    if (timeout_ticks > 0) timeout_ticks--;
  }
  pinMode(LED_PIN, INPUT);

  // second attempt with persistent true to finalize DHCP
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  if (use_bssid && use_channel != 0xFF) {
    WiFi.begin(use_ssid ? use_ssid : ssid, use_password ? use_password : password, use_channel, use_bssid, true);
  } else {
    WiFi.begin(use_ssid ? use_ssid : ssid, use_password ? use_password : password, channel);
  }

  uint32_t t0 = millis();
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield_delay(100);
  }
  pinMode(LED_PIN, INPUT);

  if (WiFi.status() == WL_CONNECTED) {
    // populate _settings from live WiFi
    _settings.ip_address = WiFi.localIP();
    _settings.ip_gateway = WiFi.gatewayIP();
    _settings.ip_mask    = WiFi.subnetMask();
    _settings.ip_dns1    = WiFi.dnsIP(0);
    _settings.ip_dns2    = WiFi.dnsIP(1);
    memcpy(_settings.wifi_bssid, WiFi.BSSID(), 6);
    _settings.wifi_channel = WiFi.channel();

    // persist useful params to RTC
    persist_connection_params_to_rtc();
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// sendUDPPacket: adapt to your existing code (keeps your yield_delay call)
// -----------------------------------------------------------------------------
void sendUDPPacket(const telemetry_frame& tframe) {
  if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
    size_t bytesWritten = udp.write((uint8_t*)&tframe, sizeof(tframe));
    if (bytesWritten == sizeof(tframe)) {
      udp.endPacket();
      yield_delay(20);
    }
  }
}

// -----------------------------------------------------------------------------
// Setup: restore RTC memory if possible, do initial connect attempt if needed
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);

  // Restore RTC-stored parameters if present (fast path)
  bool rtc_ok = restore_rtc_to_runtime();
  if (rtc_ok) {
    Serial.println("RTC data found - restored parameters.");
  } else {
    Serial.println("No valid RTC data.");
  }

  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_HIGH_HEATER_1S);
  sht4.setHeater(SHT4X_NO_HEATER);

  Serial.println("Connecting...");

  // If RTC had saved SSID/password we can try fast connect, otherwise regular connect
  rtc_store_t rtmp;
  bool have_ssid_in_rtc = false;
  if (rtc_read(rtmp) && rtmp.magic == RTC_MAGIC && rtmp.ssid[0] != 0) {
    have_ssid_in_rtc = true;
  }

  if (have_ssid_in_rtc) {
    initialConnectionEstablished = initialConnectAndStoreParamsFast(rtmp.ssid, rtmp.password, rtmp.wifi_bssid, rtmp.wifi_channel);
  } else {
    initialConnectionEstablished = initialConnectAndStoreParamsFast(nullptr, nullptr, nullptr, 0xFF);
  }

  udp.begin(port);
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(promisc_cb);

  if (initialConnectionEstablished) {
    Serial.println("Connected OK");
  } else {
    Serial.println("Failed to establish initial connection");
  }

  // Ensure saved latest params in RTC
  persist_connection_params_to_rtc();
}

// -----------------------------------------------------------------------------
// sensor/filter instantiation (kept from your original code)
// -----------------------------------------------------------------------------
SensorFilter Filter_voltage_ADC0(16, 0.1, 0.1, 0.1, 0.3);
SensorFilter Filter_wifi_rssi(128, 2.0, 0.01, 0.8, 0.001);
SensorFilter Filter_aTemperature(32, 0.1, 0.01, 0.01, 0.001);
SensorFilter Filter_aHumidity(32, 0.2, 0.01, 0.01, 0.001);

telemetry_frame tframe;
float voltage_ADC0 = 0;
uint32_t radio_active_time = 0;
float wifi_rssi = 0;
float aTemperature = 0.0;
float aHumidity = 0.0;

// -----------------------------------------------------------------------------
// promisc callback & other helpers (kept as in your original; make sure these
// functions exist in your original compilation context)
// -----------------------------------------------------------------------------
void promisc_cb(uint8_t *buf, uint16_t len) {
  struct beacon_buf *bbuf = (struct beacon_buf*) buf;
  if (!memcmp(_settings.wifi_bssid, bbuf->bssid, 6)) {
    // mark fresh beacon
    // you had is_beacon_fresh and last_beacon_timestamp in original; keep logic if present
    // For brevity, just set a flag if you implement it outside
  }
}

void reconnectAfterSleep() {
  // Try to use saved params in _settings to re-associate quickly
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(_settings.wifi_channel);
  wifi_promiscuous_enable(0);
  last_beacon_timestamp = millis(); // assuming this global exists in your larger program
  is_beacon_fresh = false;
  wifi_promiscuous_enable(1);
  uint32_t timeout = millis();
  while (!is_beacon_fresh && millis() - timeout < BEACON_PEEK_TIMEOUT) { yield(); }
  wifi_promiscuous_enable(0);
  if (!is_beacon_fresh) { Serial.println("no AP"); return; }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(_settings.ip_address, _settings.ip_gateway, _settings.ip_mask);
  WiFi.begin(rtmp.ssid, rtmp.password, _settings.wifi_channel, _settings.wifi_bssid, true);
  transmit_phase = true;
}

// -----------------------------------------------------------------------------
// loop: reads sensors, transmits, then deep-sleeps if in non-continuous mode
// -----------------------------------------------------------------------------
void loop() {
  // If we lost the initial connection flag, try to reconnect (fast or not)
  if (!initialConnectionEstablished) {
    rtc_store_t rt;
    if (rtc_read(rt) && rt.magic == RTC_MAGIC) {
      initialConnectionEstablished = initialConnectAndStoreParamsFast(rt.ssid, rt.password, rt.wifi_bssid, rt.wifi_channel);
    } else {
      initialConnectionEstablished = initialConnectAndStoreParamsFast(nullptr, nullptr, nullptr, 0xFF);
    }
    if (!initialConnectionEstablished) {
      yield_delay(1000); // small backoff, then proceed (we will deep-sleep later)
    }
  }

  if (transmit_phase || idle_sensor_update) {
    for (uint8_t i = 0; i < ITERATIONS_PER_WAKEUP; i++) {
      voltage_ADC0 = analogRead(analogInPin) * (17.0 / 1024.0);
      tframe.voltage_ADC0 = Filter_voltage_ADC0.updateSensorReading(voltage_ADC0);
    }

    sensors_event_t humidity, temp;
    for (uint8_t i = 0; i < ITERATIONS_PER_WAKEUP_SHT4x; i++) {
      sht4.getEvent(&humidity, &temp);
      tframe.SH4x_rel_humidity = Filter_aHumidity.updateSensorReading(humidity.relative_humidity);
      tframe.SH4x_temperature = Filter_aTemperature.updateSensorReading(temp.temperature);
    }

    if (transmit_phase) {
      // compute RSSI over iterations
      for (uint8_t i = 0; i < ITERATIONS_PER_WAKEUP_RSSI; i++) {
        if (WiFi.status() == WL_CONNECTED) wifi_rssi = WiFi.RSSI();
        if (WiFi.status() == WL_CONNECTED) tframe.wifi_rssi = Filter_wifi_rssi.updateSensorReading(wifi_rssi);
      }
    }
  }

  // Transmit if connected
  if (WiFi.status() == WL_CONNECTED) {
    uint32_t before = millis();
    sendUDPPacket(tframe);
    radio_active_time = millis() - before;
    tframe.radio_active_time = radio_active_time;
  }

  // Save updated parameters into RTC memory before sleeping
  persist_connection_params_to_rtc();

  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH && !CONTINOUS_MODE) {
    // go to deep sleep (very low power)
    WiFi.disconnect(true);
    WiFi.forceSleepBegin();
    yield_delay(20);
    Serial.flush();

    // Choose sleep duration
    uint64_t sleep_us = (uint64_t)SLEEP_DURATION_MS * 1000ull;
    Serial.printf("Going to deep sleep for %u ms\n", (unsigned)SLEEP_DURATION_MS);

    // Write RTC again to ensure persistence
    persist_connection_params_to_rtc();

    // Allow serial to flush
    delay(10);

    // Deep sleep - this resets the CPU; on wake we restore RTC
    ESP.deepSleep(sleep_us);
    // code will not run beyond this point (ESP resets)
    while (1) yield();
  } else {
    // Continuous mode: stay awake and loop
    transmit_phase = true;
    yield_delay(500);
    if (WiFi.status() != WL_CONNECTED) {
      transmit_phase = false;
      // Try to reconnect quickly using RTC values (non-blocking in your original logic)
      rtc_store_t rr;
      if (rtc_read(rr) && rr.magic == RTC_MAGIC) {
        initialConnectAndStoreParamsFast(rr.ssid, rr.password, rr.wifi_bssid, rr.wifi_channel);
      } else {
        // fallback to normal reconnect
        initialConnectAndStoreParamsFast(nullptr, nullptr, nullptr, 0xFF);
      }
    }
  }
}
