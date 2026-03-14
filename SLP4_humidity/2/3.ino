// Full integrated sketch for ESP8266
// - RTC persistence of WIFI_SETTINGS_T
// - promiscuous AP peek in setup() after wake
// - original promisc_cb & parse_tagged integrated
// - deep-sleep listen mode to save energy

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP.h>

#include "wifi_settings.h"        // your header with ssid/password, WIFI_SETTINGS_T, globals, etc.
#include "telemetry_frame.hpp"
#include "user_interface.h"
#include "SensorFilter.h"
#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

// Sleep durations (ms)
#define SLEEP_DURATION_MS (1u * 1000u) // base sleep duration (adjust)
#define DEEP_SLEEP_LISTEN_MS 200u      // how long to listen for AP immediately after wake (short -> saves energy)
#define ITERATIONS_PER_WAKEUP 16
#define ITERATIONS_PER_WAKEUP_RSSI 32
#define ITERATIONS_PER_WAKEUP_SHT4x 8

const bool idle_sensor_update = true;
const bool CONTINOUS_MODE = false;

WiFiUDP udp;
const int port = 5683;
IPAddress broadcast = IPAddress(224,0,1,187);

// --- RTC persistence ---------------------------------------------------------
#define RTC_MAGIC 0xA5A55A5AUL

#ifdef PERSISTENT_WIFI
struct rtc_store_t {
  uint32_t magic;
  WIFI_SETTINGS_T settings; // uses the struct you defined in wifi_settings.h
};
#else
#error "PERSISTENT_WIFI must be defined in wifi_settings.h"
#endif

static rtc_store_t rtc_cache;
static bool rtc_cache_ok = false;

// read/write RTC user memory (wrap ESP functions)
bool rtc_write(const rtc_store_t &r) {
  return ESP.rtcUserMemoryWrite(0, (uint32_t*)&r, sizeof(r));
}
bool rtc_read(rtc_store_t &r) {
  if (!ESP.rtcUserMemoryRead(0, (uint32_t*)&r, sizeof(r))) return false;
  return (r.magic == RTC_MAGIC);
}

void persist_connection_params_to_rtc() {
  rtc_store_t r;
  memset(&r, 0, sizeof(r));
  r.magic = RTC_MAGIC;

  // prefer live WiFi data if connected
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    IPAddress mask = WiFi.subnetMask();
    IPAddress d1 = WiFi.dnsIP(0);
    IPAddress d2 = WiFi.dnsIP(1);

    r.settings.ip_address = (uint32_t)ip;
    r.settings.ip_gateway = (uint32_t)gw;
    r.settings.ip_mask    = (uint32_t)mask;
    r.settings.ip_dns1    = (uint32_t)d1;
    r.settings.ip_dns2    = (uint32_t)d2;

    memcpy(r.settings.wifi_bssid, WiFi.BSSID(), 6);
    r.settings.wifi_channel = WiFi.channel();

    // copy SSID/password strings if available
    String ss = WiFi.SSID();
    String pw = WiFi.psk(); // psk() returns password when compiled with proper core
    if (ss.length() > 0) {
      strncpy(r.settings.wifi_ssid, ss.c_str(), sizeof(r.settings.wifi_ssid)-1);
    } else {
      strncpy(r.settings.wifi_ssid, ssid, sizeof(r.settings.wifi_ssid)-1);
    }
    if (pw.length() > 0) {
      strncpy(r.settings.wifi_auth, pw.c_str(), sizeof(r.settings.wifi_auth)-1);
    } else {
      strncpy(r.settings.wifi_auth, password, sizeof(r.settings.wifi_auth)-1);
    }
  } else {
    // fallback to previous stored values or compile-time
    // if rtc_cache is valid, reuse it; otherwise use compile-time ssid/password
    if (rtc_cache_ok) {
      r.settings = rtc_cache.settings;
    } else {
      r.settings.ip_address = 0;
      r.settings.ip_gateway = 0;
      r.settings.ip_mask    = 0;
      r.settings.ip_dns1    = 0;
      r.settings.ip_dns2    = 0;
      strncpy(r.settings.wifi_ssid, ssid, sizeof(r.settings.wifi_ssid)-1);
      strncpy(r.settings.wifi_auth, password, sizeof(r.settings.wifi_auth)-1);
      memset(r.settings.wifi_bssid, 0, 6);
      r.settings.wifi_channel = channel;
    }
  }

  rtc_write(r);
  // update local cache copy
  rtc_cache = r;
  rtc_cache_ok = true;
}

bool restore_rtc_to_cache() {
  rtc_store_t r;
  if (!rtc_read(r)) {
    rtc_cache_ok = false;
    return false;
  }
  rtc_cache = r;
  rtc_cache_ok = true;
  return true;
}

// --- utility delay/yield -----------------------------------------------------
void yield_delay(uint32_t delay_ms) {
  uint32_t startTime = millis();
  while (millis() - startTime < delay_ms) {
    yield();
  }
}

// --- AP scanner helpers (original functions integrated) ----------------------

// We'll keep the ap_info and aps storage even if BEACON_DEBUG is not defined, to allow internal bookkeeping.
#ifndef MAX_APS
#define MAX_APS 15
#endif

struct ap_info {
  uint8_t bssid[6];
  unsigned char essid[33];
  uint8_t essid_len;
  uint8_t chan;
  signed rssi;
  unsigned long last_seen;
};

ap_info aps[MAX_APS];
int known_aps = 0;

// original rssi comparator (kept)
int rssi_cmp(const void *cmp1, const void *cmp2) {
  struct ap_info ap1 = *((struct ap_info *)cmp1);
  struct ap_info ap2 = *((struct ap_info *)cmp2);
  if(ap2.rssi > ap1.rssi) return 1;
  if(ap1.rssi > ap2.rssi) return -1;
  return 0;
}

void print_ap(struct ap_info ap) {
  Serial.printf(" [Ch %02d] [%03d] [%s]\n", ap.chan, ap.rssi, ap.essid );
}

void add_ap(struct ap_info ap) {
    unsigned long cur_time = millis();
    // Check if it is a known ap
    for(int i=0;i<known_aps;i++) {
       if(!memcmp(aps[i].bssid, ap.bssid, 6)) {
          // If it's been less than 100ms, then only update the rssi upward
          // Otherwise, update it if it has changed.
          if( ((cur_time - aps[i].last_seen) <= 100 && aps[i].rssi <  ap.rssi) ||
              ((cur_time - aps[i].last_seen) >  100 && aps[i].rssi != ap.rssi) ) {
              aps[i].rssi = ap.rssi;
              //qsort(aps, known_aps, sizeof(struct ap_info), rssi_cmp);
          }
          aps[i].last_seen = cur_time;
          return;
        }
    }

    if (known_aps < MAX_APS) {
      memcpy(&aps[known_aps], &ap, sizeof(ap_info));
      aps[known_aps].last_seen = millis();
      known_aps++;
    }
    // Keep it sorted (optional)
    // qsort(aps, known_aps, sizeof(struct ap_info), rssi_cmp);
}

void print_all_aps() {
  int num_shown = 0;
  unsigned long cur_time = millis();

  for(int i=0;i<known_aps;i++) {
    if(cur_time - aps[i].last_seen < 10000) {
        num_shown++;
        print_ap(aps[i]);
    }
    if(num_shown==MAX_APS) return;
  }
}

// parse_tagged — uses your original implementation to extract SSID & channel
struct ap_info parse_tagged(uint8_t *tagged) {
    struct ap_info b;
    b.essid_len = 0;
    b.chan = 0;
    int pos = 0;
    uint8_t len = 0;
    uint8_t tag = 0;

    while (pos < 71) {
      tag = tagged[pos];
      len = tagged[pos+1];
      if(len <= 0) { pos += 2; continue; } // avoid infinite loop if malformed

      switch(tag) {
        case 0x00: // SSID
          if(len < 32) {
            b.essid_len = len;
            memset(b.essid, 0, 33);
            memcpy(b.essid, &tagged[pos+2], b.essid_len);
          }
          break;
        case 0x03: // DS Parameter set — channel
          b.chan = tagged[pos+2];
          break;
        default:
          break;
      }
      pos += (len + 2);
    }
    return b;
}

// --- promiscuous callback (full original) -----------------------------------
void promisc_cb(uint8_t *buf, uint16_t len)
{
    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
    if (len == 128) {
      struct ap_info ap = parse_tagged(bbuf->tagged);

      if(ap.essid_len>0) {
        ap.rssi = bbuf->rssi;
        memcpy(&ap.bssid, bbuf->bssid, 6);
        add_ap(ap);
      }

      // logic to check if "our" ap is on the list
      if(!memcmp(rtc_cache.settings.wifi_bssid, bbuf->bssid, 6)) {
        is_beacon_fresh = true; // it's fresh
        #ifdef CURRENT_BEACON_DEBUG
          Serial.print("fresh : ");
          Serial.print(millis());
          Serial.print(" ,delta :");
          Serial.println(millis()-last_beacon_timestamp);
        #endif
        last_beacon_timestamp = millis();
      }
    } else {
      // even if len != 128, still check BSSID match (some SDK versions give different len)
      if(!memcmp(rtc_cache.settings.wifi_bssid, bbuf->bssid, 6)) {
        is_beacon_fresh = true;
        last_beacon_timestamp = millis();
      }
    }
}

// --- AP peek used immediately after wake -------------------------------------
bool peek_for_ap(uint32_t timeout_ms) {
  // Ensure STATION mode and set channel
  wifi_set_opmode(STATION_MODE);
  if (rtc_cache_ok && rtc_cache.settings.wifi_channel != 0) {
    wifi_set_channel((uint8_t)rtc_cache.settings.wifi_channel);
  } else {
    wifi_set_channel(channel);
  }

  // Clear local AP list and freshness flag
  memset(aps, 0, sizeof(aps));
  known_aps = 0;
  is_beacon_fresh = false;
  last_beacon_timestamp = millis();

  // Set callback and enable promiscuous
  wifi_set_promiscuous_rx_cb(promisc_cb);
  wifi_promiscuous_enable(1);

  uint32_t t0 = millis();
  while (!is_beacon_fresh && millis() - t0 < timeout_ms) { yield(); }

  wifi_promiscuous_enable(0);

  #ifdef BEACON_DEBUG
    print_all_aps();
  #endif

  return is_beacon_fresh;
}

// --- WiFi connect & store (adapted, but behavior kept) -----------------------
bool initialConnectAndStoreParams() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  // Try to use stored BSSID+channel if we have them in rtc_cache
  if (rtc_cache_ok && rtc_cache.settings.wifi_bssid[0] != 0 && rtc_cache.settings.wifi_channel != 0) {
    WiFi.begin(rtc_cache.settings.wifi_ssid, rtc_cache.settings.wifi_auth, rtc_cache.settings.wifi_channel, rtc_cache.settings.wifi_bssid);
  } else {
    WiFi.begin(ssid, password, channel);
  }

  uint32_t timeout = 20 * 1000; // waiting segments
  pinMode(LED_PIN, OUTPUT);

  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    digitalWrite(LED_PIN, LOW);
    for (uint16_t how_many_yields_is_ms = 0; how_many_yields_is_ms < 10; how_many_yields_is_ms++) { yield(); }
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    if (timeout > 0) timeout--;
  }
  pinMode(LED_PIN, INPUT);

  // second pass to finalize DHCP
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  if (rtc_cache_ok && rtc_cache.settings.wifi_bssid[0] != 0 && rtc_cache.settings.wifi_channel != 0) {
    WiFi.begin(rtc_cache.settings.wifi_ssid, rtc_cache.settings.wifi_auth, rtc_cache.settings.wifi_channel, rtc_cache.settings.wifi_bssid, true);
  } else {
    WiFi.begin(ssid, password, channel);
  }

  timeout = millis();
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && millis() - timeout < 20000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield_delay(100);
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

    // persist live connection params into RTC
    persist_connection_params_to_rtc();
    return true;
  }
  return false;
}

// --- UDP send (kept) --------------------------------------------------------
void sendUDPPacket(const telemetry_frame& tframe) {
  if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
    size_t bytesWritten = udp.write((uint8_t*)&tframe, sizeof(tframe));
    if (bytesWritten == sizeof(tframe)) {
      udp.endPacket();
      yield_delay(20);
    }
  }
}

// --- reconnectAfterSleep (keeps earlier behavior) ----------------------------
void reconnectAfterSleep() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    return;
  }

  // peek to see if our AP is near
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(0);
#ifdef BEACON_DEBUG
  memset(aps,0,sizeof(aps));
  known_aps=0;
#endif
  last_beacon_timestamp = millis();

  is_beacon_fresh = false;
  wifi_promiscuous_enable(1);

  uint32_t timeout = millis();
  while(!is_beacon_fresh && millis()-timeout < BEACON_PEEK_TIMEOUT) { yield(); }
  wifi_promiscuous_enable(0);

  if (!is_beacon_fresh) { Serial.println("no AP"); return; }
#ifdef BEACON_DEBUG
  print_all_aps();
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(_settings.ip_address, _settings.ip_gateway, _settings.ip_mask);
  WiFi.begin(rtc_cache.settings.wifi_ssid, rtc_cache.settings.wifi_auth, _settings.wifi_channel, _settings.wifi_bssid, true);

  transmit_phase = true;
}

// --- wait_for_connection (kept) ---------------------------------------------
void wait_for_connection () {
  uint32_t timeout = millis();
  pinMode(LED_PIN, OUTPUT);
  while ((WiFi.status() != WL_CONNECTED )&&  ((millis()-timeout) < ASSOCIATE_TIMEOUT_INTERVAL)  ) {
    digitalWrite(LED_PIN, HIGH);
    yield_delay(25);
    digitalWrite(LED_PIN,LOW);
    yield_delay(5);
  }
  pinMode(LED_PIN, INPUT);
}

// --- SensorFilters, telemetry and globals (kept from original) --------------
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

bool initialConnectionEstablished = false;

// --- setup -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);

  // Read RTC cache (if any)
  if (restore_rtc_to_cache()) {
    Serial.println("RTC data found - restored parameters.");
  } else {
    Serial.println("No RTC data.");
    // initialize _settings with defaults
    memset(&_settings, 0, sizeof(_settings));
    _settings.wifi_channel = channel;
  }

  // SHT4x initialization (same as original)
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_HIGH_HEATER_1S);
  sht4.setHeater(SHT4X_NO_HEATER);

  // Deep-sleep wake: do quick promiscuous peek to decide whether to attempt association
  Serial.println("Peek for AP (deep-sleep wake) ...");
  bool ap_nearby = peek_for_ap(DEEP_SLEEP_LISTEN_MS);

  if (!ap_nearby) {
    // No AP heard in short listen window: save state and go back to deep sleep (energy saving)
    Serial.println("No AP detected on short listen; going back to deep sleep to save energy.");
    // increment sleep_periods_elapsed and persist
    sleep_periods_elapsed++;
    persist_connection_params_to_rtc();
    // final housekeeping then deep sleep
    WiFi.forceSleepBegin();
    delay(10);
    ESP.deepSleep((uint64_t)SLEEP_DURATION_MS * 1000ULL);
    while (1) yield();
  }

  // If AP found, attempt fast connect using rtc_cache values (if present)
  Serial.println("AP detected in peek; attempting fast connect...");
  initialConnectionEstablished = initialConnectAndStoreParams();
  udp.begin(port);
  wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func); // if still used elsewhere

  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(promisc_cb);

  if (initialConnectionEstablished) {
    Serial.println("Connected OK after peek");
  } else {
    Serial.println("Failed to establish initial connection after peek");
  }

  // Ensure RTC has latest info
  persist_connection_params_to_rtc();
}

// --- loop (keeps your original behavior, adapted to deep-sleep logic) ----------
void loop() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    if (!initialConnectionEstablished) {
      yield_delay(1000);  // Wait before retrying
      // If it fails repeatedly, next wake's peek will decide whether to try again
    }
  }

#ifdef PACKET_SENT_TIME_DEBUG
  uint32_t wait_for_connection_timer ;
  if (transmit_phase) { wait_for_connection_timer = millis(); }
#endif

  if (transmit_phase || idle_sensor_update) {
    for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP; i++) {
      voltage_ADC0 = analogRead(analogInPin) * (17.0/1024.0);
      tframe.voltage_ADC0 = Filter_voltage_ADC0.updateSensorReading(voltage_ADC0);
    }

    sensors_event_t humidity, temp;
    for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP_SHT4x; i++) {
      sht4.getEvent(&humidity, &temp);
      tframe.SH4x_rel_humidity = Filter_aHumidity.updateSensorReading(humidity.relative_humidity);
      tframe.SH4x_temperature = Filter_aTemperature.updateSensorReading(temp.temperature);
    }

    if (transmit_phase) {
      wait_for_connection(); // wait for connection only if it's transmit phase
      for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP_RSSI; i++) {
        if (WiFi.status() == WL_CONNECTED) { wifi_rssi = WiFi.RSSI(); }
        if (WiFi.status() == WL_CONNECTED) { tframe.wifi_rssi = Filter_wifi_rssi.updateSensorReading(wifi_rssi); }
      }
    }
  }

  if (transmit_phase) {
    radio_active_time = millis() - (uint32_t)0; // optional instrumentation; original used wait_for_connection_timer
    tframe.radio_active_time = radio_active_time;
  }

  if (WiFi.status() == WL_CONNECTED) { sendUDPPacket(tframe); }

#ifdef PACKET_SENT_TIME_DEBUG
  if (transmit_phase) { Serial.print("sent in : "); Serial.println(radio_active_time); }
#endif

  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH && !CONTINOUS_MODE ) {
    // go to deep sleep
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
    transmit_phase = false;
    WiFi.forceSleepBegin();
    yield_delay(20);
    Serial.flush();

    // keep timer list null as earlier
    extern os_timer_t *timer_list;
    timer_list = nullptr;

    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();

    // persist before sleeping
    persist_connection_params_to_rtc();

    uint64_t sleepTimeMilliSeconds = SLEEP_DURATION_MS;
    // use deepSleep: ESP.deepSleep expects microseconds
    ESP.deepSleep(sleepTimeMilliSeconds * 1000ULL);
    // code doesn't continue after deepSleep; but just in case:
    delay(10);
    while (1) yield();
  } else {
    transmit_phase = true; // always on in continuous mode
    yield_delay(500);
    if (WiFi.status() != WL_CONNECTED) { transmit_phase = false; reconnectAfterSleep(); }
  }
}
