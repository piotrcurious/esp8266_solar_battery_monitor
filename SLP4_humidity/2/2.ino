// Deep-sleep aware version that does promiscuous AP peek in setup()
// Save this as your main .ino/.cpp (replace previous file contents)

// includes (from your wifi_settings.h + others)
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP.h>
#include "wifi_settings.h"        // your provided header (contains ssid/password, struct, globals)
#include "telemetry_frame.hpp"
#include "SensorFilter.h"
#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2
#define SLEEP_DURATION_MS (1u * 1000u) // adjust as needed

#define ITERATIONS_PER_WAKEUP 16
#define ITERATIONS_PER_WAKEUP_RSSI 32
#define ITERATIONS_PER_WAKEUP_SHT4x 8

const bool idle_sensor_update = true;
const bool CONTINOUS_MODE = false;

WiFiUDP udp;
const uint16_t port = 5683;
IPAddress broadcastIP = IPAddress(224, 0, 1, 187);

// RTC persistence wrapper (we use ESP.rtcUserMemoryRead/Write)
#define RTC_MAGIC 0xA5A55A5AUL

#ifdef PERSISTENT_WIFI
// we reuse your WIFI_SETTINGS_T inside wrapper so the saved layout matches your header
struct rtc_store_t {
  uint32_t magic;           // magic to validate
  WIFI_SETTINGS_T settings; // your struct (includes wifi_ssid, wifi_auth, bssid, channel, ips)
};
#else
#error "PERSISTENT_WIFI must be defined in wifi_settings.h for RTC persistence"
#endif

// Global runtime variables (many already exist in wifi_settings.h)
WIFI_SETTINGS_T _settings; // populated from live wifi or RTC
bool initialConnectionEstablished = false;

void yield_delay(uint32_t delay_ms) {
  uint32_t start = millis();
  while (millis() - start < delay_ms) { yield(); }
}

// --- RTC helpers -------------------------------------------------------------
bool rtc_write(const rtc_store_t &r) {
  return ESP.rtcUserMemoryWrite(0, (uint32_t*)&r, sizeof(r));
}
bool rtc_read(rtc_store_t &r) {
  if (!ESP.rtcUserMemoryRead(0, (uint32_t*)&r, sizeof(r))) return false;
  return (r.magic == RTC_MAGIC);
}

// persist current runtime info into RTC
void persist_connection_params_to_rtc() {
  rtc_store_t r;
  memset(&r, 0, sizeof(r));
  r.magic = RTC_MAGIC;
  // fill WIFI_SETTINGS_T fields from _settings and runtime WiFi if connected
  // store IPs numeric (as the header expects uint32_t)
  r.settings.magic = 0xBEEF; // optional internal id if you like
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
    // copy BSSID/channel
    memcpy(r.settings.wifi_bssid, WiFi.BSSID(), 6);
    r.settings.wifi_channel = WiFi.channel();
    // store SSID/password copy
    strncpy(r.settings.wifi_ssid, WiFi.SSID().c_str(), sizeof(r.settings.wifi_ssid)-1);
    strncpy(r.settings.wifi_auth, WiFi.psk().c_str(), sizeof(r.settings.wifi_auth)-1);
  } else {
    // fallback to whatever is in compile-time ssid/password and current _settings
    r.settings.ip_address = _settings.ip_address;
    r.settings.ip_gateway = _settings.ip_gateway;
    r.settings.ip_mask    = _settings.ip_mask;
    r.settings.ip_dns1    = _settings.ip_dns1;
    r.settings.ip_dns2    = _settings.ip_dns2;
    memcpy(r.settings.wifi_bssid, _settings.wifi_bssid, 6);
    r.settings.wifi_channel = _settings.wifi_channel;
    strncpy(r.settings.wifi_ssid, ssid, sizeof(r.settings.wifi_ssid)-1);
    strncpy(r.settings.wifi_auth, password, sizeof(r.settings.wifi_auth)-1);
  }
  rtc_write(r);
}

// restore RTC -> _settings; returns true if valid rtc content
bool restore_rtc_to_settings() {
  rtc_store_t r;
  if (!rtc_read(r)) return false;
  // populate _settings from r.settings
  _settings.ip_address = r.settings.ip_address;
  _settings.ip_gateway = r.settings.ip_gateway;
  _settings.ip_mask    = r.settings.ip_mask;
  _settings.ip_dns1    = r.settings.ip_dns1;
  _settings.ip_dns2    = r.settings.ip_dns2;
  memcpy(_settings.wifi_bssid, r.settings.wifi_bssid, 6);
  _settings.wifi_channel = r.settings.wifi_channel;
  // ensure SSID/auth are present in r.settings.wifi_ssid/auth when used later
  return true;
}

// --- minimal SSID parser used in promiscuous callback ---------------------------------
// parse SSID out of tagged payload (same idea you used earlier)
String parse_ssid_from_tagged(const uint8_t *tagged, int len) {
  int pos = 0;
  while (pos + 2 <= len) {
    uint8_t tag = tagged[pos];
    uint8_t tlen = tagged[pos+1];
    if (tlen == 0) { pos += 2; continue; }
    if (tag == 0x00) { // SSID param
      if (tlen > 0 && tlen <= 32) {
        return String((const char*)(tagged + pos + 2), tlen);
      } else return String();
    }
    pos += (tlen + 2);
  }
  return String();
}

// temporary flag used during peek
volatile bool _peek_found = false;
volatile uint32_t _peek_rssi = -127;

// promiscuous callback used only for the peek
extern "C" void peek_promisc_cb(uint8_t *buf, uint16_t len) {
  if (len < sizeof(struct beacon_buf)) return;
  struct beacon_buf *bbuf = (struct beacon_buf*)buf;
  // check BSSID first (fast)
  if (memcmp(_settings.wifi_bssid, bbuf->bssid, 6) == 0) {
    _peek_found = true;
    _peek_rssi = bbuf->rssi;
    return;
  }
  // otherwise try SSID match (if available in RTC or compile-time)
  String ss = parse_ssid_from_tagged(bbuf->tagged, sizeof(bbuf->tagged));
  if (ss.length() > 0) {
    // check with RTC-stored SSID
    rtc_store_t r;
    if (rtc_read(r) && r.settings.wifi_ssid[0] != 0) {
      if (ss.equals(String(r.settings.wifi_ssid))) {
        _peek_found = true;
        _peek_rssi = bbuf->rssi;
        return;
      }
    }
    // fallback: check compile-time ssid
    if (ss.equals(String(ssid))) {
      _peek_found = true;
      _peek_rssi = bbuf->rssi;
      return;
    }
  }
}

// do a short promiscuous peek for AP presence; returns true if AP is seen
bool peek_for_ap(uint32_t timeout_ms) {
  _peek_found = false;
  _peek_rssi = -127;
  // set station mode & ensure channel (if known)
  wifi_set_opmode(STATION_MODE);
  // use stored channel if available
  if (_settings.wifi_channel != 0) {
    wifi_set_channel(_settings.wifi_channel);
  } else {
    wifi_set_channel(channel);
  }
  // set the temporary callback and enable promiscuous
  wifi_set_promiscuous_rx_cb(peek_promisc_cb);
  wifi_promiscuous_enable(1);
  uint32_t t0 = millis();
  while (! _peek_found && (millis() - t0) < timeout_ms) {
    yield();
  }
  // disable promiscuous and restore your normal callback if you use one elsewhere:
  wifi_promiscuous_enable(0);
  // if you had another promiscuous handler elsewhere, reassign it here:
  // wifi_set_promiscuous_rx_cb(promisc_cb); // uncomment if you have promisc_cb defined and want to restore
  return _peek_found;
}

// --- connection helper -------------------------------------------------------
bool fast_connect_using_rtc_or_compiled() {
  // prefer RTC copy of SSID/password if present
  rtc_store_t r;
  bool have_rtc = rtc_read(r);
  const char *use_ssid = have_rtc && r.settings.wifi_ssid[0] ? r.settings.wifi_ssid : ssid;
  const char *use_pass = have_rtc && r.settings.wifi_auth[0]  ? r.settings.wifi_auth  : password;
  const uint8_t *bssid = have_rtc ? r.settings.wifi_bssid : NULL;
  uint8_t use_channel = have_rtc ? r.settings.wifi_channel : channel;

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  if (bssid && use_channel != 0) {
    WiFi.begin(use_ssid, use_pass, use_channel, bssid, true);
  } else {
    WiFi.begin(use_ssid, use_pass, use_channel);
  }

  // initial short wait for sync (your original loop used mix of yield()/delays)
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < ASSOCIATE_TIMEOUT_INTERVAL) {
    yield_delay(50);
  }

  if (WiFi.status() == WL_CONNECTED) {
    // populate _settings from active connection
    _settings.ip_address = (uint32_t)WiFi.localIP();
    _settings.ip_gateway = (uint32_t)WiFi.gatewayIP();
    _settings.ip_mask    = (uint32_t)WiFi.subnetMask();
    _settings.ip_dns1    = (uint32_t)WiFi.dnsIP(0);
    _settings.ip_dns2    = (uint32_t)WiFi.dnsIP(1);
    memcpy(_settings.wifi_bssid, WiFi.BSSID(), 6);
    _settings.wifi_channel = WiFi.channel();
    // persist to RTC
    persist_connection_params_to_rtc();
    return true;
  }
  return false;
}

// --- setup & loop ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);

  // initialize sensor
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  // Try to restore RTC to _settings so the peek can use BSSID/channel if available
  bool rtc_ok = restore_rtc_to_settings();
  if (rtc_ok) {
    Serial.println("RTC data restored");
  } else {
    Serial.println("No RTC data");
    // If no RTC data, still populate _settings wiht compile-time defaults
    // ip fields remain zero
    memset(_settings.wifi_bssid, 0, 6);
    _settings.wifi_channel = channel;
  }

  // Immediately peek for AP presence using promiscuous mode (this runs after deep-sleep reset too)
  Serial.println("Peeking for AP...");
  bool ap_nearby = peek_for_ap(BEACON_PEEK_TIMEOUT); // BEACON_PEEK_TIMEOUT from header
  if (ap_nearby) {
    #ifdef CURRENT_BEACON_DEBUG
      Serial.printf("AP seen (rssi %d). Attempting fast connect...\n", (int)_peek_rssi);
    #endif
    // attempt fast connect (uses RTC-stored SSID/pass/BSSID/channel if present)
    initialConnectionEstablished = fast_connect_using_rtc_or_compiled();
    if (initialConnectionEstablished) {
      Serial.println("Fast connect OK");
      udp.begin(port);
    } else {
      Serial.println("Fast connect failed");
    }
  } else {
    Serial.println("No AP nearby; will skip association this wake");
    initialConnectionEstablished = false;
  }

  // Set promiscuous callback for normal operation (if you rely on it later)
  // wifi_set_promiscuous_rx_cb(promisc_cb); // uncomment if you defined promisc_cb for non-peek processing
  // Note: maintain wifi_promiscuous_enable(0) until explicit scanning is required elsewhere.
}

// (retain your SensorFilter / telemetry setup etc. - omitted here for brevity; add back from your original file)
// For purposes of illustration I keep only the transmit logic and deep-sleep decision:

// example minimal loop (integrate your original sensor code here):
void loop() {
  // if we have connection, do sensor reads and send
  if (initialConnectionEstablished && WiFi.status() == WL_CONNECTED) {
    // ... do sensor sampling & filtering like in your original loop
    telemetry_frame tframe;
    // sample sensors (use your existing filter logic)
    // send packet
    udp.beginPacketMulticast(broadcastIP, port, WiFi.localIP());
    // write telemetry_frame bytes; example:
    // udp.write((uint8_t*)&tframe, sizeof(tframe));
    udp.endPacket();
    #ifdef PACKET_SENT_TIME_DEBUG
      Serial.println("Packet sent");
    #endif
  } else {
    #ifdef PACKET_SENT_TIME_DEBUG
      Serial.println("Not connected this wake - skipping transmit");
    #endif
  }

  // Before deep sleep, persist settings so next wake can fast-peek/connect
  persist_connection_params_to_rtc();

  // Sleep decision (read MODE_PIN as in your original code)
  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH && !CONTINOUS_MODE) {
    // prepare for deep sleep
    WiFi.forceSleepBegin();
    delay(10);
    Serial.printf("Going to deep sleep for %u ms\n", (unsigned)SLEEP_DURATION_MS);
    // ensure RTC is written one last time
    persist_connection_params_to_rtc();
    ESP.deepSleep((uint64_t)SLEEP_DURATION_MS * 1000ULL);
    // execution will not continue here
    for (;;) yield();
  } else {
    // stay awake - your continuous-mode behavior
    delay(500);
    // if not connected attempt a reconnect by re-running peek & fast connect logic:
    if (!initialConnectionEstablished) {
      bool seen = peek_for_ap(BEACON_PEEK_TIMEOUT);
      if (seen) {
        initialConnectionEstablished = fast_connect_using_rtc_or_compiled();
      }
    }
  }
}
