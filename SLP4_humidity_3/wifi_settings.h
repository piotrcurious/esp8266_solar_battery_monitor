#ifndef WIFI_SETTINGS_H
#define WIFI_SETTINGS_H

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Debug flags
//#define BEACON_DEBUG
#define CURRENT_BEACON_DEBUG // report fact of capturing AP nearby
#define PACKET_SENT_TIME_DEBUG // report how long it took to reassociate and send packet

// Timeouts
#define ASSOCIATE_TIMEOUT_INTERVAL 10000 // 10 seconds
#define BEACON_PEEK_TIMEOUT 1000 // 1 second (10 * 100ms beacon interval)
#define DEEP_SLEEP_LISTEN_MS 200 // how long to listen for AP immediately after wake

// Sleep settings
#define SLEEP_DURATION_MS (1 * 60 * 1000) // 1 minute
#define SLEEP_PERIODS_TO_SEND_PACKET 10 // send packet only once per [x] sleep periods.

// WiFi credentials
extern const char* ssid;
extern const char* password;
extern const uint8_t channel;

// Network settings
extern WiFiUDP udp;
extern const int port;
extern IPAddress broadcast;

extern bool is_beacon_fresh ; // set by promiscious sniffer
extern uint32_t last_beacon_timestamp ; //

// Global state flags (for runtime)
extern uint8_t sleep_periods_elapsed;
extern bool transmit_phase;

// RTC Persistence
#define RTC_MAGIC 0xA5A55A5AUL

struct KalmanState {
    float estimate;
    float errorCovariance;
};

struct rtc_store_t {
    uint32_t magic;
    uint32_t sleep_periods_elapsed;
    uint32_t total_packets;

    // WiFi params
    uint32_t ip_address;
    uint32_t ip_gateway;
    uint32_t ip_mask;
    uint32_t ip_dns1;
    uint32_t ip_dns2;
    uint8_t wifi_bssid[6];
    uint16_t wifi_channel;
    char wifi_ssid[32];
    char wifi_auth[64];

    // Filter states
    KalmanState filter_voltage;
    KalmanState filter_rssi;
    KalmanState filter_temp;
    KalmanState filter_humidity;

    uint8_t reserved[32];
};

// structures useful for wifi scanner
struct beacon_buf {
  signed rssi : 8;
  uint8_t rxctl[11];
  uint8_t wifiheader[16];
  uint8_t bssid[6];
  uint8_t fragment[2];
  uint8_t fixed[12];
  uint8_t tagged[76];
  uint16_t cnt;
  uint16_t len;
};

struct ap_info {
  uint8_t bssid[6];
  unsigned char essid[33];
  uint8_t essid_len;
  uint8_t chan;
  signed rssi;
  unsigned long last_seen;
};

#define MAX_APS 15

#endif // WIFI_SETTINGS_H
