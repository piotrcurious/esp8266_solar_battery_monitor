/*
 * SLP4_humidity_3.ino
 * Advanced Smart Low Power (SLP) sender with deep sleep, beacon poking, and RTC state persistence.
 *
 * Version: 3.0
 * Commit History:
 * - 3.0: Integrated RTC persistence for WiFi and SensorFilter states, implemented AP peek, transitioned to deep sleep.
 */

#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "SensorFilter.h"
#include <user_interface.h>
#include <Adafruit_SHT4x.h>

// WiFi Globals (declared as extern in wifi_settings.h)
const char* ssid = "voltage2";
const char* password = "irrolling12";
const uint8_t channel = 1;
WiFiUDP udp;
const int port = 5683;
IPAddress broadcast = IPAddress(224, 0, 1, 187);

// Global State
uint8_t sleep_periods_elapsed = 0;
uint32_t total_packets = 0;
bool transmit_phase = false;
volatile bool is_beacon_fresh = false;
volatile uint32_t last_beacon_timestamp = 0;
bool initialConnectionEstablished = false;

// RTC Store
rtc_store_t rtc_data;
bool rtc_valid = false;

// Sensors
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

const bool CONTINOUS_MODE = false;
const bool idle_sensor_update = true;

#define ITERATIONS_PER_WAKEUP 16
#define ITERATIONS_PER_WAKEUP_RSSI 32
#define ITERATIONS_PER_WAKEUP_SHT4x 8

// Filters
SensorFilter Filter_voltage_ADC0(16, 0.1, 0.1, 0.1, 0.3);
SensorFilter Filter_wifi_rssi(128, 2.0, 0.01, 0.8, 0.001);
SensorFilter Filter_aTemperature(32, 0.1, 0.01, 0.01, 0.001);
SensorFilter Filter_aHumidity(32, 0.2, 0.01, 0.01, 0.001);

telemetry_frame tframe;

// --- RTC Helpers ---
void save_rtc() {
    rtc_data.magic = RTC_MAGIC;
    rtc_data.sleep_periods_elapsed = sleep_periods_elapsed;
    rtc_data.total_packets = total_packets;

    if (WiFi.status() == WL_CONNECTED) {
        rtc_data.ip_address = (uint32_t)WiFi.localIP();
        rtc_data.ip_gateway = (uint32_t)WiFi.gatewayIP();
        rtc_data.ip_mask    = (uint32_t)WiFi.subnetMask();
        rtc_data.ip_dns1    = (uint32_t)WiFi.dnsIP(0);
        rtc_data.ip_dns2    = (uint32_t)WiFi.dnsIP(1);
        memcpy(rtc_data.wifi_bssid, WiFi.BSSID(), 6);
        rtc_data.wifi_channel = WiFi.channel();
        strncpy(rtc_data.wifi_ssid, WiFi.SSID().c_str(), sizeof(rtc_data.wifi_ssid)-1);
        strncpy(rtc_data.wifi_auth, WiFi.psk().c_str(), sizeof(rtc_data.wifi_auth)-1);
    } else if (!rtc_valid) {
        // First run or invalid RTC, set defaults
        rtc_data.ip_address = 0;
        rtc_data.wifi_channel = channel;
        strncpy(rtc_data.wifi_ssid, ssid, sizeof(rtc_data.wifi_ssid)-1);
        strncpy(rtc_data.wifi_auth, password, sizeof(rtc_data.wifi_auth)-1);
        memset(rtc_data.wifi_bssid, 0, 6);
    }

    Filter_voltage_ADC0.getState(rtc_data.filter_voltage.estimate, rtc_data.filter_voltage.errorCovariance);
    Filter_wifi_rssi.getState(rtc_data.filter_rssi.estimate, rtc_data.filter_rssi.errorCovariance);
    Filter_aTemperature.getState(rtc_data.filter_temp.estimate, rtc_data.filter_temp.errorCovariance);
    Filter_aHumidity.getState(rtc_data.filter_humidity.estimate, rtc_data.filter_humidity.errorCovariance);

    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc_data, sizeof(rtc_data));
}

bool load_rtc() {
    if (ESP.rtcUserMemoryRead(0, (uint32_t*)&rtc_data, sizeof(rtc_data))) {
        if (rtc_data.magic == RTC_MAGIC) {
            sleep_periods_elapsed = rtc_data.sleep_periods_elapsed;
            total_packets = rtc_data.total_packets;

            Filter_voltage_ADC0.setState(rtc_data.filter_voltage.estimate, rtc_data.filter_voltage.errorCovariance);
            Filter_wifi_rssi.setState(rtc_data.filter_rssi.estimate, rtc_data.filter_rssi.errorCovariance);
            Filter_aTemperature.setState(rtc_data.filter_temp.estimate, rtc_data.filter_temp.errorCovariance);
            Filter_aHumidity.setState(rtc_data.filter_humidity.estimate, rtc_data.filter_humidity.errorCovariance);

            return true;
        }
    }
    return false;
}

// --- Utilities ---
void yield_delay(uint32_t delay_ms) {
    uint32_t start = millis();
    while (millis() - start < delay_ms) {
        yield();
    }
}

// --- Scanner ---
void promisc_cb(uint8_t *buf, uint16_t len) {
    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
    if (len >= 60) { // minimum length for a beacon/probe response enough to get BSSID
        if (memcmp(rtc_data.wifi_bssid, bbuf->bssid, 6) == 0) {
            is_beacon_fresh = true;
            last_beacon_timestamp = millis();
        }
    }
}

bool peek_for_ap(uint32_t timeout_ms) {
    wifi_set_opmode(STATION_MODE);
    wifi_set_channel(rtc_data.wifi_channel);

    is_beacon_fresh = false;
    wifi_set_promiscuous_rx_cb(promisc_cb);
    wifi_promiscuous_enable(1);

    uint32_t start = millis();
    while (!is_beacon_fresh && (millis() - start < timeout_ms)) {
        yield();
    }
    wifi_promiscuous_enable(0);
    return is_beacon_fresh;
}

// --- Connection ---
bool connect_wifi() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);

    if (rtc_valid && rtc_data.ip_address != 0) {
        WiFi.config(IPAddress(rtc_data.ip_address), IPAddress(rtc_data.ip_gateway), IPAddress(rtc_data.ip_mask), IPAddress(rtc_data.ip_dns1), IPAddress(rtc_data.ip_dns2));
        WiFi.begin(rtc_data.wifi_ssid, rtc_data.wifi_auth, rtc_data.wifi_channel, rtc_data.wifi_bssid, true);
    } else if (rtc_valid && rtc_data.wifi_ssid[0] != 0) {
        WiFi.begin(rtc_data.wifi_ssid, rtc_data.wifi_auth, rtc_data.wifi_channel, rtc_data.wifi_bssid, true);
    } else {
        WiFi.begin(ssid, password, channel);
    }

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start < ASSOCIATE_TIMEOUT_INTERVAL)) {
        digitalWrite(LED_PIN, LOW);
        delay(10);
        digitalWrite(LED_PIN, HIGH);
        yield_delay(90);
    }

    if (WiFi.status() == WL_CONNECTED) {
        initialConnectionEstablished = true;
        return true;
    }
    return false;
}

void send_packet() {
    if (WiFi.status() == WL_CONNECTED) {
        total_packets++;
        if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
            udp.write((uint8_t*)&tframe, sizeof(tframe));
            udp.endPacket();
            yield_delay(20);
        }
    }
}

void fpm_wakup_cb_func(void) {
  // Serial.println(F("woke up"));
  // Serial.flush();
  wifi_fpm_close();
}

// --- Core ---
void setup() {
    Serial.begin(115200);
    Serial.println("\nSLP4_humidity_3 Starting...");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(MODE_PIN, INPUT);

    rtc_valid = load_rtc();

    if (!sht4.begin()) {
        Serial.println("SHT4x not found!");
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);

    bool should_transmit = (sleep_periods_elapsed >= SLEEP_PERIODS_TO_SEND_PACKET);

    wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func);

    if (should_transmit) {
        Serial.println("Peek for AP...");
        // If we don't have a BSSID yet, we can't peek, so just try to connect
        if (rtc_data.wifi_bssid[0] == 0 && rtc_data.wifi_bssid[1] == 0) {
            Serial.println("No BSSID known, attempting full connect...");
            if (connect_wifi()) {
                transmit_phase = true;
                udp.begin(port);
            }
        } else if (peek_for_ap(DEEP_SLEEP_LISTEN_MS)) {
            Serial.println("AP found, connecting...");
            uint32_t connect_start = millis();
            if (connect_wifi()) {
                transmit_phase = true;
                tframe.radio_active_time = millis() - connect_start;
                Serial.printf("Connected in %d ms\n", (int)tframe.radio_active_time);
                udp.begin(port);
            }
        } else {
            Serial.println("AP not found in peek.");
        }
    }
}

void loop() {
    // Sensor Update
    if (transmit_phase || idle_sensor_update) {
        // ADC0
        for (int i = 0; i < ITERATIONS_PER_WAKEUP; i++) {
            float val = analogRead(analogInPin) * (17.0 / 1024.0);
            tframe.voltage_ADC0 = Filter_voltage_ADC0.updateSensorReading(val);
        }

        // SHT4x
        sensors_event_t humidity, temp;
        for (int i = 0; i < ITERATIONS_PER_WAKEUP_SHT4x; i++) {
            if (sht4.getEvent(&humidity, &temp)) {
                tframe.SH4x_rel_humidity = Filter_aHumidity.updateSensorReading(humidity.relative_humidity);
                tframe.SH4x_temperature = Filter_aTemperature.updateSensorReading(temp.temperature);
            }
        }

        // RSSI
        if (transmit_phase && WiFi.status() == WL_CONNECTED) {
            for (int i = 0; i < ITERATIONS_PER_WAKEUP_RSSI; i++) {
                tframe.wifi_rssi = Filter_wifi_rssi.updateSensorReading(WiFi.RSSI());
            }
        }
    }

    if (transmit_phase) {
        send_packet();
        sleep_periods_elapsed = 0;
    } else {
        sleep_periods_elapsed++;
    }

    save_rtc();

    if (!CONTINOUS_MODE && digitalRead(MODE_PIN) == HIGH) {
        Serial.printf("Sleeping. Periods: %d/%d\n", sleep_periods_elapsed, SLEEP_PERIODS_TO_SEND_PACKET);
        Serial.flush();

        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
        delay(1);

        // Deep sleep for ESP8266
        ESP.deepSleep(SLEEP_DURATION_MS * 1000ULL, WAKE_RF_DISABLED);
        delay(100);
    } else {
        // Continuous mode
        transmit_phase = true;
        yield_delay(500);
        if (WiFi.status() != WL_CONNECTED) {
            if (connect_wifi()) {
                udp.begin(port);
            }
        }
    }
}
