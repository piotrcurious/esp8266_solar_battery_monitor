#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <cstdint>

// Mock classes/structs to simulate ESP32 environment
struct IPAddress {
    uint32_t addr;
    IPAddress(uint32_t a=0) : addr(a) {}
    std::string toString() const { return "192.168.4.1"; }
};

struct WiFiClass {
    void mode(int m) {}
    void begin(const char* s, const char* p) {}
    void softAP(const char* s, const char* p) {}
    IPAddress localIP() { return IPAddress(0x0100A8C0); }
    IPAddress softAPIP() { return IPAddress(0x0104A8C0); }
};

WiFiClass WiFi;

#define WIFI_AP_STA 3

struct AsyncUDPPacket {
    std::vector<uint8_t> _data;
    AsyncUDPPacket(const uint8_t* d, size_t l) : _data(d, d+l) {}
    size_t length() const { return _data.size(); }
    const uint8_t* data() const { return _data.data(); }
};

typedef void (*UDPCallback)(AsyncUDPPacket);

struct AsyncUDP {
    UDPCallback _cb;
    bool listenMulticast(IPAddress ip, uint16_t port) { return true; }
    void onPacket(UDPCallback cb) { _cb = cb; }
    void simulatePacket(const uint8_t* data, size_t len) {
        if(_cb) _cb(AsyncUDPPacket(data, len));
    }
};

#include "../telemetry_frame.hpp"

#define NUMBER_OF_BUFFERS 5
#define MINUTES_GRAPH_BUFFER_MAX 1440

float* minutes_buffer[NUMBER_OF_BUFFERS];
uint32_t global_total_packets = 0;
telemetry_frame last_frame;
bool new_packet_received = false;

void shift_buffers() {
    for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
        if (minutes_buffer[j]) {
            for(int i=0; i<MINUTES_GRAPH_BUFFER_MAX-1; i++) {
                minutes_buffer[j][i] = minutes_buffer[j][i+1];
            }
        }
    }
}

void update_minute_tick() {
    shift_buffers();
    if (new_packet_received) {
        minutes_buffer[0][MINUTES_GRAPH_BUFFER_MAX - 1] = last_frame.radio_active_time;
        minutes_buffer[1][MINUTES_GRAPH_BUFFER_MAX - 1] = last_frame.voltage_ADC0;
        minutes_buffer[2][MINUTES_GRAPH_BUFFER_MAX - 1] = last_frame.wifi_rssi;
        minutes_buffer[3][MINUTES_GRAPH_BUFFER_MAX - 1] = last_frame.SH4x_rel_humidity;
        minutes_buffer[4][MINUTES_GRAPH_BUFFER_MAX - 1] = last_frame.SH4x_temperature;
        new_packet_received = false;
    } else {
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
            if (minutes_buffer[j]) {
                minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX - 1] = NAN;
            }
        }
    }
}

int main() {
    // Initialize
    for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
        minutes_buffer[j] = new float[MINUTES_GRAPH_BUFFER_MAX];
        for (int i = 0; i < MINUTES_GRAPH_BUFFER_MAX; i++) {
            minutes_buffer[j][i] = NAN;
        }
    }

    AsyncUDP mock_udp;
    mock_udp.onPacket([](AsyncUDPPacket packet) {
        if (packet.length() == sizeof(telemetry_frame)) {
            memcpy(&last_frame, packet.data(), sizeof(telemetry_frame));
            new_packet_received = true;
            global_total_packets++;
        }
    });

    std::cout << "Starting Simulation..." << std::endl;

    // Simulate 10 packets and 10 minute ticks
    for (int i = 0; i < 10; i++) {
        telemetry_frame frame;
        frame.radio_active_time = 100.0f + i;
        frame.voltage_ADC0 = 12.5f + (i * 0.1f);
        frame.wifi_rssi = -60.0f - i;
        frame.SH4x_rel_humidity = 45.0f + i;
        frame.SH4x_temperature = 22.0f + (i * 0.2f);

        mock_udp.simulatePacket((const uint8_t*)&frame, sizeof(frame));
        update_minute_tick();

        std::cout << "Tick " << i << ": V=" << minutes_buffer[1][MINUTES_GRAPH_BUFFER_MAX-1]
                  << " Packets=" << global_total_packets << std::endl;
    }

    // Verify last buffer values
    bool success = true;
    if (global_total_packets != 10) success = false;
    if (std::abs(minutes_buffer[1][MINUTES_GRAPH_BUFFER_MAX-1] - 13.4f) > 0.001f) success = false;

    if (success) {
        std::cout << "Simulation SUCCESS" << std::endl;
        // Clean up
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) delete[] minutes_buffer[j];
        return 0;
    } else {
        std::cout << "Simulation FAILED" << std::endl;
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) delete[] minutes_buffer[j];
        return 1;
    }
}
