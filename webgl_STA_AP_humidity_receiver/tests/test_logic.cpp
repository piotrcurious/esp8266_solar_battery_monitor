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

// Mock Mutex
typedef int SemaphoreHandle_t;
#define pdTRUE 1
#define pdMS_TO_TICKS(x) x
int xSemaphoreTake(SemaphoreHandle_t m, int t) { return pdTRUE; }
void xSemaphoreGive(SemaphoreHandle_t m) {}
SemaphoreHandle_t xSemaphoreCreateMutex() { return 1; }

#include "../telemetry_frame.hpp"

#define NUMBER_OF_BUFFERS 5
#define MINUTES_GRAPH_BUFFER_MAX 1440

float* minutes_buffer[NUMBER_OF_BUFFERS];
int buffer_head = 0;
uint32_t global_total_packets = 0;
telemetry_frame last_frame;
bool new_packet_received = false;
SemaphoreHandle_t bufferMutex = 1;

void update_minute_tick() {
    buffer_head = (buffer_head + 1) % MINUTES_GRAPH_BUFFER_MAX;
    if (new_packet_received) {
        minutes_buffer[0][buffer_head] = last_frame.radio_active_time;
        minutes_buffer[1][buffer_head] = last_frame.voltage_ADC0;
        minutes_buffer[2][buffer_head] = last_frame.wifi_rssi;
        minutes_buffer[3][buffer_head] = last_frame.SH4x_rel_humidity;
        minutes_buffer[4][buffer_head] = last_frame.SH4x_temperature;
        new_packet_received = false;
    } else {
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
            if (minutes_buffer[j]) minutes_buffer[j][buffer_head] = NAN;
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

    std::cout << "Starting Circular Buffer Simulation..." << std::endl;

    // Simulate 2000 ticks to wrap around
    for (int i = 0; i < 2000; i++) {
        telemetry_frame frame;
        frame.voltage_ADC0 = (float)i;
        mock_udp.simulatePacket((const uint8_t*)&frame, sizeof(frame));
        update_minute_tick();
    }

    std::cout << "Head: " << buffer_head << std::endl;
    std::cout << "Last Voltage: " << minutes_buffer[1][buffer_head] << std::endl;

    // Verify circular wrap
    bool success = true;
    if (buffer_head != (2000 % MINUTES_GRAPH_BUFFER_MAX)) success = false;
    if (std::abs(minutes_buffer[1][buffer_head] - 1999.0f) > 0.001f) success = false;

    if (success) {
        std::cout << "Simulation SUCCESS" << std::endl;
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) delete[] minutes_buffer[j];
        return 0;
    } else {
        std::cout << "Simulation FAILED" << std::endl;
        for (int j = 0; j < NUMBER_OF_BUFFERS; j++) delete[] minutes_buffer[j];
        return 1;
    }
}
