#include "Arduino.h"
#include <chrono>
#include <thread>
#include <stdarg.h>

static auto start_time = std::chrono::steady_clock::now();
unsigned long _mock_millis_offset = 0;
unsigned long _mock_micros_offset = 0;

unsigned long millis() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() + _mock_millis_offset;
}

unsigned long micros() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count() + _mock_micros_offset;
}

void delay(unsigned long ms) {
    // In simulation, we might want to just skip time or actually sleep.
    // For now, let's just increment the offset to simulate time passing if we were in a non-realtime loop.
    // But since we use steady_clock, real sleep is better for now unless we want a fast-forward sim.
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void analogReadResolution(int res) {}
void analogSetPinAttenuation(int pin, int atten) {}
void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int val) {}
bool digitalRead(int pin) { return true; } // Default HIGH (pullup)

// These will be provided by the test runner
uint32_t (*mock_adc_func)(int) = nullptr;

uint32_t analogReadMilliVolts(int pin) {
    if (mock_adc_func) return mock_adc_func(pin);
    return 0;
}

MockSerial Serial;
MockESP ESP;
