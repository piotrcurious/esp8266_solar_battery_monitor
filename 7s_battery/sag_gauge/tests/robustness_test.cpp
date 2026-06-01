#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>

struct BatterySim {
    float rint = 0.070f;
    float capacity_ah = 20.0f;
    float current_soc = 0.95f;
    float current_i = 0.0;

    // Failure modes
    bool sensor_disconnected = false;
    float drift_mv = 0.0f;
    bool spike_active = false;

    float get_v_terminal() {
        if (sensor_disconnected) return 0.0f;
        float vc = (3.2f + (current_soc * 1.0f)) * 7.0f; // Simplified linear OCV for testing
        return vc - current_i * rint + (drift_mv / 1000.0f);
    }

    void step(float dt_s) {
        current_soc -= (current_i * dt_s) / (capacity_ah * 3600.0f);
        current_soc = std::clamp(current_soc, 0.0f, 1.0f);
    }
};

BatterySim sim;
std::default_random_engine generator;
std::normal_distribution<float> noise_dist(0.0, 5.0); // Noisier

extern uint32_t (*mock_adc_func)(int);

uint32_t my_mock_adc(int pin) {
    float noise = noise_dist(generator);
    if (sim.spike_active && (rand() % 10 == 0)) noise += 500.0f; // Huge spike

    if (pin == 34) {
        float v = sim.get_v_terminal();
        return (uint32_t)((v * 1000.0f / 12.0f) + noise);
    }
    if (pin == 35) {
        float sZeroMv = 1250.0f;
        float iA = sim.current_i;
        float sCurMv = iA * 185.0f / 2.0f + sZeroMv;
        return (uint32_t)(sCurMv + noise);
    }
    return 0;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

int main() {
    mock_adc_func = my_mock_adc;
    setup();

    printf("--- TEST: Normal Operation ---\n");
    sim.current_i = 10.0f;
    for(int i=0; i<100; i++) { loop(); _mock_millis_offset += 100; }
    printf("Result: V=%.2f I=%.1f R=%.1fmO SOC=%.1f%%\n", vPack, iA, rInt*1000.0f, socBlend);

    printf("\n--- TEST: Sensor Drift (+500mV) ---\n");
    sim.drift_mv = 500.0f;
    for(int i=0; i<500; i++) { loop(); _mock_millis_offset += 100; }
    printf("Result: V=%.2f (Expect ~+0.5V shift) R=%.1fmO\n", vPack, rInt*1000.0f);

    printf("\n--- TEST: High Noise & Spikes ---\n");
    sim.spike_active = true;
    float max_r = 0, min_r = 1.0;
    for(int i=0; i<500; i++) {
        loop(); _mock_millis_offset += 100;
        if(rInt > max_r) max_r = rInt;
        if(rInt < min_r) min_r = rInt;
    }
    printf("Result: Rint Range: %.1f - %.1f mOhm (Estimator stability check)\n", min_r*1000.0f, max_r*1000.0f);

    printf("\n--- TEST: Sensor Disconnection (0V) ---\n");
    sim.sensor_disconnected = true;
    for(int i=0; i<50; i++) { loop(); _mock_millis_offset += 100; }
    printf("Result: V=%.2f SOC=%.1f%% (Sanity check: clamped to 10V?)\n", vPack, socBlend);

    return 0;
}
