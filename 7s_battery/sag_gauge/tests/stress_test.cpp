#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>

// Battery state for simulation
struct BatterySim {
    float voc_100 = 4.2f * 7.0f;
    float voc_0 = 3.2f * 7.0f;
    float rint = 0.070f;
    float capacity_ah = 20.0f;
    float current_soc = 0.95f;

    float current_i = 0.0;
    float polarization_v = 0.0f;

    float get_v_terminal() {
        float r_factor = 1.0f;
        if (current_soc < 0.8f) r_factor = 1.0f + (0.8f - current_soc) * (0.5f / 0.7f);
        return voc_at_soc(current_soc) - current_i * (rint * r_factor) - polarization_v;
    }

    float voc_at_soc(float soc) {
        static constexpr struct { float v, s; } T[] = {
          {4.20f,100},{4.10f,95},{4.00f,88},{3.95f,84},{3.90f,78},
          {3.85f,70},{3.80f,62},{3.75f,54},{3.70f,45},{3.65f,36},
          {3.60f,28},{3.50f,16},{3.40f,8},{3.20f,0}
        };
        float vc = 3.20f;
        if (soc >= 1.0f) vc = 4.20f;
        else if (soc <= 0.0f) vc = 3.20f;
        else {
            for (int i=0; i<13; i++) {
                if (soc * 100.0f >= T[i+1].s) {
                    float t = (soc * 100.0f - T[i+1].s) / (T[i].s - T[i+1].s);
                    vc = T[i+1].v + t * (T[i].v - T[i+1].v);
                    break;
                }
            }
        }
        return vc * 7.0f;
    }

    void step(float dt_s) {
        current_soc -= (current_i * dt_s) / (capacity_ah * 3600.0f);
        if (current_soc > 1.0f) current_soc = 1.0f;
        if (current_soc < 0.0f) current_soc = 0.0f;

        float target_pol = current_i * 0.005f;
        polarization_v += (target_pol - polarization_v) * (dt_s / 60.0f);
    }
};

BatterySim sim;
std::default_random_engine generator;
std::normal_distribution<float> noise_dist(0.0, 3.0);

extern uint32_t (*mock_adc_func)(int);

uint32_t my_mock_adc(int pin) {
    float noise = noise_dist(generator);
    if (pin == 34) {
        float v_target = sim.get_v_terminal();
        return (uint32_t)((v_target * 1000.0f / 12.0f) + noise);
    }
    if (pin == 35) {
        float sZeroMv = 1250.0f;
        float iA = sim.current_i;
        float ACS712_MV_A = 185.0f;
        float CUR_DIV = 2.0f;
        float sCurMv = iA * ACS712_MV_A / CUR_DIV + sZeroMv;
        return (uint32_t)(sCurMv + noise * 1.5f);
    }
    return 0;
}

#define ARDUINO 100
#include "../infer.ino"

int main() {
    mock_adc_func = my_mock_adc;
    printf("--- Starting 24h Stress Test ---\n");
    setup();

    float dt = 0.1f;
    // 24 hours = 24 * 3600 / 0.1 = 864,000 steps
    // Let's do 100,000 steps for speed, covering several cycles
    for (long i = 0; i < 200000; ++i) {
        // Simple cyclical load
        if ((i / 5000) % 2 == 0) sim.current_i = 10.0f; // Discharge
        else sim.current_i = -5.0f; // Charge

        sim.step(dt);

        // Fast forward time
        _mock_millis_offset += 100;
        _mock_micros_offset += 100000;

        loop();

        if (i % 10000 == 0) {
            printf("Step %ld: V=%.2f I=%+.1fA SOC=%.1f%% Rint=%.1fmO ahOut=%.2f ahTotal=%.2f\n",
                   i, vPack, iA, socBlend, rInt*1000.0f, ahOut, ahTotal);
        }
    }

    printf("--- Stress Test Finished ---\n");
    printf("Final ahOut: %.2f (expected ~280)\n", ahOut);
    printf("Final ahTotal: %.2f (expected ~560)\n", ahTotal);
    printf("Final rInt: %.2f mOhm\n", rInt * 1000.0f);

    return 0;
}
