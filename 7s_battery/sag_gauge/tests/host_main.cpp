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
    float current_soc = 0.8f; // 0.0 to 1.0

    float current_i = 0.0; // Positive for discharge
    float polarization_v = 0.0f; // Simplified polarization voltage

    float get_v_terminal() {
        float r_factor = 1.0f;
        if (current_soc < 0.8f) r_factor = 1.0f + (0.8f - current_soc) * (0.5f / 0.7f);
        return voc_at_soc(current_soc) - current_i * (rint * r_factor) - polarization_v;
    }

    float voc_at_soc(float soc) {
        // Coarse Li-ion curve matching socFromV
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

        // Simple polarization model: follows current with some lag
        float target_pol = current_i * 0.005f;
        polarization_v += (target_pol - polarization_v) * (dt_s / 60.0f); // 60s time constant
    }
};

BatterySim sim;
std::default_random_engine generator;
std::normal_distribution<float> noise_dist(0.0, 5.0); // 5mV RMS noise

// Mock ADC implementation
extern uint32_t (*mock_adc_func)(int);

float driftyZeroMv = 1250.0f;

uint32_t my_mock_adc(int pin) {
    float noise = noise_dist(generator);

    // Add a slight bias to current noise to test deadband
    float cur_noise_bias = 2.0f; // ~0.1A bias

    // Occasional massive spike (outlier)
    if (rand() % 100 == 0) noise += (rand() % 2 == 0 ? 500.0f : -500.0f);

    if (pin == 34) { // PIN_BAT_VOLT
        float v_target = sim.get_v_terminal();
        return (uint32_t)((v_target * 1000.0f / 12.0f) + noise);
    }
    if (pin == 35) { // PIN_CUR_SENS
        float sZeroMv = driftyZeroMv; // 2.5V after 1/2 divider
        float iA = sim.current_i;
        float ACS712_MV_A = 185.0f;
        float CUR_DIV = 2.0f;
        float sCurMv = iA * ACS712_MV_A / CUR_DIV + sZeroMv;
        return (uint32_t)(sCurMv + noise * 2.0f + cur_noise_bias);
    }
    return 0;
}

#define ARDUINO 100
#include "../infer.ino"

int main() {
    mock_adc_func = my_mock_adc;

    printf("--- Starting Simulation ---\n");
    // sim.rint = 0.140f; // test degraded battery
    setup();

    float dt = 0.1f; // 100ms steps
    for (int i = 0; i < 5000; ++i) {
        // Drift the zero point slightly over time
        driftyZeroMv += 0.005f;

        if (i == 100) {
            printf("--- Applying 10A Load ---\n");
            sim.current_i = 10.0f;
        }
        if (i == 300) {
            printf("--- Oscillating Load (5-15A) ---\n");
        }
        if (i >= 300 && i < 600) {
            sim.current_i = 10.0f + 5.0f * sin(i * 0.1f);
        }
        if (i == 600) {
            printf("--- Removing Load ---\n");
            sim.current_i = 0.0f;
        }
        if (i == 1000) {
            printf("--- Charging at 5A ---\n");
            sim.current_i = -5.0f;
        }
        if (i == 1500) {
            printf("--- Idle ---\n");
            sim.current_i = 0.0f;
        }
        if (i == 2500) {
            printf("--- Removing Zero Drift ---\n");
        }
        if (i == 3500) {
            printf("--- Long Idle with Noise ---\n");
            sim.current_i = 0.0f;
        }

        sim.step(dt);
        loop();

        // Analytics against ground truth
        static float rint_mse_sum = 0;
        static int rint_samples = 0;
        if (fabsf(sim.current_i) > 1.0f) {
            float err = (rInt * 1000.0f) - (sim.rint * 1000.0f);
            rint_mse_sum += err * err;
            rint_samples++;
        }

        if (i == 4999) {
            printf("\n--- Final Report ---\n");
            printf("Rint MSE (during load): %.2f\n", rint_samples > 0 ? rint_mse_sum / rint_samples : 0);
            printf("Final estimated Rint: %.1f mOhm (True: %.1f)\n", rInt * 1000.0f, sim.rint * 1000.0f);
            printf("Final SOC error: %.2f%%\n", socBlend - sim.current_soc * 100.0f);
        }

        // Advance time for the next loop
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        _mock_millis_offset += 100;
        _mock_micros_offset += 100000;
    }

    printf("--- Simulation Finished ---\n");
    return 0;
}
