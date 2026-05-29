#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

// Battery state for simulation
struct BatterySim {
    float voc_100 = 4.2f * 7.0f;
    float voc_0 = 3.2f * 7.0f;
    float rint = 0.070f;
    float capacity_ah = 20.0f;
    float current_soc = 0.8f; // 0.0 to 1.0

    float current_i = 0.0; // Positive for discharge

    float get_v_terminal() {
        return voc_at_soc(current_soc) - current_i * rint;
    }

    float voc_at_soc(float soc) {
        return voc_0 + soc * (voc_100 - voc_0);
    }

    void step(float dt_s) {
        current_soc -= (current_i * dt_s) / (capacity_ah * 3600.0f);
        if (current_soc > 1.0f) current_soc = 1.0f;
        if (current_soc < 0.0f) current_soc = 0.0f;
    }
};

BatterySim sim;

// Mock ADC implementation
extern uint32_t (*mock_adc_func)(int);

uint32_t my_mock_adc(int pin) {
    if (pin == 34) { // PIN_BAT_VOLT
        float v_target = sim.get_v_terminal();
        return (uint32_t)(v_target * 1000.0f / 12.0f);
    }
    if (pin == 35) { // PIN_CUR_SENS
        float sZeroMv = 1250.0f; // 2.5V after 1/2 divider
        float iA = sim.current_i;
        float ACS712_MV_A = 185.0f;
        float CUR_DIV = 2.0f;
        float sCurMv = iA * ACS712_MV_A / CUR_DIV + sZeroMv;
        return (uint32_t)sCurMv;
    }
    return 0;
}

#define ARDUINO 100
#include "../infer.ino"

int main() {
    mock_adc_func = my_mock_adc;

    printf("--- Starting Simulation ---\n");
    setup();

    float dt = 0.1f; // 100ms steps
    for (int i = 0; i < 1000; ++i) {
        if (i == 100) {
            printf("--- Applying 10A Load ---\n");
            sim.current_i = 10.0f;
        }
        if (i == 400) {
            printf("--- Removing Load ---\n");
            sim.current_i = 0.0f;
        }
        if (i == 600) {
            printf("--- Charging at 5A ---\n");
            sim.current_i = -5.0f;
        }
        if (i == 900) {
            printf("--- Idle ---\n");
            sim.current_i = 0.0f;
        }

        sim.step(dt);
        loop();

        // Advance time for the next loop
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        _mock_millis_offset += 100;
        _mock_micros_offset += 100000;
    }

    printf("--- Simulation Finished ---\n");
    return 0;
}
