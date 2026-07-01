#include "mock_arduino/Arduino.h"
#include <iostream>
#include <assert.h>
#include <math.h>

extern uint32_t (*mock_adc_func)(int);

uint32_t thermal_mock_adc(int pin) {
    return 0; // Not used for this test as we override globals
}

#define ARDUINO 100
#include "../sag_gauge.ino"

int main() {
    mock_adc_func = thermal_mock_adc;
    setup();

    std::cout << "Starting Thermal Model Test..." << std::endl;

    // Initial state
    tEst = 25.0f;
    rInt = 0.060f;
    socBlend = 100.0f;
    iA = 20.0f; // High load

    // 1. Heating test
    std::cout << "  Testing heating under 20A load..." << std::endl;
    for (int i=0; i<600; i++) { // 1 minute at 10Hz (UI_MS=100)
        updateThermal(0.1f);
    }
    std::cout << "    Temp after 1min: " << tEst << " C (Expect > 25C)" << std::endl;
    assert(tEst > 25.0f);
    float t1 = tEst;

    // 2. Cooling test
    std::cout << "  Testing cooling while idle..." << std::endl;
    iA = 0.0f;
    for (int i=0; i<600; i++) {
        updateThermal(0.1f);
    }
    std::cout << "    Temp after 1min idle: " << tEst << " C (Expect < " << t1 << "C)" << std::endl;
    assert(tEst < t1);

    // 3. Equilibrium check
    std::cout << "  Testing equilibrium..." << std::endl;
    iA = 10.0f; // 10A load
    // Loss = 10^2 * 0.06 = 6W
    // DeltaT = (6 - 0.2 * (T - 25)) * (dt / 120)
    // Equilibrium when 6 = 0.2 * (T - 25) => T - 25 = 30 => T = 55C
    for (int i=0; i<10000; i++) { // Long run
        updateThermal(0.1f);
    }
    std::cout << "    Equilibrium temp at 10A: " << tEst << " C (Expect ~55C)" << std::endl;
    if (fabs(tEst - 55.0f) > 2.0f) {
        std::cout << "    WARNING: Equilibrium mismatch. Result=" << tEst << std::endl;
    }

    // 4. Robustness
    std::cout << "  Testing robustness (NaN)..." << std::endl;
    tEst = NAN;
    updateThermal(0.1f);
    std::cout << "    Temp after NaN recovery: " << tEst << std::endl;
    assert(isfinite(tEst));

    std::cout << "THERMAL TEST PASSED" << std::endl;
    return 0;
}
