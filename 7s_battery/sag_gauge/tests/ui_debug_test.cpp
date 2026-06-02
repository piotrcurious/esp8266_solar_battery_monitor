#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <string>

extern uint32_t (*mock_adc_func)(int);

uint32_t ui_debug_mock_adc(int pin) {
    return 1250;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

void reset_mock_state() {
    uiPage = 0;
    pState = PackState::IDLE;
    socBlend = 80.0f;
    vPack = 25.2f;
    vCellLoad = 3.6f;
    vCellRest = 3.65f;
    iA = 0.0f;
    pW = 0.0f;
    useImperial = false;
    isDimmed = false;
}

int main() {
    mock_adc_func = ui_debug_mock_adc;
    setup();

    printf("\n--- UI DEBUG: PAGE 0 NORMAL ---\n");
    reset_mock_state();
    renderPage0();

    printf("\n--- UI DEBUG: LOW BATTERY ALERT ---\n");
    reset_mock_state();
    socBlend = 10.0f;
    _mock_millis_offset = 0; // Flash phase
    renderPage0();

    printf("\n--- UI DEBUG: CRITICAL ALERT ---\n");
    reset_mock_state();
    vCellLoad = 3.0f;
    _mock_millis_offset = 250;
    renderPage0();

    printf("\n--- UI DEBUG: CHARGING ANIMATION ---\n");
    reset_mock_state();
    pState = PackState::CHARGING;
    _mock_millis_offset = 100;
    renderPage0();

    printf("\n--- UI DEBUG: PAGE 1 TRENDS ---\n");
    reset_mock_state();
    uiPage = 1;
    // Mock history
    for(int i=0; i<HIST_N; i++) { vHist[i] = 24.0f; iHist[i] = 2.0f; }
    renderPage1();

    printf("\n--- UI DEBUG: IMPERIAL UNITS ---\n");
    reset_mock_state();
    useImperial = true;
    renderPage0();

    printf("\n--- UI DEBUG: ENERGY DISTRIBUTION ---\n");
    reset_mock_state();
    whCruise = 10.0f; whActive = 50.0f; whBurst = 20.0f;
    whOut = 80.0f;
    renderPage3();

    printf("\n--- UI DEBUG COMPLETE ---\n");
    return 0;
}
