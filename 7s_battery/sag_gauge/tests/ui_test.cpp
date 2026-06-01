#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>

extern uint32_t (*mock_adc_func)(int);

uint32_t ui_mock_adc(int pin) {
    if (pin == 34) return (uint32_t)(22.0f * 1000.0f / 12.0f); // Low battery case
    if (pin == 35) return 1250; // Idle
    return 0;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

int main() {
    mock_adc_func = ui_mock_adc;
    setup();

    printf("Generating UI snapshots...\n");

    // 1. LOW BAT!
    _mock_millis_offset = 600; // Trigger flash phase 1
    socBlend = 10.0f;
    vCellLoad = 3.6f;
    canvas.fillScreen(0);
    renderPage0();
    canvas.savePPM("shot_low_bat.ppm");

    // 2. CRITICAL!
    vCellLoad = 2.9f;
    _mock_millis_offset = 250;
    canvas.fillScreen(0);
    renderPage0();
    canvas.savePPM("shot_critical.ppm");

    // 3. Normal / Predicted load
    vCellLoad = 3.8f;
    socBlend = 80.0f;
    vRested = 3.9f * 7.0f;
    rInt = 0.065f;
    canvas.fillScreen(0);
    renderPage0();
    canvas.savePPM("shot_normal.ppm");

    // 4. Trends
    for(int i=0; i<40; i++) {
        vHist[i] = 24.0f + i*0.1f;
        iHist[i] = 10.0f - i*0.2f;
    }
    canvas.fillScreen(0);
    renderPage1();
    canvas.savePPM("shot_trends.ppm");

    printf("UI Snapshots generated.\n");
    return 0;
}
