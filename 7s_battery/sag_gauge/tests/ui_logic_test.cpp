#include "mock_arduino/Arduino.h"
#include <iostream>
#include <assert.h>

extern uint32_t (*mock_adc_func)(int);
extern MockSerial Serial;

uint32_t ui_logic_mock_adc(int pin) {
    return 1250;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

extern unsigned long _mock_millis_offset;

int main() {
    mock_adc_func = ui_logic_mock_adc;
    setup();

    printf("--- Testing Page Transitions ---\n");
    int initialPage = uiPage;
    _mock_millis_offset += PAGE_MS + 100;
    loop();
    printf("Page transition: %d -> %d\n", initialPage, uiPage);
    assert(uiPage == (initialPage + 1) % 5);

    printf("--- Testing Auto-Dimming ---\n");
    lastActMs = millis();
    pState = PackState::IDLE;

    _mock_millis_offset += CFG.dim_ms + 100;
    loop();
    printf("Brightness after dim period: %d\n", lcd.getBrightness());
    assert(lcd.getBrightness() == 20);
    assert(isDimmed == true);

    printf("--- Testing Auto-Off ---\n");
    _mock_millis_offset += CFG.dim_ms * 10;
    loop();
    printf("Brightness after off period: %d\n", lcd.getBrightness());
    assert(lcd.getBrightness() == 0);

    printf("--- Testing Wake on Current ---\n");
    iA = 5.0f; // Discharge
    pState = PackState::DISCHARGING;
    loop();
    printf("Brightness after load detected: %d\n", lcd.getBrightness());
    assert(lcd.getBrightness() == 200);
    assert(isDimmed == false);

    printf("UI Logic Verification OK\n");
    return 0;
}
