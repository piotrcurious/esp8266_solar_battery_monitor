#include "mock_arduino/Arduino.h"
#include <iostream>
#include <assert.h>

extern uint32_t (*mock_adc_func)(int);
extern MockSerial Serial;

static uint32_t mock_cur_mv = 1250;
uint32_t ui_logic_mock_adc(int pin) {
    if (pin == 34) return (uint32_t)(25.2f * 1000.0f / 12.0f);
    if (pin == 35) return mock_cur_mv;
    return 0;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

extern unsigned long _mock_millis_offset;

int main() {
    mock_adc_func = ui_logic_mock_adc;
    setup();

    // Initial state check
    assert(uiPage == 0);
    assert(isDimmed == false);

    printf("--- Testing Page Transitions (Auto) ---\n");
    int initialPage = uiPage;
    _mock_millis_offset += PAGE_MS + 100;
    loop();
    printf("Page transition: %d -> %d\n", initialPage, uiPage);
    assert(uiPage == 1);

    printf("--- Testing Auto-Dimming ---\n");
    // Reset activity
    lastActMs = millis();
    pageMs = millis(); // Avoid auto page flip during this test
    pState = PackState::IDLE;
    isDimmed = false;
    lcd.setBrightness(200);

    _mock_millis_offset += CFG.dim_ms + 100;
    loop();
    printf("Brightness after dim period: %d, isDimmed=%d\n", lcd.getBrightness(), isDimmed);
    assert(isDimmed == true);
    assert(lcd.getBrightness() == 20);

    printf("--- Testing Auto-Off ---\n");
    _mock_millis_offset += CFG.dim_ms * 10;
    loop();
    printf("Brightness after off period: %d\n", lcd.getBrightness());
    assert(lcd.getBrightness() == 0);

    printf("--- Testing Wake on Current ---\n");
    // Ensure we are dimmed first
    pState = PackState::IDLE;
    isDimmed = true;
    lcd.setBrightness(0);
    lastActMs = millis() - (CFG.dim_ms * 11);
    setDigitalRead(PIN_BUTTON, HIGH);

    mock_cur_mv = 2000; // Large discharge
    _mock_millis_offset += UI_MS + 10;
    loop(); // updates measurements -> pState = DISCHARGING, THEN handles wake logic

    printf("Brightness after load detected: %d, pState=%d, iA=%.1f, isDimmed=%d\n", lcd.getBrightness(), (int)pState, iA, (int)isDimmed);
    assert(pState == PackState::DISCHARGING);

    if (isDimmed) {
        printf("DEBUG: still dimmed, loop() again\n");
        loop();
    }

    assert(isDimmed == false);
    assert(lcd.getBrightness() == 200);

    printf("--- Testing Manual Page Navigation (Button) ---\n");
    isDimmed = false;
    lcd.setBrightness(200);
    pageMs = millis(); // Reset auto-flip timer
    int pageBefore = uiPage;

    // Button Down
    setDigitalRead(PIN_BUTTON, LOW);
    _mock_millis_offset += 10;
    loop(); // detected down (btnDownMs = now)

    // Hold 200ms
    _mock_millis_offset += 200;
    loop(); // still down

    // Button Up
    setDigitalRead(PIN_BUTTON, HIGH);
    _mock_millis_offset += 10;
    loop(); // detected up (dur = now - btnDownMs)

    printf("Page after button: %d -> %d\n", pageBefore, uiPage);
    assert(uiPage == (pageBefore + 1) % 5);

    printf("UI Logic Verification PASSED\n");
    return 0;
}
