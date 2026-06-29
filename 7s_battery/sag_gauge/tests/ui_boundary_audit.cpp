#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <string>

extern uint32_t (*mock_adc_func)(int);

uint32_t ui_audit_mock_adc(int pin) {
    return 1250;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

// Global to store violations
int clipping_violations = 0;
int zero_size_violations = 0;

// This will be called by our instrumented mock
extern "C" void audit_draw_call(const char* type, int x, int y, int w, int h) {
    if (x < 0 || y < 0 || x + w > 160 || y + h > 80) {
        printf("AUDIT:CLIPPING: %s at (%d,%d) size %dx%d\n", type, x, y, w, h);
        clipping_violations++;
    }
    if (w <= 0 || h <= 0) {
        // Text might have 0 height in some mocks, but usually 8*size.
        // We only care about fill/draw rects that are zero size.
        if (strstr(type, "Rect") != nullptr) {
            printf("AUDIT:ZERO_SIZE: %s at (%d,%d) size %dx%d\n", type, x, y, w, h);
            zero_size_violations++;
        }
    }
}

int main() {
    mock_adc_func = ui_audit_mock_adc;
    setup();

    printf("--- UI BOUNDARY AUDIT ---\n");

    for (int p=0; p<5; p++) {
        printf("Auditing Page %d...\n", p);
        uiPage = p;
        // Mock a few states
        socBlend = 50.0f; vCellRest = 3.7f; iA = 5.0f;
        canvas.fillScreen(0);
        if      (uiPage == 0) renderPage0();
        else if (uiPage == 1) renderPage1();
        else if (uiPage == 2) renderPage2();
        else if (uiPage == 3) renderPage3();
        else                  renderPage4();
    }

    if (clipping_violations > 0 || zero_size_violations > 0) {
        printf("FAILED: %d clipping, %d zero-size violations detected.\n",
               clipping_violations, zero_size_violations);
        return 1;
    } else {
        printf("PASSED: All drawing within 160x80 bounds and sane.\n");
        return 0;
    }
}
