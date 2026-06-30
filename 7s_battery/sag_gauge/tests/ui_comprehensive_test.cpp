#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <string>

extern uint32_t (*mock_adc_func)(int);

uint32_t ui_comp_mock_adc(int pin) {
    if (pin == 34) return (uint32_t)(25.2f * 1000.0f / 12.0f); // ~3.6V/c
    if (pin == 35) return 1250; // Idle
    return 0;
}

#define ARDUINO 100
#include "../sag_gauge.ino"

void set_state(int page, PackState state, float soc, float vcell, float current, bool imperial = false) {
    uiPage = page;
    pState = state;
    socBlend = soc;
    vPack = vcell * CFG.cells_s;
    vCellLoad = vcell;
    vCellRest = vcell + 0.05f; // Slight offset for rested
    vRested = vCellRest * CFG.cells_s;
    vSag = vRested - vPack;
    iA = current;
    pW = vPack * iA;
    // useImperial removed from firmware
}

int main() {
    mock_adc_func = ui_comp_mock_adc;
    setup();

    printf("--- Systemic UI State Audit ---\n");

    struct TestCase {
        std::string name;
        int page;
        PackState state;
        float soc;
        float vcell;
        float current;
        bool imperial;
    };

    std::vector<TestCase> cases = {
        {"P0_IDLE_METRIC", 0, PackState::IDLE, 80.0f, 3.85f, 0.0f, false},
        {"P0_DISCH_IMP", 0, PackState::DISCHARGING, 45.0f, 3.65f, 15.0f, true},
        {"P0_LOWBAT", 0, PackState::DISCHARGING, 12.0f, 3.45f, 5.0f, false},
        {"P0_CRITICAL", 0, PackState::DISCHARGING, 2.0f, 3.05f, 2.0f, false},
        {"P0_CHARGING", 0, PackState::CHARGING, 95.0f, 4.15f, -5.0f, false},

        {"P1_TRENDS", 1, PackState::DISCHARGING, 60.0f, 3.75f, 8.5f, false},
        {"P2_SUMMARY", 2, PackState::IDLE, 50.0f, 3.70f, 0.0f, false},
        {"P3_ENERGY", 3, PackState::DISCHARGING, 30.0f, 3.60f, 12.0f, false},
        {"P4_HEALTH", 4, PackState::IDLE, 90.0f, 4.00f, 0.0f, false},

        {"P0_MAX_VALUES", 0, PackState::DISCHARGING, 100.0f, 4.20f, 99.9f, false},
        {"P1_MAX_VALUES", 1, PackState::DISCHARGING, 100.0f, 4.20f, 99.9f, false},
        {"P2_MAX_VALUES", 2, PackState::IDLE, 100.0f, 4.20f, 0.0f, false},
        {"P3_MAX_VALUES", 3, PackState::DISCHARGING, 100.0f, 4.20f, 99.9f, false},
        {"P4_MAX_VALUES", 4, PackState::IDLE, 100.0f, 4.20f, 0.0f, false}
    };

    // Setup history for trends
    for(int i=0; i<HIST_N; i++) {
        vHist[i] = 25.0f - i*0.05f;
        iHist[i] = 5.0f + i*0.2f;
    }

    for (const auto& c : cases) {
        printf("Generating: %s\n", c.name.c_str());
        set_state(c.page, c.state, c.soc, c.vcell, c.current, c.imperial);

        // Handle flashing states by setting time
        if (c.name.find("LOWBAT") != std::string::npos || c.name.find("CRITICAL") != std::string::npos) {
            _mock_millis_offset = 600; // Flash ON
        }

        canvas.fillScreen(0);
        if      (uiPage == 0) renderPage0();
        else if (uiPage == 1) renderPage1();
        else if (uiPage == 2) renderPage2();
        else if (uiPage == 3) renderPage3();
        else                  renderPage4();

        std::string filename = "ui_" + c.name + ".ppm";
        canvas.savePPM(filename.c_str());
    }

    printf("UI Audit Complete. %zu states rendered.\n", cases.size());
    return 0;
}
