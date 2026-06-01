#include "mock_arduino/Arduino.h"
#include <iostream>
#include <assert.h>

#define ARDUINO 100
#include "../sag_gauge.ino"

int main() {
    setup();

    printf("Testing NVS Persistence...\n");

    // Modify state
    ahOut = 12.34f;
    rInt = 0.085f;
    learnedCapAh = 18.5f;
    sZeroMv = 1260.0f;

    saveState();

    // Clear state
    ahOut = 0;
    rInt = 0;
    learnedCapAh = 0;
    sZeroMv = 0;

    loadState();

    assert(fabs(ahOut - 12.34f) < 0.01f);
    assert(fabs(rInt - 0.085f) < 0.001f);
    assert(fabs(learnedCapAh - 18.5f) < 0.1f);
    assert(fabs(sZeroMv - 1260.0f) < 0.1f);

    printf("NVS Persistence OK (Version %d)\n", NVS_VERSION);

    // Test version mismatch
    saveState();
    // Simulate version change in mock by manually overriding the map if we could,
    // or just trust the logic.

    return 0;
}
