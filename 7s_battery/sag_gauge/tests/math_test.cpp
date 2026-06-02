#include "mock_arduino/Arduino.h"
#include <iostream>
#include <assert.h>
#include <math.h>

#define ARDUINO 100
#include "../sag_gauge.ino"

void test_soc_from_v() {
    printf("Testing socFromV()...\n");
    assert(fabs(socFromV(4.20f) - 100.0f) < 0.1f);
    // New curve: 3.61V=20%, 3.63V=25% -> 3.62V=22.5%
    // 3.65V = 30.0%
    assert(fabs(socFromV(3.65f) - 30.0f) < 0.1f);
    assert(fabs(socFromV(3.20f) - 0.0f) < 0.1f);
    assert(fabs(socFromV(4.50f) - 100.0f) < 0.1f);
    assert(fabs(socFromV(3.00f) - 0.0f) < 0.1f);
    printf("socFromV() OK\n");
}

void test_kahan_summation() {
    printf("Testing Kahan Summation...\n");
    float sum = 0.0f;
    float c = 0.0f;
    auto kahanAdd = [](float &sum, float &c, float input) {
        float y = input - c; float t = sum + y;
        c = (t - sum) - y; sum = t;
    };

    // Test with small values that would normally cause precision loss
    for(int i=0; i<1000000; i++) {
        kahanAdd(sum, c, 1.2345e-6f);
    }
    // Simple float addition would lose significant precision here
    assert(fabs(sum - 1.2345f) < 0.001f);
    printf("Kahan Summation OK\n");
}

void test_color_blending() {
    printf("Testing color blending...\n");
    // Blue to Red
    uint32_t b = 0x0000FF;
    uint32_t r = 0xFF0000;
    uint32_t mixed = blendCol(b, r, 0.5f);
    // r, g, b components
    assert(((mixed >> 16) & 0xFF) > 120); // Red half
    assert((mixed & 0xFF) > 120);        // Blue half
    printf("Color blending OK\n");
}

void test_low_pass_filter() {
    printf("Testing low-pass filter...\n");
    float f = 0.0f;
    f = lp(f, 100.0f, 0.1f);
    assert(fabs(f - 10.0f) < 0.01f);
    for(int i=0; i<100; i++) f = lp(f, 100.0f, 0.1f);
    assert(fabs(f - 100.0f) < 1.0f);
    printf("Low-pass filter OK\n");
}

int main() {
    test_soc_from_v();
    test_kahan_summation();
    test_color_blending();
    test_low_pass_filter();
    printf("--- All Math Tests Passed ---\n");
    return 0;
}
