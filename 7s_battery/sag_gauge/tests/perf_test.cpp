#include "mock_arduino/Arduino.h"
#include <iostream>
#include <chrono>

#define ARDUINO 100
#include "../infer.ino"

int main() {
    setup();

    printf("--- Performance Audit ---\n");

    auto start = std::chrono::high_resolution_clock::now();
    const int iterations = 100; // Reduced iterations to avoid potential overhead issues

    for (int i=0; i<iterations; i++) {
        _mock_millis_offset += 100;
        loop();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    double avg_ms = (diff.count() * 1000.0) / iterations;
    printf("Average loop time: %.3f ms (Target < 20ms for ESP32)\n", avg_ms);

    if (avg_ms < 10.0) printf("Performance: EXCELLENT\n");
    else if (avg_ms < 50.0) printf("Performance: ACCEPTABLE\n");
    else printf("Performance: POOR - Optimization Required\n");

    printf("Audit Complete.\n");
    return 0;
}
