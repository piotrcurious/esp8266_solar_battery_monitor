#include "Arduino.h"
#include <chrono>
#include <thread>
#include <random>

MockSerial Serial;

unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool MockSerial::available() {
    return std::cin.peek() != EOF;
}

String MockSerial::readStringUntil(char terminator) {
    std::string line;
    if (std::getline(std::cin, line, terminator)) {
        return String(line);
    }
    return String("");
}

static std::mt19937 gen;

void randomSeed(unsigned long seed) {
    gen.seed(seed);
}

long random(long howbig) {
    if (howbig == 0) return 0;
    std::uniform_int_distribution<long> dis(0, howbig - 1);
    return dis(gen);
}

long random(long howsmall, long howbig) {
    if (howsmall >= howbig) return howsmall;
    std::uniform_int_distribution<long> dis(howsmall, howbig - 1);
    return dis(gen);
}

int analogRead(int pin) {
    return random(1024);
}
