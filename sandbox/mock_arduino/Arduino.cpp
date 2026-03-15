#include "Arduino.h"
#include <chrono>
#include <thread>

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
