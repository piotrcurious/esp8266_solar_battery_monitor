#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <string>
#include <random>

typedef uint8_t byte;

// String class mock
class String : public std::string {
public:
    String() : std::string("") {}
    String(const char* s) : std::string(s) {}
    String(std::string s) : std::string(s) {}
    String(float f, int p = 2) : std::string(std::to_string(f)) {}
    String(int i) : std::string(std::to_string(i)) {}

    int indexOf(char c) const {
        auto pos = find(c);
        return (pos == std::string::npos) ? -1 : (int)pos;
    }

    String substring(int start, int end = -1) const {
        if (end == -1) return String(substr(start));
        return String(substr(start, end - start));
    }

    float toFloat() const {
        try { return std::stof(*this); } catch (...) { return 0; }
    }

    int toInt() const {
        try { return std::stoi(*this); } catch (...) { return 0; }
    }

    int length() const { return (int)std::string::length(); }
};

namespace arduino {
    template <class T, class L>
    auto min(T a, L b) -> decltype(a < b ? a : b) {
        return (a < b) ? a : b;
    }

    template <class T, class L>
    auto max(T a, L b) -> decltype(a > b ? a : b) {
        return (a > b) ? a : b;
    }
}

using arduino::min;
using arduino::max;

class MockSerial {
public:
    void begin(int speed) {}
    bool available();
    String readStringUntil(char terminator);

    template<typename T>
    void print(T val) { std::cout << val; }
    template<typename T>
    void println(T val) { std::cout << val << std::endl; }
    void printf(const char *format, ...) {
        va_list args;
        va_start(args, format);
        vfprintf(stdout, format, args);
        va_end(args);
    }
};

extern MockSerial Serial;

unsigned long millis();
void delay(unsigned long ms);

long random(long howbig);
long random(long howsmall, long howbig);
void randomSeed(unsigned long seed);
int analogRead(int pin);

#ifndef isnan
#define isnan(x) std::isnan(x)
#endif

#define TFT_BLACK 0
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_YELLOW 0xFFE0

#endif
