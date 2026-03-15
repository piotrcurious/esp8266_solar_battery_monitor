#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <stdint.h>

#define A0 0
#define A1 1
#define OUTPUT 1
#define INPUT 0

extern unsigned long _mock_millis;

unsigned long millis();
void delay(unsigned long ms);
int analogRead(int pin);
void pinMode(int pin, int mode);
void analogWrite(int pin, int value);

class MockSerial {
public:
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(double f, int p = 2) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void print(unsigned long l) { std::cout << l; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(double f, int p = 2) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void println(unsigned long l) { std::cout << l << std::endl; }
    void println() { std::cout << std::endl; }
    bool peek_exit();
};

extern MockSerial Serial;

// Mock AVR_PWM
class AVR_PWM {
public:
    AVR_PWM(int pin, float freq, float duty) {}
    void setPWM(int pin, float freq, float duty);
    void setPWM() {}
    uint16_t getPWMPeriod() { return 1000; }
};

#define F(s) s

#endif
