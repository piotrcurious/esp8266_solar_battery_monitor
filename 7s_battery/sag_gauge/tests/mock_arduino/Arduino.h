#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <stdarg.h>

#define ADC_11db 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define VSPI_HOST 0

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void Serial_begin(int baud);
void analogReadResolution(int res);
void analogSetPinAttenuation(int pin, int atten);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
uint32_t analogReadMilliVolts(int pin);

class MockSerial {
public:
    void begin(int baud) {}
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
    void print(const char* s) { printf("%s", s); }
    void print(float f, int p = 2) { printf("%.*f", p, f); }
    void print(int i) { printf("%d", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println() { printf("\n"); }
};

extern MockSerial Serial;
extern unsigned long _mock_millis_offset;
extern unsigned long _mock_micros_offset;

#endif
