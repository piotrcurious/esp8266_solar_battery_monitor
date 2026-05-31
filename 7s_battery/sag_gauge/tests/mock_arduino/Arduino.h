#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <stdarg.h>
#include <map>
#include <string>

#define ADC_11db 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
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
bool digitalRead(int pin);
uint32_t analogReadMilliVolts(int pin);

class String : public std::string {
public:
    String(const char* s = "") : std::string(s) {}
    String(std::string s) : std::string(s) {}
    void trim() {}
    float toFloat() { try { return std::stof(*this); } catch(...) { return 0; } }
    bool startsWith(const char* s) { return find(s) == 0; }
    String substring(int i) { return String(substr(i)); }
};

class MockSerial {
public:
    void begin(int baud) {}
    bool available() { return false; }
    String readStringUntil(char c) { return String(""); }
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

class MockESP {
public:
    void restart() { printf("[MOCK ESP] Restarting...\n"); }
};

extern MockESP ESP;
extern MockSerial Serial;
extern unsigned long _mock_millis_offset;
extern unsigned long _mock_micros_offset;

class Preferences {
    std::map<std::string, float> _data_f;
    std::map<std::string, uint32_t> _data_u;
public:
    void begin(const char* name, bool readonly) {}
    void end() {}
    void clear() { _data_f.clear(); _data_u.clear(); }
    void putFloat(const char* key, float val) { _data_f[key] = val; }
    float getFloat(const char* key, float def) {
        if (_data_f.find(key) == _data_f.end()) return def;
        return _data_f[key];
    }
    void putUInt(const char* key, uint32_t val) { _data_u[key] = val; }
    uint32_t getUInt(const char* key, uint32_t def) {
        if (_data_u.find(key) == _data_u.end()) return def;
        return _data_u[key];
    }
};

#endif
