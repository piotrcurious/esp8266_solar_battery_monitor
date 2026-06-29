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
#include <vector>

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
void setDigitalRead(int pin, bool val);
uint32_t analogReadMilliVolts(int pin);

class String {
    std::string _s;
public:
    String(const char* s = "") : _s(s ? s : "") {}
    String(const std::string& s) : _s(s) {}
    void trim() {
        if (_s.empty()) return;
        size_t first = _s.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) {
            _s.clear();
            return;
        }
        _s.erase(0, first);
        size_t last = _s.find_last_not_of(" \t\n\r");
        if (last != std::string::npos) _s.erase(last + 1);
    }
    float toFloat() const {
        if (_s.empty()) return 0;
        try { return std::stof(_s); } catch(...) { return 0; }
    }
    bool startsWith(const char* s) const {
        if (!s) return false;
        return _s.compare(0, strlen(s), s) == 0;
    }
    String substring(int i) const {
        if (i < 0) i = 0;
        if (i >= (int)_s.size()) return String("");
        return String(_s.substr(i));
    }
    const char* c_str() const { return _s.c_str(); }
    bool operator==(const char* s) const { return _s == s; }
    bool operator==(const String& s) const { return _s == s._s; }
    size_t length() const { return _s.length(); }
};

class MockSerial {
public:
    std::string _input_buffer;
    void begin(int baud) {}
    bool available() { return !_input_buffer.empty(); }
    String readStringUntil(char c) {
        size_t pos = _input_buffer.find(c);
        if (pos == std::string::npos) {
            String s = _input_buffer;
            _input_buffer.clear();
            return s;
        } else {
            String s = _input_buffer.substr(0, pos);
            _input_buffer.erase(0, pos + 1);
            return s;
        }
    }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
    void print(const char* s) { printf("%s", s); }
    void print(const String& s) { printf("%s", s.c_str()); }
    void print(float f, int p = 2) { printf("%.*f", p, f); }
    void print(int i) { printf("%d", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println(const String& s) { printf("%s\n", s.c_str()); }
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
