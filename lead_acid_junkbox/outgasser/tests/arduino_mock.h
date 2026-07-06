#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <map>
#include <algorithm>
#include <cstring>

#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 3
#define ADC_11db 0
#define AUTOMATIC 1
#define DIRECT 0

typedef uint32_t uint32_t;
typedef uint64_t uint64_t;

extern unsigned long _mock_millis;

inline unsigned long millis() { return _mock_millis; }
inline void delay(unsigned long ms) { _mock_millis += ms; }

struct SerialMock {
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(float f, int p = 2) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void print(unsigned long i) { std::cout << i; }
    void print(double d) { std::cout << d; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f, int p = 2) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void println(unsigned long i) { std::cout << i << std::endl; }
    void println(uint32_t i) { std::cout << i << std::endl; }
    void println(double d) { std::cout << d << std::endl; }
    void println() { std::cout << std::endl; }
    int available() { return 0; }
    char read() { return 0; }
};

extern SerialMock Serial;

struct WireMock {
    void begin() {}
};

extern WireMock Wire;

struct Adafruit_INA219 {
    float _v = 12.0;
    float _i_ma = 0.0;
    bool begin() { return true; }
    void setCalibration_32V1A() {}
    float getBusVoltage_V() { return _v; }
    float getCurrent_mA() { return _i_ma; }
};

struct Preferences {
    std::map<std::string, std::vector<uint8_t>> storage;
    void begin(const char* name, bool readOnly) {}
    void end() {}
    bool isKey(const char* key) { return storage.count(key) > 0; }
    size_t putBytes(const char* key, const void* value, size_t len) {
        std::vector<uint8_t> data((uint8_t*)value, (uint8_t*)value + len);
        storage[key] = data;
        return len;
    }
    size_t getBytes(const char* key, void* value, size_t len) {
        if (!isKey(key)) return 0;
        auto& data = storage[key];
        size_t n = std::min(len, data.size());
        memcpy(value, data.data(), n);
        return n;
    }
};

struct PID {
    double* input;
    double* output;
    double* setpoint;
    double kp, ki, kd;
    int mode;
    int direction;
    unsigned long sampleTime = 100;

    PID(double* in, double* out, double* set, double p, double i, double d, int dir)
        : input(in), output(out), setpoint(set), kp(p), ki(i), kd(d), direction(dir) {}

    void SetMode(int m) { mode = m; }
    void SetOutputLimits(double min, double max) {}
    void SetSampleTime(int t) { sampleTime = t; }
    void Compute() {
        double error = *setpoint - *input;
        if (direction == 0) *output += error * kp * 0.1;
        else *output -= error * kp * 0.1;
        if (*output < 0) *output = 0;
        if (*output > 255) *output = 255;
    }
};

inline void pinMode(int pin, int mode) {}
inline int digitalRead(int pin) { return HIGH; }
inline int analogRead(int pin) { return 2000; }
inline void analogSetAttenuation(int att) {}

inline void ledcSetup(int ch, int freq, int res) {}
inline void ledcAttachPin(int pin, int ch) {}

extern int _mock_duty_ch;
extern int _mock_duty_dis;
inline void ledcWrite(int ch, int duty) {
    if (ch == 0) _mock_duty_ch = duty;
    if (ch == 1) _mock_duty_dis = duty;
}

#define esp_sleep_enable_timer_wakeup(t)
#define esp_deep_sleep_start() exit(0)

template <typename T>
T constrain(T x, T a, T b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#endif
