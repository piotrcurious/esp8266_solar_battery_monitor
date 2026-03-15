#include "mock_arduino.hpp"
#include <map>
#include <string>

unsigned long _mock_millis = 0;
std::map<int, int> _mock_analog_inputs;
float _current_pwm_duty = 0;

unsigned long millis() {
    return _mock_millis;
}

void delay(unsigned long ms) {
    _mock_millis += ms;
    std::cout << "DELAY " << ms << std::endl;
    std::cout.flush();
}

int analogRead(int pin) {
    std::cout << "READ " << pin << std::endl;
    std::cout.flush();
    int val;
    if (!(std::cin >> val)) {
        exit(0);
    }
    return val;
}

void pinMode(int pin, int mode) {}

void analogWrite(int pin, int value) {
    _current_pwm_duty = (float)value / 255.0f;
    std::cout << "PWM " << _current_pwm_duty << std::endl;
    std::cout.flush();
}

void AVR_PWM::setPWM(int pin, float freq, float duty) {
    _current_pwm_duty = duty / 1000.0;
    std::cout << "PWM " << _current_pwm_duty << std::endl;
    std::cout.flush();
}

bool MockSerial::peek_exit() {
    return false; // Not used in this host
}

MockSerial Serial;
