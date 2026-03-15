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
    // In mock, we tell the emulator we delayed, or we just update our internal clock
    // and wait for the emulator to provide new data.
    // Actually, for the simulation loop, we'll handle clock incrementing in the wrapper.
    _mock_millis += ms;
    // We need to request new data from emulator after delay if we want it to be accurate.
    // For now, let's just increment.
    std::cout << "DELAY " << ms << std::endl;
}

int analogRead(int pin) {
    std::cout << "READ " << pin << std::endl;
    int val;
    if (!(std::cin >> val)) {
        exit(0);
    }
    return val;
}

void pinMode(int pin, int mode) {}

void analogWrite(int pin, int value) {
    // Not used because we use AVR_PWM
}

void AVR_PWM::setPWM(int pin, float freq, float duty) {
    _current_pwm_duty = duty / 1000.0;
    std::cout << "PWM " << _current_pwm_duty << std::endl;
}

bool MockSerial::peek_exit() {
    std::cout << "SYNC " << _mock_millis << std::endl;
    std::string resp;
    if (!(std::cin >> resp)) {
        exit(0);
    }
    if (resp == "EXIT") return true;
    return false;
}

MockSerial Serial;
