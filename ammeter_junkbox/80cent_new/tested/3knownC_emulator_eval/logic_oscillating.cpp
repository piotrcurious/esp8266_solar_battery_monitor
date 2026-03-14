#include "mock_arduino.hpp"
#include "controller_logic.hpp"
#include <math.h>

// Globals required by logic
AVR_PWM* PWM_Instance;
float frequency;
uint16_t PWMPeriod = 0;
float open_circuit_voltage = 0;
float internal_resistance_src = 1.0;
float resistance_tau_est = 0;
float load = 0;
float fitted_tau = 0.00001;
float calibration_interval_current = 20000;

#define PWM_PIN 9
#define SOURCE_PIN A0
#define LOAD_PIN A1
#define SOURCE_VOLTAGE_RANGE 28.0
#define LOAD_VOLTAGE_RANGE 30.0
#define KNOWN_CAPACITANCE 0.020
#define NUM_SAMPLES 50
#define SAMPLE_INTERVAL 20
#define RC_FIT_LEARNING_RATE 0.001
#define RC_FIT_ITERATIONS 50

float voltage_samples[NUM_SAMPLES];
unsigned long time_samples[NUM_SAMPLES];
float fitted_voc_param, fitted_b_param;

#ifndef ARDUINO
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// Oscillation verification parameters
#define OSCILLATION_PERCENTAGE 0.05
#define OSCILLATION_STEPS 3
#define OSCILLATION_DELAY 100

bool verify_voc_with_oscillation() {
    float test_pwm_values[OSCILLATION_STEPS];
    float measured_voltages[OSCILLATION_STEPS];
    float base_pwm = load * PWMPeriod;

    for (int i = 0; i < OSCILLATION_STEPS; i++) {
        float delta = (2.0 * (i % 2) - 1.0) * OSCILLATION_PERCENTAGE * base_pwm;
        test_pwm_values[i] = constrain(base_pwm + delta, 0, PWMPeriod);
    }

    for (int i = 0; i < OSCILLATION_STEPS; i++) {
        PWM_Instance->setPWM(PWM_PIN, 14000, test_pwm_values[i]);
        delay(OSCILLATION_DELAY);
        measured_voltages[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    }

    // Basic logic: if voltages are significantly different but expected to be same, or vice versa
    // In this mock, we just want to see if it triggers
    float diff = fabs(measured_voltages[1] - measured_voltages[0]);
    Serial.print("OscDiff:"); Serial.println(diff);

    return true;
}

void controller_setup() {
    Serial.begin(115200);
    PWM_Instance = new AVR_PWM(PWM_PIN, 14000, 0);
    PWMPeriod = 1000;
}

void controller_loop() {
    verify_voc_with_oscillation();
    delay(5000);
}
