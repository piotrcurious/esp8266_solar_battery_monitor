#include "mock_arduino.hpp"
#include <math.h>
#include <algorithm>

// Definitions to match oscillating.ino
#define PWM_PIN 9
#define SOURCE_PIN A0
#define SOURCE_VOLTAGE_RANGE 28.0

#define MAX_PWM 99.0
#define MIN_PWM 0.0
#define VOLTAGE_FACTOR 0.80
#define PARTIAL_MEASUREMENT_INTERVAL 20000
#define KNOWN_CAPACITANCE 0.020

#define NUM_SAMPLES 50
#define SAMPLE_INTERVAL 20
#define RC_FIT_ITERATIONS 50
#define RC_FIT_LEARNING_RATE 0.001

#define OSCILLATION_PERCENTAGE 0.05
#define OSCILLATION_STEPS 3
#define OSCILLATION_DELAY 10

// Globals - shared across both controller types in this mock
AVR_PWM* PWM_Instance;
uint16_t PWMPeriod;
float frequency;

static float last_fitted_voc = 0.0;
static float resistance_tau_est = 0;
static float pwm_value = 0; // In percentage 0-100
static unsigned long last_partial_measurement_time = 0;

static float voltage_samples[NUM_SAMPLES];
static unsigned long time_samples[NUM_SAMPLES];

#ifndef ARDUINO
#define max(a,b) std::max((float)(a),(float)(b))
#define min(a,b) std::min((float)(a),(float)(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

static bool fit_rc_profile_legacy() {
    float f_voc = voltage_samples[NUM_SAMPLES - 1];
    float f_b = f_voc - voltage_samples[0];
    float f_tau = 0.01;

    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
        float grad_voc = 0.0, grad_b = 0.0, grad_tau = 0.0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] - time_samples[0]) / 1000.0;
            float expected_v = f_voc - f_b * exp(-t / (f_tau + 0.000001));
            float err = voltage_samples[i] - expected_v;
            grad_voc += -2.0 * err;
            grad_b += 2.0 * err * exp(-t / (f_tau + 0.00001));
            grad_tau += 2.0 * err * f_b * (t / ((f_tau * f_tau)) * exp(-t / (f_tau + 0.00001)) + 0.00001);
        }
        f_voc -= RC_FIT_LEARNING_RATE * grad_voc / NUM_SAMPLES;
        f_b -= RC_FIT_LEARNING_RATE * grad_b / NUM_SAMPLES;
        f_tau -= RC_FIT_LEARNING_RATE * grad_tau / NUM_SAMPLES;
        f_tau = max(f_tau, 1e-6);
    }
    last_fitted_voc = f_voc;
    resistance_tau_est = f_tau / KNOWN_CAPACITANCE;
    return true;
}

void controller_setup() {
    frequency = 14000;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, 0);
    PWMPeriod = PWM_Instance->getPWMPeriod();
    PWM_Instance->setPWM(PWM_PIN, frequency, 0);
    delay(100);
    for(int i=0; i<NUM_SAMPLES; i++) {
        time_samples[i] = millis();
        voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
        delay(SAMPLE_INTERVAL);
    }
    fit_rc_profile_legacy();
    pwm_value = 10.0;
}

void controller_loop() {
    unsigned long now = millis();
    float v = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    if (v > last_fitted_voc * VOLTAGE_FACTOR) pwm_value += 0.1;
    else pwm_value -= 0.1;
    pwm_value = constrain(pwm_value, MIN_PWM, MAX_PWM);
    PWM_Instance->setPWM(PWM_PIN, frequency, (pwm_value/100.0) * PWMPeriod);

    if (now - last_partial_measurement_time > PARTIAL_MEASUREMENT_INTERVAL) {
        // Model failure: using step-response model during active tracking
        float d_pwm = OSCILLATION_PERCENTAGE * pwm_value;
        PWM_Instance->setPWM(PWM_PIN, frequency, ((pwm_value + d_pwm)/100.0) * PWMPeriod);
        delay(OSCILLATION_DELAY);
        float v_osc = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

        // This is the bug: using the wrong model to verify
        float t = OSCILLATION_DELAY / 1000.0;
        float v_pred = last_fitted_voc * (1.0 - exp(-t/0.05)); // Arbitrary bad model
        float mse = pow(v_osc - v_pred, 2);

        Serial.print("MSE:"); Serial.print(mse);
        Serial.println(" Oscillation Verification: Divergence detected!");

        last_partial_measurement_time = now;
    }
    delay(10);
    static unsigned long last_log = 0;
    if (now - last_log > 100) {
        Serial.print("V_oc:"); Serial.print(last_fitted_voc);
        Serial.print(",R_tau:"); Serial.print(resistance_tau_est);
        Serial.print(",load:"); Serial.println(pwm_value/100.0);
        last_log = now;
    }
}
