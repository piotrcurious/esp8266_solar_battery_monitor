#include "controller_logic.hpp"
#include <math.h>

#ifdef ARDUINO
#include "AVR_PWM.h"
#else
#include "mock_arduino.hpp"
#endif

// Unified and Optimized 3knownC Logic v3 (Final Candidate)
// Features:
// 1. Blended Rint (Tau + Iterative)
// 2. Adaptive Sampling Interval based on Tau
// 3. Momentum-based Gradient Descent for RC fitting
// 4. Predictive Oscillation Verification for fast transient detection

AVR_PWM* PWM_Instance;
float frequency;

// Pins & Ranges
#define PWM_PIN 9
#define SOURCE_PIN A0
#define LOAD_PIN A1
#define SOURCE_VOLTAGE_RANGE 28.0
#define LOAD_VOLTAGE_RANGE 30.0

// Constants
#define MAX_PWM 99.0
#define MIN_PWM 0.0
uint16_t PWMPeriod = 0;
#define FREQ_MAX 14000.0

#define VOLTAGE_FACTOR 0.80
#define CALIBRATION_INTERVAL 25000
#define LEARNING_RATE_LOAD 0.00015

#define DEBUG_SERIAL_OUTPUT_INTERVAL 250
#define VERIFICATION_INTERVAL 4000

// RC fitting parameters
#define NUM_SAMPLES 50
#define RC_FIT_ITERATIONS 100
#define RC_FIT_LEARNING_RATE 0.008
#define MOMENTUM 0.85f

// Known Input Capacitance
#define KNOWN_CAPACITANCE 0.020

// Variables
float open_circuit_voltage = 0;
float internal_resistance_src = 1.0;
float resistance_tau_est = 0;
float load = 0;
float fitted_tau = 0.05;
float calibration_interval_current = CALIBRATION_INTERVAL;
int sampling_interval_ms = 20;

static unsigned long last_calibration_time = 0;
static unsigned long last_debug_output = 0;
static unsigned long last_verification_time = 0;
static bool is_calibrating = false;

static float voltage_samples[NUM_SAMPLES];
static unsigned long time_samples[NUM_SAMPLES];

#ifndef ARDUINO
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false;

    constexpr float MIN_TAU = 1e-6f;
    constexpr float MAX_TAU = 2.5f;

    float f_voc = voltage_samples[NUM_SAMPLES - 1];
    float f_b   = f_voc - voltage_samples[0];

    float t0 = time_samples[0] / 1000.0f;
    float sum_ln = 0, sum_t = 0;
    int valid = 0;
    for(int i=1; i<NUM_SAMPLES/2; i++) {
        float v_diff = f_voc - voltage_samples[i];
        float v_init = f_voc - voltage_samples[0];
        if (v_diff > 0.02 && v_init > 0.02) {
            sum_ln += logf(v_diff/v_init);
            sum_t += (time_samples[i]/1000.0f - t0);
            valid++;
        }
    }
    float f_tau = (valid > 5) ? -sum_t/sum_ln : fitted_tau;
    f_tau = constrain(f_tau, MIN_TAU, MAX_TAU);

    float lr = RC_FIT_LEARNING_RATE;
    float prev_err = 1e10;
    float m_voc = 0, m_b = 0, m_tau = 0;

    for (int iter = 0; iter < RC_FIT_ITERATIONS; iter++) {
        float grad_voc = 0, grad_b = 0, grad_tau = 0, err_sum = 0;
        float inv_tau = 1.0f / max(f_tau, MIN_TAU);

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] / 1000.0f) - t0;
            float e_term = expf(-t * inv_tau);
            float ev = f_voc - f_b * e_term;
            float err = voltage_samples[i] - ev;
            err_sum += err * err;

            grad_voc += -2.0f * err;
            grad_b   += 2.0f * err * e_term;
            grad_tau += (f_b * t * e_term * 2.0f * err * inv_tau * inv_tau);
        }

        m_voc = MOMENTUM * m_voc + (1.0f - MOMENTUM) * grad_voc;
        m_b   = MOMENTUM * m_b   + (1.0f - MOMENTUM) * grad_b;
        m_tau = MOMENTUM * m_tau + (1.0f - MOMENTUM) * grad_tau;

        f_voc -= lr * m_voc / NUM_SAMPLES;
        f_b   -= lr * m_b / NUM_SAMPLES;
        f_tau -= lr * m_tau / NUM_SAMPLES;

        f_tau = constrain(f_tau, MIN_TAU, MAX_TAU);
        f_b   = max(f_b, 0.0f);
        f_voc = max(f_voc, 0.0f);

        if (err_sum > prev_err) lr *= 0.5;
        else lr *= 1.05;
        prev_err = err_sum;
        if (lr < 1e-8) break;
    }

    if (f_voc > 0.5) {
        open_circuit_voltage = f_voc;
        fitted_tau = f_tau;
        resistance_tau_est = f_tau / KNOWN_CAPACITANCE;

        // Adaptive sampling: aim for 3-4 Tau total observation
        sampling_interval_ms = constrain((int)(3500.0f * f_tau / NUM_SAMPLES), 5, 120);
        return true;
    }
    return false;
}

bool verify_voc_with_oscillation() {
    float v0 = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    float d0 = load;

    // If we're not loaded much, Voc change is visible directly
    if (d0 < 0.05) {
        if (v0 < open_circuit_voltage * 0.95 || v0 > open_circuit_voltage + 0.5) return false;
        return true;
    }

    if (open_circuit_voltage < 1.0) return true;

    float d1 = d0 * 0.7f; // More aggressive poke
    PWM_Instance->setPWM(PWM_PIN, frequency, d1 * PWMPeriod);
    delay(60); // Longer delay for stabilization
    float v1 = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    PWM_Instance->setPWM(PWM_PIN, frequency, d0 * PWMPeriod);

    // Predictive check:
    // Under steady state: V = Voc * RL / (RL + Rsrc)
    // v1_pred = Voc / (1 + ((Voc-v0)/v0) * (d1/d0))
    float ratio = (open_circuit_voltage - v0) / max(v0, 0.1f);
    float v1_pred = open_circuit_voltage / (1.0f + ratio * (d1/d0));

    // Detection sensitivity
    if (v0 > open_circuit_voltage + 0.5) return false;
    if (v0 < open_circuit_voltage * 0.5) return false; // Immediate large drop
    if (fabsf(v1 - v1_pred) > 0.4) return false;      // Tighter tolerance

    return true;
}

void calibrate() {
    PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);
    delay(40);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        time_samples[i] = millis();
        voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
        delay(sampling_interval_ms);
    }
    if (fit_rc_profile()) {
        // Blended Rint update
        if (resistance_tau_est > 0.1) {
            internal_resistance_src = 0.75 * resistance_tau_est + 0.25 * internal_resistance_src;
        }
    }
    PWM_Instance->setPWM(PWM_PIN, frequency, load * PWMPeriod);
    last_calibration_time = millis();
    is_calibrating = false;
}

void controller_setup() {
    Serial.begin(115200);
    frequency = FREQ_MAX;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);
    PWM_Instance->setPWM();
    PWMPeriod = PWM_Instance->getPWMPeriod();
    // Trigger calibration almost immediately
    last_calibration_time = millis() - (CALIBRATION_INTERVAL - 1000);
}

void controller_loop() {
    unsigned long now = millis();
    float s_volt = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

    bool trigger_cal = false;
    if (now - last_verification_time > VERIFICATION_INTERVAL) {
        if (!verify_voc_with_oscillation()) {
            trigger_cal = true;
        }
        last_verification_time = now;
    }

    if (trigger_cal || (now - last_calibration_time > calibration_interval_current)) {
        is_calibrating = true;
        calibrate();
    }

    if (!is_calibrating) {
        float err = (VOLTAGE_FACTOR * open_circuit_voltage) - s_volt;
        load = load - LEARNING_RATE_LOAD * err;
        load = constrain(load, 0, 1);
        PWM_Instance->setPWM(PWM_PIN, frequency, load * PWMPeriod);
    }

    if (now - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        Serial.print(F("V_oc:")); Serial.print(open_circuit_voltage);
        Serial.print(F(",V_src:")); Serial.print(s_volt);
        Serial.print(F(",R_src:")); Serial.print(internal_resistance_src);
        Serial.print(F(",R_tau:")); Serial.print(resistance_tau_est);
        Serial.print(F(",load:")); Serial.print(load,3);
        Serial.print(F(",Tau:")); Serial.println(fitted_tau,4);
        last_debug_output = now;
    }
}
