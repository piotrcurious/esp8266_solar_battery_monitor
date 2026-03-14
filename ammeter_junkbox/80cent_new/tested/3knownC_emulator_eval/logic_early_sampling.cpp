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
float fitted_voc, fitted_b;

#ifndef ARDUINO
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// SUGGESTED SAMPLING logic from fit_early_fix4sampling.cpp
int suggestedSamplingIntervalMs = 20;

bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false;

    constexpr float FALLBACK_TAU        = 0.01f;
    constexpr float MIN_TAU             = 1e-6f;
    constexpr float MAX_TAU             = 1.0f;
    constexpr float MIN_LR              = 1e-5f;
    constexpr float MAX_LR              = 0.1f;
    constexpr float MOMENTUM            = 0.9f;
    constexpr float MIN_IMPROVEMENT     = 1e-6f;

    fitted_voc = voltage_samples[NUM_SAMPLES - 1];
    fitted_b    = fitted_voc - voltage_samples[0];

    float t0 = time_samples[0] / 1000.0f;

    float sum_ln = 0.0f, sum_t = 0.0f;
    int valid_points = 0;
    for (int i = 1; i < NUM_SAMPLES / 2; i++) {
        float t_i = (time_samples[i] / 1000.0f) - t0;
        float v_diff = fitted_voc - voltage_samples[i];
        float v_init = fitted_voc - voltage_samples[0];
        if (v_diff > 0.001f && v_init > 0.001f) {
            float ln_ratio = logf(v_diff / v_init);
            if (fabsf(ln_ratio) > 1e-6f) {
                sum_ln += ln_ratio;
                sum_t  += t_i;
                valid_points++;
            }
        }
    }

    fitted_tau = (valid_points > 0 && fabsf(sum_ln) > 1e-6f) ? -sum_t / sum_ln : FALLBACK_TAU;
    fitted_tau = constrain(fitted_tau, MIN_TAU, MAX_TAU);

    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float prev_grad_voc = 0.0f, prev_grad_b = 0.0f, prev_grad_tau = 0.0f;
    float prev_error = INFINITY;
    int stagnant_iters = 0;

    for (int iter = 0; iter < RC_FIT_ITERATIONS; iter++) {
        float grad_voc = 0.0f, grad_b = 0.0f, grad_tau = 0.0f, error_sum = 0.0f;
        float inv_tau = 1.0f / max(fitted_tau, MIN_TAU);

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] / 1000.0f) - t0;
            float exp_term = expf(-t * inv_tau);
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = voltage_samples[i] - expected_v;
            error_sum += error * error;
            grad_voc += -2.0f * error;
            grad_b    +=  2.0f * error * exp_term;
            grad_tau += (fitted_b * t * exp_term * grad_b * inv_tau) * 2.0f;
        }

        float current_error = error_sum / NUM_SAMPLES;
        if (fabsf(prev_error - current_error) < MIN_IMPROVEMENT) {
            if (++stagnant_iters > 5) break;
        } else {
            stagnant_iters = 0;
        }

        grad_voc = MOMENTUM * prev_grad_voc + (1 - MOMENTUM) * grad_voc;
        grad_b    = MOMENTUM * prev_grad_b    + (1 - MOMENTUM) * grad_b;
        grad_tau = MOMENTUM * prev_grad_tau + (1 - MOMENTUM) * grad_tau;
        prev_grad_voc = grad_voc; prev_grad_b = grad_b; prev_grad_tau = grad_tau;

        float inv_samples = 1.0f / NUM_SAMPLES;
        fitted_voc -= adaptive_lr * grad_voc * inv_samples;
        fitted_b    -= adaptive_lr * grad_b * inv_samples;
        fitted_tau -= adaptive_lr * grad_tau * inv_samples;

        fitted_tau = constrain(fitted_tau, MIN_TAU, MAX_TAU);
        fitted_b    = max(fitted_b, 0.0f);
        fitted_voc = constrain(fitted_voc, voltage_samples[NUM_SAMPLES - 1] * 0.8f, voltage_samples[NUM_SAMPLES - 1] * 1.2f);
        adaptive_lr = constrain(adaptive_lr * (current_error > prev_error ? 0.5f : 1.1f), MIN_LR, MAX_LR);
        prev_error = current_error;
    }

    bool valid_params = (fitted_voc > 0.0f) && (fitted_tau > MIN_TAU) && (fitted_b >= 0.0f);
    if (valid_params) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        suggestedSamplingIntervalMs = constrain((int)(fitted_tau * 0.25 * 1000), 5, 50);
    }
    return valid_params;
}

void controller_setup() {
    Serial.begin(115200);
    PWM_Instance = new AVR_PWM(PWM_PIN, 14000, 0);
    PWMPeriod = 1000;
}

void controller_loop() {
    // Simplified loop just to test the fitting
    for (int i = 0; i < NUM_SAMPLES; i++) {
        time_samples[i] = millis();
        voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
        delay(SAMPLE_INTERVAL);
    }
    if (fit_rc_profile()) {
        open_circuit_voltage = fitted_voc;
        Serial.print("V_oc:"); Serial.print(open_circuit_voltage);
        Serial.print(",Tau:"); Serial.print(fitted_tau, 4);
        Serial.print(",R_tau:"); Serial.print(resistance_tau_est);
        Serial.print(",SuggInt:"); Serial.println(suggestedSamplingIntervalMs);
    }
    delay(5000);
}
