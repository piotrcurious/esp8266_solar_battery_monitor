#include "controller_logic.hpp"
#include <math.h>

#ifdef ARDUINO
#include "AVR_PWM.h"
#else
#include "mock_arduino.hpp"
#endif

// Unified and Optimized 3knownC Logic v4.5.2 (Advanced Hybrid)
// Features:
// 1. High-precision RC profile fitting (Gradient Descent) for baseline.
// 2. Real-time "Voltage-Binning" algebraic tracking during steady state.
// 3. Adaptive setpoint adjustment based on continuous Voc/Rint estimation.
// 4. Momentum-based GD with lambda-space fitting.
// 5. Fast-Recovery logic for source transients.

AVR_PWM* PWM_Instance;
float frequency;

// Pins & Ranges
#define PWM_PIN 9
#define SOURCE_PIN A0
#define CURRENT_PIN A1
#define SOURCE_VOLTAGE_RANGE 25.0
#define CURRENT_RANGE 10.0

// Constants
#define MAX_PWM 0.99f
#define MIN_PWM 0.0f
uint16_t PWMPeriod = 0;
#define FREQ_MAX 14000.0

#define VOLTAGE_FACTOR 0.80f
#define CALIBRATION_INTERVAL 30000
#define LEARNING_RATE_LOAD 0.0001f

#define DEBUG_SERIAL_OUTPUT_INTERVAL 100

// RC fitting parameters
#define NUM_SAMPLES 50
#define RC_FIT_ITERATIONS 100
#define RC_FIT_LEARNING_RATE 0.01f
#define MOMENTUM 0.85f
#define KNOWN_CAPACITANCE 0.020f

// Binning parameters
#define BIN_SAMPLES_REQUIRED 15
#define BIN_ALPHA 0.25f

// Variables
float open_circuit_voltage = 20.0f;
float internal_resistance_src = 5.0f;
float resistance_tau_est = 5.0f;
float load = 0.1f;
float fitted_tau = 0.1f;
float calibration_interval_current = CALIBRATION_INTERVAL;
int sampling_interval_ms = 10;

// Binning accumulators
float sum_v_low = 0, sum_i_low = 0;
int count_low = 0;
float sum_v_high = 0, sum_i_high = 0;
int count_high = 0;

static unsigned long last_calibration_time = 0;
static unsigned long last_debug_output = 0;
static bool is_calibrating = false;

static float voltage_samples[NUM_SAMPLES];
static unsigned long time_samples[NUM_SAMPLES];

#ifndef ARDUINO
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

bool fit_rc_profile(bool aggressive = false) {
    if (NUM_SAMPLES < 2) return false;

    float last_v = voltage_samples[NUM_SAMPLES - 1];
    float first_v = voltage_samples[0];
    float f_voc = last_v;
    float f_b   = f_voc - first_v;
    float t0 = time_samples[0] / 1000.0f;

    float f_lambda = 1.0f / fitted_tau;
    float lr = RC_FIT_LEARNING_RATE;
    float prev_err = 1e10;
    float m_voc = 0, m_b = 0, m_lambda = 0;

    for (int iter = 0; iter < RC_FIT_ITERATIONS; iter++) {
        float grad_voc = 0, grad_b = 0, grad_lambda = 0, err_sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] / 1000.0f) - t0;
            float e_term = expf(-t * f_lambda);
            float ev = f_voc - f_b * e_term;
            float err = voltage_samples[i] - ev;
            err_sum += err * err;
            grad_voc    += -2.0f * err;
            grad_b      += 2.0f * err * e_term;
            grad_lambda += -2.0f * err * f_b * t * e_term;
        }

        float g_norm = sqrtf(grad_voc*grad_voc + grad_b*grad_b + grad_lambda*grad_lambda);
        if (g_norm > 100.0f) {
            float scale = 100.0f / g_norm;
            grad_voc *= scale; grad_b *= scale; grad_lambda *= scale;
        }

        m_voc    = MOMENTUM * m_voc    + (1.0f - MOMENTUM) * grad_voc;
        m_b      = MOMENTUM * m_b      + (1.0f - MOMENTUM) * grad_b;
        m_lambda = MOMENTUM * m_lambda + (1.0f - MOMENTUM) * grad_lambda;

        f_voc    -= lr * m_voc / NUM_SAMPLES;
        f_b      -= lr * m_b / NUM_SAMPLES;
        f_lambda -= lr * m_lambda / NUM_SAMPLES;

        f_lambda = constrain(f_lambda, 0.1f, 1000.0f);
        if (err_sum > prev_err) lr *= 0.5f; else lr *= 1.05f;
        prev_err = err_sum;
        if (lr < 1e-8) break;
    }

    if (f_voc > 0.5) {
        float alpha = aggressive ? 0.9f : 0.5f;
        open_circuit_voltage = alpha * f_voc + (1.0f - alpha) * open_circuit_voltage;
        fitted_tau = 1.0f / f_lambda;
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        internal_resistance_src = alpha * resistance_tau_est + (1.0f - alpha) * internal_resistance_src;
        sampling_interval_ms = constrain((int)(3000.0f * fitted_tau / NUM_SAMPLES), 5, 100);
        return true;
    }
    return false;
}

void process_binning(float v, float i_src) {
    float target_v = open_circuit_voltage * VOLTAGE_FACTOR;
    // Collect samples in 2% bins around target
    if (v > target_v * 1.01f) {
        sum_v_high += v; sum_i_high += i_src; count_high++;
    } else if (v < target_v * 0.99f) {
        sum_v_low += v; sum_i_low += i_src; count_low++;
    }

    if (count_low >= BIN_SAMPLES_REQUIRED && count_high >= BIN_SAMPLES_REQUIRED) {
        float v_l = sum_v_low / count_low;
        float i_l = sum_i_low / count_low;
        float v_h = sum_v_high / count_high;
        float i_h = sum_i_high / count_high;
        float di = i_l - i_h;

        if (di > 0.01f) {
            float r_new = (v_h - v_l) / di;
            if (r_new > 0.1f && r_new < 50.0f) {
                internal_resistance_src = internal_resistance_src * (1.0f - BIN_ALPHA) + r_new * BIN_ALPHA;
                float voc_new = v_h + i_h * internal_resistance_src;
                if (voc_new > 5.0f && voc_new < 50.0f) {
                    open_circuit_voltage = open_circuit_voltage * (1.0f - BIN_ALPHA) + voc_new * BIN_ALPHA;
                }
            }
        }
        sum_v_low = sum_i_low = 0; count_low = 0;
        sum_v_high = sum_i_high = 0; count_high = 0;
    }
}

void calibrate(bool aggressive = false) {
    PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);
    delay(100);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        time_samples[i] = millis();
        voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1024.0);
        delay(sampling_interval_ms);
    }
    fit_rc_profile(aggressive);
    PWM_Instance->setPWM(PWM_PIN, frequency, load * PWMPeriod);
    last_calibration_time = millis();
    is_calibrating = false;
    // Reset bins after calibration to start fresh
    sum_v_low = sum_i_low = 0; count_low = 0;
    sum_v_high = sum_i_high = 0; count_high = 0;
}

void controller_setup() {
    frequency = FREQ_MAX;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);
    PWM_Instance->setPWM();
    PWMPeriod = PWM_Instance->getPWMPeriod();
    last_calibration_time = millis();
}

void controller_loop() {
    unsigned long now = millis();
    float s_volt = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1024.0);
    float i_src  = analogRead(CURRENT_PIN) * (CURRENT_RANGE / 1024.0);

    // v4.5.2: Aggressive Stuck-at-OC Detection
    static int stuck_high_count = 0;
    if (i_src < 0.05f && (open_circuit_voltage - s_volt) > 0.3f) {
        stuck_high_count++;
    } else {
        stuck_high_count = 0;
    }

    if (now - last_calibration_time > CALIBRATION_INTERVAL || stuck_high_count > 15) {
        is_calibrating = true;
        calibrate(stuck_high_count > 15);
        stuck_high_count = 0;
        return;
    }

    if (!is_calibrating) {
        process_binning(s_volt, i_src);

        // v4.5.2: Proactive Voc bleed when current is zero
        if (i_src < 0.02f) {
            if (s_volt < open_circuit_voltage) {
                open_circuit_voltage = open_circuit_voltage * 0.99f + s_volt * 0.01f;
            } else if (s_volt > open_circuit_voltage + 0.1f) {
                open_circuit_voltage = open_circuit_voltage * 0.99f + s_volt * 0.01f;
            }
        }

        float target_v = open_circuit_voltage * VOLTAGE_FACTOR;

        // v4.5.2: Dynamic Pulse Exploration
        static unsigned long last_pulse = 0;
        static int pulse_freq = 2000;
        bool in_pulse = false;

        // Increase pulse frequency if we are missing bin samples
        if (count_low < 5) pulse_freq = 1000; else pulse_freq = 3000;

        if (now - last_pulse > pulse_freq) {
            in_pulse = true;
            if (now - last_pulse > (pulse_freq + 100)) last_pulse = now;
        }

        static float dither_amp = 0.04f;
        static float dither_phase = 0;
        dither_phase += 0.25f;
        float dither = dither_amp * sinf(dither_phase);

        float err = target_v - s_volt;
        load = load - LEARNING_RATE_LOAD * err;
        load = constrain(load, 0, 1);

        float active_load = in_pulse ? 0.9f : constrain(load + dither, 0, 1);
        PWM_Instance->setPWM(PWM_PIN, frequency, (int)(active_load * PWMPeriod));
    }

    if (now - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        printf("DATA:%lu:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f\n",
               now, s_volt, i_src, load, open_circuit_voltage, internal_resistance_src, load);
        last_debug_output = now;
    }
}
