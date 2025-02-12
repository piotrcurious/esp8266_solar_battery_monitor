#include "AVR_PWM.h"
#include <math.h> 
#include <Arduino.h>

AVR_PWM* PWM_Instance;
float frequency;

// Define pins
#define PWM_PIN 9 
#define SOURCE_PIN A0 
#define SOURCE_VOLTAGE_RANGE 28.0 

// Constants
#define MAX_PWM 99.0 
#define MIN_PWM 0.0 
uint16_t PWMPeriod = 0;
#define FREQ_MAX 14000.0;

#define VOLTAGE_FACTOR 0.80
#define PARTIAL_MEASUREMENT_INTERVAL 20000 
#define KNOWN_CAPACITANCE 0.020 

// RC fitting parameters
#define NUM_SAMPLES 50
#define SAMPLE_INTERVAL 20 
#define RC_FIT_ITERATIONS 50
#define RC_FIT_LEARNING_RATE 0.001 

// Oscillation verification parameters
#define OSCILLATION_PERCENTAGE 0.05  
#define OSCILLATION_STEPS 3 
#define OSCILLATION_DELAY 10 

// Variables
float open_circuit_voltage = 0;
float fitted_voc = 0.00001;
float fitted_b = 0.00001;
float fitted_tau = 0.00001;
float last_fitted_voc = 0.0;
float last_fitted_b = 0.0;
float last_fitted_tau = 0.0;
float resistance_tau_est = 0;
float pwm_value = 0;
unsigned long last_partial_measurement_time = 0;

// Arrays for RC fitting
float voltage_samples[NUM_SAMPLES];
unsigned long time_samples[NUM_SAMPLES];

// Function to fit RC profile using gradient descent
bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false;
    
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];  
    fitted_b = fitted_voc - voltage_samples[0];  
    fitted_tau = 0.01;

    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
        float gradient_voc = 0.0, gradient_b = 0.0, gradient_tau = 0.0;
        float error_sum_squares = 0.0;

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] - time_samples[0]) / 1000.0;
            float measured_v = voltage_samples[i];
            float expected_v = fitted_voc - fitted_b * exp(-t / (fitted_tau + 0.000001));
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            gradient_voc += -2.0 * error;
            gradient_b += 2.0 * error * exp(-t / (fitted_tau + 0.00001));
            gradient_tau += 2.0 * error * fitted_b * (t / ((fitted_tau * fitted_tau)) * exp(-t / (fitted_tau + 0.00001)) + 0.00001);
        }

        fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / NUM_SAMPLES;
        fitted_b -= RC_FIT_LEARNING_RATE * gradient_b / NUM_SAMPLES;
        fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / NUM_SAMPLES;

        fitted_tau = max(fitted_tau, 1e-6);
        fitted_b = max(fitted_b, 0.0);
        fitted_voc = max(fitted_voc, 0.0);
    }

    if (fitted_voc > 0 && fitted_tau > 1e-6 && fitted_b >= 0) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        return true;
    } 
    return false;
}

// Function to verify Voc using small oscillations
bool verify_voc_with_oscillation() {
    float test_pwm_values[OSCILLATION_STEPS];
    float measured_voltages[OSCILLATION_STEPS];
    float base_pwm = pwm_value;
    
    for (int i = 0; i < OSCILLATION_STEPS; i++) {
        float delta = (2.0 * (i % 2) - 1.0) * OSCILLATION_PERCENTAGE * base_pwm;
        test_pwm_values[i] = constrain(base_pwm + delta, MIN_PWM, MAX_PWM);
    }

    // Apply oscillations and record responses
    for (int i = 0; i < OSCILLATION_STEPS; i++) {
        PWM_Instance->setPWM(PWM_PIN, frequency, test_pwm_values[i]);
        delay(OSCILLATION_DELAY);
        measured_voltages[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    }

    // Check if measured response fits expected RC behavior
    float error_sum = 0.0;
    for (int i = 1; i < OSCILLATION_STEPS; i++) {
        float t = i * OSCILLATION_DELAY / 1000.0;
        float expected_v = last_fitted_voc - last_fitted_b * exp(-t / (last_fitted_tau + 0.00001));
        float error = measured_voltages[i] - expected_v;
        error_sum += pow(error, 2);
    }

    float mse = error_sum / OSCILLATION_STEPS;

    if (mse > 0.3) {
        Serial.println(F("Oscillation Verification: Divergence detected!"));
        return false;
    } else {
        Serial.println(F("Oscillation Verification: Voc consistent!"));
        return true;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, MIN_PWM);
    frequency = FREQ_MAX;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);
    if (PWM_Instance) PWM_Instance->setPWM();
    PWMPeriod = PWM_Instance->getPWMPeriod();
}

void loop() {
    if (millis() - last_partial_measurement_time > PARTIAL_MEASUREMENT_INTERVAL) {
        verify_voc_with_oscillation();
        last_partial_measurement_time = millis();
    }
}
