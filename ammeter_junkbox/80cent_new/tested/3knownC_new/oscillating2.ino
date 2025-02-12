#include "AVR_PWM.h"

#include <math.h> // Required for exp(), pow()

#include <Arduino.h>
AVR_PWM* PWM_Instance;
float frequency;
#define FITTING_DEBUG // uncomment to get messages from calibration fitters
// Define the pins

#define PWM_PIN 9 // The pin for the PWM output to the load

#define LOAD_PIN A1 // The pin for measuring the load voltage - not directly used in core logic anymore

#define LOAD_VOLTAGE_RANGE 30.0 // the max value of load analog input maps to this voltage, in Volts - not directly used in core logic anymore

#define SOURCE_PIN A0 // The pin for measuring the source voltage

#define SOURCE_VOLTAGE_RANGE 28.0 // the max value of source analog input maps to this voltage, in Volts
// Define the constants

#define MAX_PWM 99.0 // The maximum PWM duty cycle

#define MIN_PWM 0.0 // The minimum PWM duty cycle

uint16_t PWMPeriod = 0 ;

#define FREQ_MAX 14000.0;
#define LOAD_FACTOR 0.001 // The factor for increasing or decreasing the load - not directly used, control by error signal

#define CALIBRATION_FACTOR 0.0001 // The factor for increasing or decreasing the load during RINT calibration - not used anymore

#define VOLTAGE_FACTOR 0.80 // The factor for keeping the output voltage within 80% of the open-circuit voltage

#define CALIBRATION_INTERVAL 20000 // Initial interval for full calibration in milliseconds

#define OPEN_CIRCUIT_MEASUREMENT_TIME 50 // Reduced time for initial measurement - not directly used

#define CALIBRATION_DURATION 3000 // The duration for calibrating the internal resistance in milliseconds - not used anymore

#define RINT_CALIBRATION_MEASUREMENT_TIME 100 // time to wait after changing pwm value before measuring voltage across load - not used anymore

#define LEARNING_RATE 0.001 // The learning rate for the gradient descent algorithm - not directly used

#define LEARNING_RATE_LOAD 0.0001 // The learning rate for the gradient descent algorithm - load adjustment
#define DEBUG_SERIAL_OUTPUT_INTERVAL 200 // interval for debug output for visualization

#define PARTIAL_MEASUREMENT_INTERVAL 20000 // Interval for partial VOC measurement
// RC fitting parameters

#define NUM_SAMPLES 50 // Number of samples for FULL RC fitting

#define PARTIAL_SAMPLES NUM_SAMPLES/2 // Number of samples for PARTIAL RC measurement

#define SAMPLE_INTERVAL 20 // Interval between samples in milliseconds

#define PARTIAL_SAMPLE_INTERVAL 5 // Interval between samples during partial check (in milliseconds)
#define RC_FIT_ITERATIONS 50 // Number of iterations for RC fitting

#define RC_FIT_LEARNING_RATE 0.001 // Learning rate for RC fitting - adjust as needed

#define DIVERGENCE_THRESHOLD 0.3 // Threshold for divergence detection (adjust based on voltage scale and noise)
// Adaptive calibration interval parameters

#define INTERVAL_INCREASE_FACTOR 1.2f

#define INTERVAL_DECREASE_FACTOR 0.8f

#define MAX_CALIBRATION_INTERVAL 160000 // Maximum calibration interval (2 minutes)

#define MIN_CALIBRATION_INTERVAL 20000  // Minimum calibration interval (20 seconds)
// Known Input Capacitance - DEFINE THIS VALUE!

#define KNOWN_CAPACITANCE 0.020 // Example: 20000 uF in Farads (adjust to your actual capacitance)
// Internal resistance calibration parameters - not used anymore

#define R_CALIB_DUTY_CYCLE_LOW 0.3f  // Low duty cycle for R calibration - not used anymore

#define R_CALIB_DUTY_CYCLE_HIGH 0.6f // High duty cycle for R calibration - not used anymore

#define R_CALIB_ITERATIONS 30       // Iterations for internal resistance calibration - not used anymore

#define R_CALIB_LEARNING_RATE 0.2   // Learning rate for internal resistance calibration - not used anymore

// Verification Parameters
#define VOC_VERIFICATION_INTERVAL 60000 // Interval for VOC verification (e.g., every 60 seconds)
#define VERIFICATION_PWM_PERTURBATION 0.1f // +/- 10% perturbation of current load
#define VERIFICATION_SAMPLES 20 // Number of samples for verification
#define VERIFICATION_SAMPLE_INTERVAL 10 // Sample interval for verification
#define VOC_VERIFICATION_THRESHOLD 0.05 // 5% threshold for VOC verification (as a fraction)

// Define the variables

float open_circuit_voltage = 0;         // The open-circuit voltage of the source (fitted)

float open_circuit_voltage_raw = 0;     // Raw measured open-circuit voltage

float open_circuit_load_voltage = 0;     // The open-circuit voltage of the source - not used anymore

float resistance_tau_est = 0;           // Resistance estimated from Tau and Known Capacitance - kept for tau info
float output_voltage = 0;              // The output voltage - not directly used in core logic

float output_current = 0;              // The output current - not directly used in core logic

float input_current = 0;               // The input current - not directly used in core logic

float output_power = 0;                // The output power - not directly used in core logic

float source_voltage = 0;              // measured source voltage

float load_voltage = 0;                // measured load voltage - not directly used in core logic

float corrected_voltage = 0;           // measured load voltage - not directly used in core logic

float error = 0;                       // error for load adjusting

float load = 0;                        // estimated load - PWM duty cycle control variable
float last_fitted_voc = 0.0;          // Store last fitted Voc for divergence check

float last_fitted_b = 0.0;            // Store last fitted B

float last_fitted_tau = 0.0;          // Store last fitted Tau

float calibration_interval_current = CALIBRATION_INTERVAL; // Adaptive calibration interval
float pwm_value = 0;                 // The PWM value
unsigned long last_calibration_time = -CALIBRATION_INTERVAL; // Initialize to trigger first calibration

unsigned long calibration_start_time = 0;    // The start time of the internal resistance calibration - not used anymore

unsigned long last_debug_output = 0;         // The start time of the debug output

unsigned long last_partial_measurement_time = 0; // Last time partial measurement was performed
unsigned long last_voc_verification_time = 0;  // Last time VOC verification was performed
bool is_calibrating = false;             // The flag for indicating if the internal resistance is being calibrated - not used anymore
// Arrays for RC fitting samples

float voltage_samples[NUM_SAMPLES];

unsigned long time_samples[NUM_SAMPLES];

float partial_voltage_samples[PARTIAL_SAMPLES];

unsigned long partial_time_samples[PARTIAL_SAMPLES];

float verification_voltage_samples[VERIFICATION_SAMPLES];
unsigned long verification_time_samples[VERIFICATION_SAMPLES];

// RC fitting parameters - estimated from fitting

float fitted_voc = 0.00001;

float fitted_b = 0.00001;

float fitted_tau = 0.00001;
// Function to fit RC profile using Gradient Descent (same as before, with R_tau calculation)

bool fit_rc_profile() {

if (NUM_SAMPLES < 2) return false;
fitted_voc = voltage_samples[NUM_SAMPLES - 1];
fitted_b = fitted_voc - voltage_samples[0];
fitted_tau = 0.01;

for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
    float gradient_voc = 0.0;
    float gradient_b = 0.0;
    float gradient_tau = 0.0;
    float error_sum_squares = 0.0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        float t = (time_samples[i] - time_samples[0]) / 1000.0;
        float measured_v = voltage_samples[i];
        float expected_v = fitted_voc - fitted_b * exp(-t /( fitted_tau+0.000001));
        float error = measured_v - expected_v;
        error_sum_squares += error * error;

        gradient_voc += -2.0 * error;
        gradient_b   +=  2.0 * error * exp(-t / (fitted_tau+0.00001));
        gradient_tau +=  2.0 * error * fitted_b * (t / ((fitted_tau * fitted_tau)) * exp(-t / (fitted_tau+0.00001))+0.00001);
    }

    fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / NUM_SAMPLES;
    fitted_b   -= RC_FIT_LEARNING_RATE * gradient_b   / NUM_SAMPLES;
    fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / NUM_SAMPLES;

    fitted_tau = max(fitted_tau, 1e-6);
    fitted_b   = max(fitted_b,   0.0);
    fitted_voc = max(fitted_voc, 0.0);
#ifdef FITTING_DEBUG

if (iteration % (RC_FIT_ITERATIONS / 10) == 0) {

Serial.print(F("RC Fit Iteration: ")); Serial.print(iteration);

Serial.print(F(", Voc: ")); Serial.print(fitted_voc);

Serial.print(F(", B: "));   Serial.print(fitted_b);

Serial.print(F(", Tau: ")); Serial.print(fitted_tau);

Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares/NUM_SAMPLES);

}

#endif

}
if (fitted_voc > 0 && fitted_tau > 1e-6 && fitted_b >= 0) {
    resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE; // Calculate resistance from tau and C - kept for tau info
    return true;
} else {
    return false;
}
}
bool fit_rc_profile_early() {

if (NUM_SAMPLES < 2) return false;
// Initialize parameters with better initial guesses
fitted_voc = voltage_samples[NUM_SAMPLES - 1];  // Final voltage as Voc
fitted_b = fitted_voc - voltage_samples[0];     // Initial voltage drop

// Estimate initial tau using two points method
float v1 = voltage_samples[0];
float v2 = voltage_samples[NUM_SAMPLES/2];
float t1 = (time_samples[0] - time_samples[0]) / 1000.0;
float t2 = (time_samples[NUM_SAMPLES/2] - time_samples[0]) / 1000.0;
fitted_tau = (t2 - t1) / (log((fitted_voc - v1)+0.00001) / ((fitted_voc - v2))+0.00001);

// Adaptive learning rate parameters
float adaptive_lr = RC_FIT_LEARNING_RATE;
float momentum_voc = 0.00001;
float momentum_b = 0.00001;
float momentum_tau = 0.0001;
const float momentum_factor = 0.9;
const float min_improvement = 1e-6;

float prev_error = INFINITY;
int stagnant_iterations = 0;

for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
    float gradient_voc = 0.0001;
    float gradient_b = 0.00001;
    float gradient_tau = 0.00001;
    float error_sum_squares = 0.0;

    // Calculate gradients and error
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float t = (time_samples[i] - time_samples[0]) / 1000.0;
        float measured_v = voltage_samples[i];
        float exp_term = exp(-t / (fitted_tau+0.00001));
        float expected_v = fitted_voc - fitted_b * exp_term;
        float error = measured_v - expected_v;
        error_sum_squares += error * error;

        // More numerically stable gradient calculations
        gradient_voc += -2.0 * error;
        gradient_b += 2.0 * error * exp_term;
        gradient_tau += 2.0 * error * fitted_b * (t / ((fitted_tau * fitted_tau)) * exp_term+0.00001);
    }

    float current_error = error_sum_squares / NUM_SAMPLES;

    // Check for convergence
    if (fabs(prev_error - current_error) < min_improvement) {
        stagnant_iterations++;
        if (stagnant_iterations > 5) {
            Serial.println(F("RC Fit: Early convergence reached"));
            break;
        }
    } else {
        stagnant_iterations = 0;
    }

    // Apply momentum and adaptive learning rate
    momentum_voc = momentum_factor * momentum_voc - adaptive_lr * gradient_voc / NUM_SAMPLES;
    momentum_b = momentum_factor * momentum_b - adaptive_lr * gradient_b / NUM_SAMPLES;
    momentum_tau = momentum_factor * momentum_tau - adaptive_lr * gradient_tau / NUM_SAMPLES;

    fitted_voc += momentum_voc;
    fitted_b += momentum_b;
    fitted_tau += momentum_tau;

    // Parameter constraints with smooth clamping
    fitted_tau = max(fitted_tau, 1e-6);
    fitted_b = max(fitted_b, 0.0);
    fitted_voc = max(fitted_voc, voltage_samples[NUM_SAMPLES - 1] * 0.5);

    // Adaptive learning rate adjustment
    if (current_error > prev_error) {
        adaptive_lr *= 0.5;
    } else if (stagnant_iterations == 0) {
        adaptive_lr *= 1.1;
    }
    adaptive_lr = constrain(adaptive_lr, 0.0001, 0.1);

    prev_error = current_error;

    // Debug output with reduced frequency
    if (iteration % (RC_FIT_ITERATIONS / 5) == 0) {
        Serial.print(F("RC Fit: "));
        Serial.print(iteration);
        Serial.print(F(", fitted_tau: "));
        Serial.print(fitted_tau);
        Serial.print(F(", Error: "));
        Serial.print(current_error, 6);
        Serial.print(F(", LR: "));
        Serial.println(adaptive_lr, 6);
    }
}

// Validation checks
bool parameters_valid = (fitted_voc > 0) &&
                       (fitted_tau > 1e-6) &&
                       (fitted_b >= 0) &&
                       (fitted_voc < voltage_samples[0] * 2.0);  // Sanity check

if (parameters_valid) {
    // Calculate final R estimate using known capacitance - kept for tau info
    resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;

    return true;
}

return false;
}
bool perform_partial_voc_measurement() {

PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM); // switch off load
// 1. Sample partial voltage profile
for (int i = 0; i < PARTIAL_SAMPLES; i++) {
    partial_time_samples[i] = millis();
    partial_voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    delay(PARTIAL_SAMPLE_INTERVAL);
}

// 2. Predict voltages based on last fitted RC profile
float divergence_error_sum = 0.0;
for (int i = 0; i < PARTIAL_SAMPLES; i++) {
    float t = (partial_time_samples[i] - partial_time_samples[0]) / 1000.0;
    float expected_v = last_fitted_voc - last_fitted_b * exp(-t / (last_fitted_tau+0.00001));
    float measured_v = partial_voltage_samples[i];
    divergence_error_sum += pow(measured_v - expected_v, 2); // Sum of squared errors
}
float divergence_mse = divergence_error_sum / PARTIAL_SAMPLES; // Mean Squared Error

// 3. Check for divergence
if (divergence_mse > DIVERGENCE_THRESHOLD) {
#ifdef FITTING_DEBUG

Serial.print(F("Partial Voc Measurement: Divergence detected, MSE: ")); Serial.println(divergence_mse);

#endif

calibration_interval_current *= INTERVAL_DECREASE_FACTOR; // Shorten interval

calibration_interval_current = max(calibration_interval_current, MIN_CALIBRATION_INTERVAL); // Limit minimum

return true; // Divergence detected

} else {

#ifdef FITTING_DEBUG

Serial.print(F("Partial Voc Measurement: No significant divergence, MSE: ")); Serial.println(divergence_mse,4);

#endif

calibration_interval_current *= INTERVAL_INCREASE_FACTOR; // Lengthen interval

calibration_interval_current = min(calibration_interval_current, MAX_CALIBRATION_INTERVAL); // Limit maximum

return false; // No significant divergence

}

}

bool verify_voc() {
    float current_pwm_value = pwm_value; // Store current PWM value
    float verification_voc;
    float verification_b;
    float verification_tau;
    bool fit_verification_success;

    // 1. Oscillate PWM and Sample voltage
    for (int i = 0; i < VERIFICATION_SAMPLES; i++) {
        verification_time_samples[i] = millis();
        // Oscillate PWM around current value
        float perturbation = (i % 2 == 0) ? VERIFICATION_PWM_PERTURBATION : -VERIFICATION_PWM_PERTURBATION;
        float temp_pwm_value = current_pwm_value * (1 + perturbation);
        temp_pwm_value = constrain(temp_pwm_value, MIN_PWM, PWMPeriod * MAX_PWM / 100.0f); // Ensure PWM within bounds
        PWM_Instance->setPWM(PWM_PIN, frequency, temp_pwm_value);
        delay(VERIFICATION_SAMPLE_INTERVAL);
        verification_voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    }
    PWM_Instance->setPWM(PWM_PIN, frequency, current_pwm_value); // Restore original PWM

    // 2. Fit RC profile to verification data (simplified fit might be sufficient)
    fitted_voc = verification_voltage_samples[VERIFICATION_SAMPLES - 1];
    fitted_b = fitted_voc - verification_voltage_samples[0];
    fitted_tau = 0.01; // Initial guess
    for (int iteration = 0; iteration < RC_FIT_ITERATIONS / 5; iteration++) { // Fewer iterations for verification
        float gradient_voc = 0.0;
        float gradient_b = 0.0;
        float gradient_tau = 0.0;
        float error_sum_squares = 0.0;

        for (int i = 0; i < VERIFICATION_SAMPLES; i++) {
            float t = (verification_time_samples[i] - verification_time_samples[0]) / 1000.0;
            float measured_v = verification_voltage_samples[i];
            float expected_v = fitted_voc - fitted_b * exp(-t /( fitted_tau+0.000001));
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            gradient_voc += -2.0 * error;
            gradient_b   +=  2.0 * error * exp(-t / (fitted_tau+0.00001));
            gradient_tau +=  2.0 * error * fitted_b * (t / ((fitted_tau * fitted_tau)) * exp(-t / (fitted_tau+0.00001))+0.00001);
        }

        fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / VERIFICATION_SAMPLES;
        fitted_b   -= RC_FIT_LEARNING_RATE * gradient_b   / VERIFICATION_SAMPLES;
        fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / VERIFICATION_SAMPLES;

        fitted_tau = max(fitted_tau, 1e-6);
        fitted_b   = max(fitted_b,   0.0);
        fitted_voc = max(fitted_voc, 0.0);
    }
    verification_voc = fitted_voc;
    verification_b = fitted_b;
    verification_tau = fitted_tau;
    fit_verification_success = (verification_voc > 0 && verification_tau > 1e-6 && verification_b >= 0);


    // 3. Compare verification Voc with current Voc
    if (fit_verification_success) {
        float voc_difference_percentage = fabs(verification_voc - open_circuit_voltage) / open_circuit_voltage;
        if (voc_difference_percentage <= VOC_VERIFICATION_THRESHOLD) {
            #ifdef FITTING_DEBUG
            Serial.print(F("VOC Verification: PASS, Verified Voc: ")); Serial.print(verification_voc);
            Serial.print(F(", Diff%: ")); Serial.println(voc_difference_percentage*100.0);
            #endif
            return true; // VOC verified successfully
        } else {
            #ifdef FITTING_DEBUG
            Serial.print(F("VOC Verification: FAIL, Verified Voc: ")); Serial.print(verification_voc);
            Serial.print(F(", Diff%: ")); Serial.println(voc_difference_percentage*100.0);
            #endif
            return false; // VOC verification failed
        }
    } else {
        #ifdef FITTING_DEBUG
        Serial.println(F("VOC Verification: Fit Failed"));
        #endif
        return false; // Fit failed during verification
    }
}


void setup() {

// Initialize the serial monitor

Serial.begin(115200);

// Set the PWM pin as output

pinMode(PWM_PIN, OUTPUT);

// Set the initial PWM value to zero

analogWrite(PWM_PIN, MIN_PWM);
frequency = FREQ_MAX;
PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);

if (PWM_Instance)
{
    PWM_Instance->setPWM();
}

PWMPeriod = PWM_Instance->getPWMPeriod();
last_voc_verification_time = millis(); // Initialize verification time
}
void debug_output(){

Serial.print(F("V_oc:"));
Serial.print(open_circuit_voltage);
Serial.print(F(",V_oc_raw:"));
Serial.print(open_circuit_voltage_raw);
Serial.print(F(",V_src:"));
Serial.print(source_voltage);
Serial.print(F(",R_tau:"));
Serial.print(resistance_tau_est); // New output - resistance from tau
Serial.print(F(",load:"));
Serial.print(load,3);
Serial.print(F(",Tau:"));
Serial.print(fitted_tau,4);
Serial.print(F(",CalInt:"));
Serial.println(calibration_interval_current/60000.0);
}
void loop() {

// Read the source voltage

source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
// Perform Partial VOC measurement more frequently
if (millis() - last_partial_measurement_time > PARTIAL_MEASUREMENT_INTERVAL ) {
    perform_partial_voc_measurement();
    last_partial_measurement_time = millis();
}

// Check if it is time to verify VOC
if (millis() - last_voc_verification_time > VOC_VERIFICATION_INTERVAL) {
    verify_voc(); // Perform VOC verification
    last_voc_verification_time = millis();
}

// Check if it is time to calibrate the open-circuit voltage and rint
if (millis() - last_calibration_time > calibration_interval_current) { // Use adaptive interval
    // Set the PWM value to zero
    PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);
    last_calibration_time = millis();
    // Start sampling for full open-circuit voltage measurement
    for (int i = 0; i < NUM_SAMPLES; i++) {
        time_samples[i] = millis();
        voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
        delay(SAMPLE_INTERVAL);
    }

    open_circuit_voltage_raw = voltage_samples[NUM_SAMPLES - 1]; // Store raw measurement

    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);

    // Fit RC profile using Gradient Descent
    bool fit_success = fit_rc_profile(); // or fit_rc_profile_early(); for testing early fit
    if (fit_success) {
        open_circuit_voltage = fitted_voc; // Use fitted Voc

        // Store fitted parameters for partial measurement prediction
        last_fitted_voc = fitted_voc;
        last_fitted_b = fitted_b;
        last_fitted_tau = fitted_tau;
#ifdef FITTING_DEBUG

Serial.print(F("Full RC Fit Success, Voc: ")); Serial.print(open_circuit_voltage);

Serial.print(F(", Tau: ")); Serial.println(fitted_tau);

#endif

} else {

open_circuit_voltage = open_circuit_voltage_raw; // Fallback to raw if fitting fails

#ifdef FITTING_DEBUG

Serial.println(F("Full RC Fit Failed, using raw Voc."));

#endif

}

}//if (millis() - last_calibration_time > calibration_interval_current) {
// Voltage regulation logic - simplified, resistance calculations removed
load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0); // Keep reading load voltage if needed for monitoring
source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);


error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - source_voltage)/2; // Simple error calculation

load = load - LEARNING_RATE_LOAD * error; // Adjust load (PWM duty cycle) based on error
load = constrain(load, 0, 1); // Constrain load to 0-1 range
pwm_value = (PWMPeriod)*load ;  // Calculate PWM value
PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value); // Set PWM


if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
    debug_output();
    last_debug_output = millis();
}
}
