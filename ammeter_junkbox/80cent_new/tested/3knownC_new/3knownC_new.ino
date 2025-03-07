#include "AVR_PWM.h"
#include <math.h> // Required for exp(), pow()
#include <Arduino.h>

AVR_PWM* PWM_Instance;

float frequency;

#define FITTING_DEBUG // uncomment to get messages from calibration fitters

// Define the pins
#define PWM_PIN 9 // The pin for the PWM output to the load
#define LOAD_PIN A1 // The pin for measuring the load voltage
#define LOAD_VOLTAGE_RANGE 30.0 // the max value of load analog input maps to this voltage, in Volts
#define SOURCE_PIN A0 // The pin for measuring the source voltage
#define SOURCE_VOLTAGE_RANGE 28.0 // the max value of source analog input maps to this voltage, in Volts

// Define the constants
#define MAX_PWM 99.0 // The maximum PWM duty cycle
#define MIN_PWM 0.0 // The minimum PWM duty cycle
uint16_t PWMPeriod = 0 ;
#define FREQ_MAX 14000.0;

#define LOAD_FACTOR 0.001 // The factor for increasing or decreasing the load
#define CALIBRATION_FACTOR 0.0001 // The factor for increasing or decreasing the load during RINT calibration
#define VOLTAGE_FACTOR 0.80 // The factor for keeping the output voltage within 80% of the open-circuit voltage
#define CALIBRATION_INTERVAL 20000 // Initial interval for full calibration in milliseconds
#define OPEN_CIRCUIT_MEASUREMENT_TIME 50 // Reduced time for initial measurement, will be followed by sampling
#define CALIBRATION_DURATION 3000 // The duration for calibrating the internal resistance in milliseconds
#define RINT_CALIBRATION_MEASUREMENT_TIME 100 // time to wait after changing pwm value before measuring voltage across load
#define LEARNING_RATE 0.001 // The learning rate for the gradient descent algorithm
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

//#define MAX_CALIBRATION_INTERVAL 25000 // Maximum calibration interval (2 minutes)
//#define MIN_CALIBRATION_INTERVAL 20000  // Minimum calibration interval (20 seconds)


// Known Input Capacitance - DEFINE THIS VALUE!
//#define KNOWN_CAPACITANCE 0.0001 // Example: 0.1 uF in Farads (adjust to your actual capacitance)
#define KNOWN_CAPACITANCE 0.020 // Example: 20000 uF in Farads (adjust to your actual capacitance)

// Internal resistance calibration parameters
#define R_CALIB_DUTY_CYCLE_LOW 0.3f  // Low duty cycle for R calibration
#define R_CALIB_DUTY_CYCLE_HIGH 0.6f // High duty cycle for R calibration
#define R_CALIB_ITERATIONS 15       // Iterations for internal resistance calibration
#define R_CALIB_LEARNING_RATE 0.1   // Learning rate for internal resistance calibration

// Define the variables
float open_circuit_voltage = 0;         // The open-circuit voltage of the source (fitted)
float open_circuit_voltage_raw = 0;     // Raw measured open-circuit voltage
float open_circuit_load_voltage = 0;     // The open-circuit voltage of the source
float internal_resistance_src = 1.0;       // The internal resistance of the source (from power/voltage/current) - Initial guess
float internal_resistance_load = 0.1;      // The internal resistance of the load - Initial guess
float load_resistance = 10.0;             // The load resistance - Initial guess
float source_to_load_resistance = 0;    // The load resistance - not used directly
float resistance_tau_est = 0;           // Resistance estimated from Tau and Known Capacitance

float output_voltage = 0;              // The output voltage
float output_current = 0;              // The output current
float input_current = 0;               // The input current
float output_power = 0;                // The output power
float source_voltage = 0;              // measured source voltage
float load_voltage = 0;                // measured load voltage
float corrected_voltage = 0;           // measured load voltage
float error = 0;                       // error for load adjusting
float load = 0;                        // estimated load

float last_fitted_voc = 0.0;          // Store last fitted Voc for divergence check
float last_fitted_b = 0.0;            // Store last fitted B
float last_fitted_tau = 0.0;          // Store last fitted Tau
float calibration_interval_current = CALIBRATION_INTERVAL; // Adaptive calibration interval

//uint16_t pwm_value = 0; // The PWM value
float pwm_value = 0;                 // The PWM value

unsigned long last_calibration_time = -CALIBRATION_INTERVAL; // Initialize to trigger first calibration
unsigned long calibration_start_time = 0;    // The start time of the internal resistance calibration
unsigned long last_debug_output = 0;         // The start time of the internal resistance calibration
unsigned long last_partial_measurement_time = 0; // Last time partial measurement was performed

bool is_calibrating = false;             // The flag for indicating if the internal resistance is being calibrated

// Arrays for RC fitting samples
float voltage_samples[NUM_SAMPLES];
unsigned long time_samples[NUM_SAMPLES];
float partial_voltage_samples[PARTIAL_SAMPLES];
unsigned long partial_time_samples[PARTIAL_SAMPLES];

// RC fitting parameters - estimated from fitting
float fitted_voc = 0.00001;
float fitted_b = 0.00001;
float fitted_tau = 0.00001;

// Function to fit RC profile using Gradient Descent (same as before, with R_tau calculation)
bool fit_rc_profile_gradient() {
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
    resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE; // Calculate resistance from tau and C
    return true;
} else {
    return false;
}

}

//-----------------------early optimized fitter

bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false;

    // Constants for parameter constraints
    constexpr float FALLBACK_TAU      = 0.01f;
    constexpr float MIN_TAU           = 1e-6f;
    constexpr float MAX_TAU           = 1.0f;
    constexpr float MIN_LR            = 1e-5f;
    constexpr float MAX_LR            = 0.1f;
    constexpr float MOMENTUM          = 0.9f;
    constexpr float MIN_IMPROVEMENT   = 1e-6f;
    
    // Initial parameter estimates
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];
    fitted_b   = fitted_voc - voltage_samples[0];
    
    float t0 = time_samples[0] / 1000.0f;  // Convert to seconds

    // Estimate initial tau using multiple data points
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

    // Gradient descent parameters
    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float prev_grad_voc = 0.0f, prev_grad_b = 0.0f, prev_grad_tau = 0.0f;
    float prev_error = INFINITY;
    int stagnant_iters = 0;

    // Main optimization loop
    for (int iter = 0; iter < RC_FIT_ITERATIONS; iter++) {
        float grad_voc = 0.0f, grad_b = 0.0f, grad_tau = 0.0f, error_sum = 0.0f;
        float inv_tau = 1.0f / max(fitted_tau, MIN_TAU);  // Precompute division for efficiency

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] / 1000.0f) - t0;
            float exp_term = expf(-t * inv_tau);
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = voltage_samples[i] - expected_v;
            error_sum += error * error;

            // Compute gradients
            grad_voc += -2.0f * error;
            grad_b   +=  2.0f * error * exp_term;
            grad_tau += (fitted_b * t * exp_term * grad_b * inv_tau) * 2.0f;
        }

        float current_error = error_sum / NUM_SAMPLES;

        // Early stopping if no improvement
        if (fabsf(prev_error - current_error) < MIN_IMPROVEMENT) {
            if (++stagnant_iters > 5) break;
        } else {
            stagnant_iters = 0;
        }

        // Apply momentum
        grad_voc = MOMENTUM * prev_grad_voc + (1 - MOMENTUM) * grad_voc;
        grad_b   = MOMENTUM * prev_grad_b   + (1 - MOMENTUM) * grad_b;
        grad_tau = MOMENTUM * prev_grad_tau + (1 - MOMENTUM) * grad_tau;

        prev_grad_voc = grad_voc;
        prev_grad_b   = grad_b;
        prev_grad_tau = grad_tau;

        // Update parameters
        float inv_samples = 1.0f / NUM_SAMPLES;  // Precompute division
        fitted_voc -= adaptive_lr * grad_voc * inv_samples;
        fitted_b   -= adaptive_lr * grad_b * inv_samples;
        fitted_tau -= adaptive_lr * grad_tau * inv_samples;

        // Constrain values to reasonable bounds
        fitted_tau = constrain(fitted_tau, MIN_TAU, MAX_TAU);
        fitted_b   = max(fitted_b, 0.0f);
        fitted_voc = constrain(fitted_voc, 
                               voltage_samples[NUM_SAMPLES - 1] * 0.8f,
                               voltage_samples[NUM_SAMPLES - 1] * 1.2f);

        // Adjust learning rate adaptively
        adaptive_lr = constrain(adaptive_lr * (current_error > prev_error ? 0.5f : 1.1f), MIN_LR, MAX_LR);
        prev_error = current_error;

#ifdef FITTING_DEBUG
        if (iter % (RC_FIT_ITERATIONS / 5) == 0) {
            Serial.print(F("RC Fit: Iter "));
            Serial.print(iter);
            Serial.print(F(", Voc: "));
            Serial.print(fitted_voc, 4);
            Serial.print(F(", B: "));
            Serial.print(fitted_b, 4);
            Serial.print(F(", Tau: "));
            Serial.print(fitted_tau, 4);
            Serial.print(F(", Error: "));
            Serial.println(current_error, 6);
        }
#endif
    }

    // Final parameter validation
    bool valid_params = (fitted_voc > 0.0f) &&
                        (fitted_tau > MIN_TAU) &&
                        (fitted_b >= 0.0f) &&
                        (fitted_voc < voltage_samples[0] * 1.5f);

    if (valid_params) {
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

}

void debug_output(){

Serial.print(F("V_oc:"));
Serial.print(open_circuit_voltage);
Serial.print(F(",V_oc_raw:"));
Serial.print(open_circuit_voltage_raw);
Serial.print(F(",V_cor:"));
Serial.print(corrected_voltage);
Serial.print(F(",V_src:"));
Serial.print(source_voltage);
Serial.print(F(",V_load:"));
Serial.print(load_voltage);
Serial.print(F(",R_src:")); // Changed output name for clarity
Serial.print(internal_resistance_src);
Serial.print(F(",R_load_est:")); // Changed output name for clarity
Serial.print(load_resistance);
Serial.print(F(",R_tau:"));
Serial.print(resistance_tau_est); // New output - resistance from tau
Serial.print(F(",load:"));
Serial.print(load,3);
Serial.print(F(",Tau:"));
Serial.print(fitted_tau,4);
Serial.print(F(",CalInt:"));
Serial.println(calibration_interval_current/60000.0);

}

// Function to predict load voltage based on estimated resistances and duty cycle
float predict_load_voltage(float r_src, float r_load, float duty_cycle, float voc) {
    return (duty_cycle * voc * r_load) / (r_src + r_load+0.00001);
}

// Function to predict source voltage based on estimated resistances and duty cycle
float predict_source_voltage(float r_src, float r_load, float duty_cycle, float voc) {
    float predicted_load_v = predict_load_voltage(r_src, r_load, duty_cycle, voc);
    return voc - r_src * (predicted_load_v / (r_load+0.00001)); // Using Ohm's law: Vs = Voc - Rsrc * I, and I = Vl/Rload
}

void calibrate_internal_resistance() {
#ifdef FITTING_DEBUG
    Serial.println(F("Starting Internal Resistance Calibration..."));
#endif
    unsigned long calibration_start_time = millis();
    float current_r_src = internal_resistance_src;
    float current_r_load = load_resistance;

    for (int iteration = 0; iteration < R_CALIB_ITERATIONS; iteration++) {
        float grad_r_src = 0.0;
        float grad_r_load = 0.0;
        float error_sum_squares = 0.0;

        float duty_cycle_low = R_CALIB_DUTY_CYCLE_LOW;
        float duty_cycle_high = R_CALIB_DUTY_CYCLE_HIGH;

        float load_pwm_low = duty_cycle_low * (PWMPeriod);
        float load_pwm_high = duty_cycle_high * (PWMPeriod);

        // Measurement 1: Low Duty Cycle
        PWM_Instance->setPWM(PWM_PIN, frequency, load_pwm_low);
        delay(RINT_CALIBRATION_MEASUREMENT_TIME);
        float measured_v_load_low = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
        float measured_v_source_low = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

        // Measurement 2: High Duty Cycle
        PWM_Instance->setPWM(PWM_PIN, frequency, load_pwm_high);
        delay(RINT_CALIBRATION_MEASUREMENT_TIME);
        float measured_v_load_high = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
        float measured_v_source_high = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

        // Predictions from the model
        float predicted_v_load_low = predict_load_voltage(current_r_src, current_r_load, duty_cycle_low, open_circuit_voltage);
        float predicted_v_source_low = predict_source_voltage(current_r_src, current_r_load, duty_cycle_low, open_circuit_voltage);
        float predicted_v_load_high = predict_load_voltage(current_r_src, current_r_load, duty_cycle_high, open_circuit_voltage);
        float predicted_v_source_high = predict_source_voltage(current_r_src, current_r_load, duty_cycle_high, open_circuit_voltage);

        // Calculate errors
        float error_v_load_low   = predicted_v_load_low - measured_v_load_low;
        float error_v_source_low = predicted_v_source_low - measured_v_source_low;
        float error_v_load_high  = predicted_v_load_high - measured_v_load_high;
        float error_v_source_high= predicted_v_source_high - measured_v_source_high;

        error_sum_squares = error_v_load_low*error_v_load_low + error_v_source_low*error_v_source_low + error_v_load_high*error_v_load_high + error_v_source_high*error_v_source_high;

        // Calculate gradients - using derived partial derivatives (simplified, assuming Voc is constant for differentiation)
        float common_denominator_low = pow(current_r_src + current_r_load, 2)+0.00001;
        float common_denominator_high = pow(current_r_src + current_r_load, 2)+0.00001;

        float dVlp_dRsrc_low   = - (duty_cycle_low  * open_circuit_voltage * current_r_load) / common_denominator_low;
        float dVlp_dRload_low  =   (duty_cycle_low  * open_circuit_voltage * current_r_src ) / common_denominator_low;
        float dVsp_dRsrc_low   = - (predicted_v_load_low / (current_r_load+0.00001)) - (current_r_src / (current_r_load+0.00001)) * dVlp_dRsrc_low;
        float dVsp_dRload_low  =   (current_r_src / pow(current_r_load, 2)) * predicted_v_load_low - (current_r_src / (current_r_load+0.00001)) * dVlp_dRload_low;

        float dVlp_dRsrc_high  = - (duty_cycle_high * open_circuit_voltage * current_r_load) / common_denominator_high;
        float dVlp_dRload_high =   (duty_cycle_high * open_circuit_voltage * current_r_src ) / common_denominator_high;
        float dVsp_dRsrc_high  = - (predicted_v_load_high / current_r_load) - (current_r_src / (current_r_load+0.00001)) * dVlp_dRsrc_high;
        float dVsp_dRload_high =   (current_r_src / pow(current_r_load, 2)) * predicted_v_load_high - (current_r_src / (current_r_load+0.00001)) * dVlp_dRload_high;


        grad_r_src  = 2 * (error_v_source_low * dVsp_dRsrc_low + error_v_load_low * dVlp_dRsrc_low + error_v_source_high * dVsp_dRsrc_high + error_v_load_high * dVlp_dRsrc_high);
        grad_r_load = 2 * (error_v_source_low * dVsp_dRload_low + error_v_load_low * dVlp_dRload_low + error_v_source_high * dVsp_dRload_high + error_v_load_high * dVlp_dRload_high);


        current_r_src  -= R_CALIB_LEARNING_RATE * grad_r_src;
        current_r_load -= R_CALIB_LEARNING_RATE * grad_r_load;

        current_r_src  = max(current_r_src, 0.001f); // Prevent negative resistance, add small offset to avoid division by zero.
        current_r_load = max(current_r_load, 0.001f);

#ifdef FITTING_DEBUG
        if (iteration % (R_CALIB_ITERATIONS / 10) == 0) {
            Serial.print(F("R Calib Iteration: ")); Serial.print(iteration);
            Serial.print(F(", R_src: ")); Serial.print(current_r_src);
            Serial.print(F(", R_load: ")); Serial.print(current_r_load);
            Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares);
        }
#endif        
    }

    internal_resistance_src = current_r_src;
    load_resistance = current_r_load;
    is_calibrating = false;
    last_calibration_time = millis();
#ifdef FITTING_DEBUG
    Serial.println(F("Internal Resistance Calibration complete."));
#endif
}


void loop() {
// Read the source voltage
source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

// Perform Partial VOC measurement more frequently
if (millis() - last_partial_measurement_time > PARTIAL_MEASUREMENT_INTERVAL && !is_calibrating) {
    perform_partial_voc_measurement();
    last_partial_measurement_time = millis();
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
    open_circuit_load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);     // read open circuit voltage of the load

    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);

    // Fit RC profile using Gradient Descent
    bool fit_success = fit_rc_profile();
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

    // Set the calibration flag to true to trigger the rint calibration
    is_calibrating = true;
    calibrate_internal_resistance(); // Call the new calibration function

}//if (millis() - last_calibration_time > calibration_interval_current) {


if (!is_calibrating) { // Only adjust load if not in calibration

/*
    load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

    output_current  = load_voltage / (load_resistance+0.001); // Use estimated load resistance
    input_current = source_voltage / (load_resistance + internal_resistance_src+0.001); // Use estimated source resistance

    output_power = output_current * load_voltage;

    corrected_voltage = (open_circuit_voltage - input_current * internal_resistance_src);

    error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - corrected_voltage)/2;

    load = load - LEARNING_RATE_LOAD * error;
    load = constrain(load, 0, 1);
    pwm_value = (PWMPeriod)*load ;
    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);
*/

    load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

//    load_resistance = (load_voltage / ((source_voltage-load_voltage)/(1-load)+0.001));
    output_current  = load_voltage / (load_resistance+0.0001);

    source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.0001);
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.0001);
    //load_resistance = ((load_voltage) / (((1-load)+0.001)) - internal_resistance_load) ;

    input_current = source_voltage / (load_resistance + source_to_load_resistance+0.00001);
//    input_current = (1-load)+0.0001;
 
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.0001);

    output_power = output_current * load_voltage;

    //float internal_resistance_adj_src = (internal_resistance_src *0.9 + 0.1*(open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001))) ;

    corrected_voltage = (open_circuit_voltage - input_current * (-internal_resistance_src));

    error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - source_voltage)/2;

    load = load - LEARNING_RATE_LOAD * error;
    load = constrain(load, 0, 1);
    pwm_value = (PWMPeriod)*load ;
    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);



}


if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
    debug_output();
    last_debug_output = millis();
}

}
