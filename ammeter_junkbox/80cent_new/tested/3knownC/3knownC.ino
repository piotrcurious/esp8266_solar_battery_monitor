#include "AVR_PWM.h"
#include <math.h> // Required for exp(), pow()
#include <Arduino.h>

AVR_PWM* PWM_Instance;

float frequency;

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
#define RINT_CALIBRATION_MEASUREMENT_TIME 20 // time to wait after changing pwm value before measuring voltage across load
#define LEARNING_RATE 0.001 // The learning rate for the gradient descent algorithm
#define LEARNING_RATE_LOAD 0.001 // The learning rate for the gradient descent algorithm - load adjustment

#define DEBUG_SERIAL_OUTPUT_INTERVAL 100 // interval for debug output for visualization
#define PARTIAL_MEASUREMENT_INTERVAL 5000 // Interval for partial VOC measurement

// RC fitting parameters
#define NUM_SAMPLES 50 // Number of samples for FULL RC fitting
#define PARTIAL_SAMPLES NUM_SAMPLES/2 // Number of samples for PARTIAL RC measurement
#define SAMPLE_INTERVAL 20 // Interval between samples in milliseconds
#define RC_FIT_ITERATIONS 250 // Number of iterations for RC fitting
#define RC_FIT_LEARNING_RATE 0.01 // Learning rate for RC fitting - adjust as needed
#define DIVERGENCE_THRESHOLD 0.9 // Threshold for divergence detection (adjust based on voltage scale and noise)

// Adaptive calibration interval parameters
#define INTERVAL_INCREASE_FACTOR 1.2f
#define INTERVAL_DECREASE_FACTOR 0.8f
#define MAX_CALIBRATION_INTERVAL 120000 // Maximum calibration interval (2 minutes)
#define MIN_CALIBRATION_INTERVAL 20000  // Minimum calibration interval (20 seconds)

// Known Input Capacitance - DEFINE THIS VALUE!
//#define KNOWN_CAPACITANCE 0.0001 // Example: 0.1 uF in Farads (adjust to your actual capacitance)
#define KNOWN_CAPACITANCE 20.0 // Example: 20000 uF in Farads (adjust to your actual capacitance)

// Define the variables
float open_circuit_voltage = 0;         // The open-circuit voltage of the source (fitted)
float open_circuit_voltage_raw = 0;     // Raw measured open-circuit voltage
float open_circuit_load_voltage = 0;     // The open-circuit voltage of the source
float internal_resistance_src = 0;       // The internal resistance of the source (from power/voltage/current)
float internal_resistance_load = 0;      // The internal resistance of the load
float load_resistance = 0;             // The load resistance
float source_to_load_resistance = 0;    // The load resistance
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
float fitted_voc = 0.0;
float fitted_b = 0.0;
float fitted_tau = 0.0;

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
            float expected_v = fitted_voc - fitted_b * exp(-t / fitted_tau);
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            gradient_voc += -2.0 * error;
            gradient_b   +=  2.0 * error * exp(-t / fitted_tau);
            gradient_tau +=  2.0 * error * fitted_b * (t / (fitted_tau * fitted_tau)) * exp(-t / fitted_tau);
        }

        fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / NUM_SAMPLES;
        fitted_b   -= RC_FIT_LEARNING_RATE * gradient_b   / NUM_SAMPLES;
        fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / NUM_SAMPLES;

        fitted_tau = max(fitted_tau, 1e-6);
        fitted_b   = max(fitted_b,   0.0);
        fitted_voc = max(fitted_voc, 0.0);
        if (iteration % (RC_FIT_ITERATIONS / 10) == 0) {
          Serial.print(F("RC Fit Iteration: ")); Serial.print(iteration);
          Serial.print(F(", Voc: ")); Serial.print(fitted_voc);
          Serial.print(F(", B: "));   Serial.print(fitted_b);
          Serial.print(F(", Tau: ")); Serial.print(fitted_tau);
          Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares/NUM_SAMPLES);
        }
    }

    if (fitted_voc > 0 && fitted_tau > 1e-6 && fitted_b >= 0) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE; // Calculate resistance from tau and C
        return true;
    } else {
        return false;
    }
}

bool perform_partial_voc_measurement() {
    PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM); // switch off load
  
    // 1. Sample partial voltage profile
    for (int i = 0; i < PARTIAL_SAMPLES; i++) {
        partial_time_samples[i] = millis();
        partial_voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
        delay(SAMPLE_INTERVAL);
    }

    // 2. Predict voltages based on last fitted RC profile
    float divergence_error_sum = 0.0;
    for (int i = 0; i < PARTIAL_SAMPLES; i++) {
        float t = (partial_time_samples[i] - partial_time_samples[0]) / 1000.0;
        float expected_v = last_fitted_voc - last_fitted_b * exp(-t / last_fitted_tau);
        float measured_v = partial_voltage_samples[i];
        divergence_error_sum += pow(measured_v - expected_v, 2); // Sum of squared errors
    }
    float divergence_mse = divergence_error_sum / PARTIAL_SAMPLES; // Mean Squared Error

    // 3. Check for divergence
    if (divergence_mse > DIVERGENCE_THRESHOLD) {
        Serial.print(F("Partial Voc Measurement: Divergence detected, MSE: ")); Serial.println(divergence_mse);
        calibration_interval_current *= INTERVAL_DECREASE_FACTOR; // Shorten interval
        calibration_interval_current = max(calibration_interval_current, MIN_CALIBRATION_INTERVAL); // Limit minimum
        return true; // Divergence detected
    } else {
        Serial.print(F("Partial Voc Measurement: No significant divergence, MSE: ")); Serial.println(divergence_mse);
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
    Serial.print(F(",R_int:"));
    Serial.print(internal_resistance_src);
    Serial.print(F(",R_tau:"));
    Serial.print(resistance_tau_est); // New output - resistance from tau
    Serial.print(F(",load:"));
    Serial.print(load,3);
    Serial.print(F(",Tau:"));
    Serial.print(fitted_tau);
    Serial.print(F(",CalInt:"));
    Serial.println(calibration_interval_current);
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
//        uint16_t last_pwm_value = pwm_value; 
//        pwm_value = 0;
        PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);

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

            Serial.print(F("Full RC Fit Success, Voc: ")); Serial.print(open_circuit_voltage);
            Serial.print(F(", Tau: ")); Serial.println(fitted_tau);
        } else {
            open_circuit_voltage = open_circuit_voltage_raw; // Fallback to raw if fitting fails
            Serial.println(F("Full RC Fit Failed, using raw Voc."));
        }


        // Update the last calibration time
        last_calibration_time = millis();
        // Set the calibration flag to true to trigger the rint calibration
        is_calibrating = true;
        // Set the rint calibration start time
        calibration_start_time = millis();
    }//if (millis() - last_calibration_time > calibration_interval_current) {


    // Check if the internal resistance is being calibrated (same as before)
    if (is_calibrating) {
        if (millis() - calibration_start_time > CALIBRATION_DURATION) {
            is_calibrating = false;
        } else {
            load = load + 1 * CALIBRATION_FACTOR;
            load = constrain(load, 0, 1);
            pwm_value = (PWMPeriod-1)*load ;
            PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);

            delay(RINT_CALIBRATION_MEASUREMENT_TIME);
            load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
            source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

            internal_resistance_load = (open_circuit_load_voltage-load_voltage) / ((1-load)+0.001);
            load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ;

            output_current = load_voltage / (load_resistance+internal_resistance_load+0.001);
            float source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001);

            output_power = output_current * output_current * (load_resistance+internal_resistance_load);
            internal_resistance_src = (open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001)) ;

            float cost = output_power - (open_circuit_voltage * open_circuit_voltage) / ((4 * internal_resistance_src)+0.01);
            float gradient =  (open_circuit_voltage * open_circuit_voltage) / (((4 * internal_resistance_src * internal_resistance_src) + output_current * output_current)+0.001);
            internal_resistance_src = internal_resistance_src - gradient * LEARNING_RATE+0.001;
            internal_resistance_src = max(internal_resistance_src, 0);
        }
    }

    load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

    load_resistance = (load_voltage / ((source_voltage-load_voltage)/(1-load)+0.001));
    output_current  = load_voltage / (load_resistance+0.001);

    source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001);
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);
    load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ;

    input_current = source_voltage / (load_resistance + source_to_load_resistance+0.001);
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);

    output_power = output_current * load_voltage;

    internal_resistance_src = (internal_resistance_src *0.9 + 0.1*(open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001))) ;

    corrected_voltage = (open_circuit_voltage - input_current * internal_resistance_src);

    error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - corrected_voltage)/2;

    load = load - LEARNING_RATE_LOAD * error;
    load = constrain(load, 0, 1);
    pwm_value = (PWMPeriod)*load ;
    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);


    if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        debug_output();
        last_debug_output = millis();
    }
}
