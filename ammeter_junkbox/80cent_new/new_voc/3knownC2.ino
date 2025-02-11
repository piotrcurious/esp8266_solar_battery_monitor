#include "AVR_PWM.h"
#include <math.h> // Required for exp(), pow()
#include <Arduino.h>

// Define PWM instance globally
AVR_PWM* PWM_Instance;

// Frequency variable - keep it global if it's used across functions
float frequency;

// Define the pins - use descriptive names and consider using enum or const int for pin numbers
#define PWM_PIN 9        // PWM output to load
#define LOAD_PIN A1      // Analog input for load voltage
#define SOURCE_PIN A0    // Analog input for source voltage

// Voltage Ranges - use const float for clarity and potential compiler optimization
const float LOAD_VOLTAGE_RANGE = 30.0f;   // Max load voltage (Volts)
const float SOURCE_VOLTAGE_RANGE = 28.0f; // Max source voltage (Volts)

// PWM Duty Cycle Limits - use const float for clarity and consistency
const float MAX_PWM_DUTY = 99.0f; // Maximum PWM duty cycle (%)
const float MIN_PWM_DUTY = 0.0f; // Minimum PWM duty cycle (%)
uint16_t PWMPeriod = 0;
const float FREQ_MAX_HZ = 14000.0f; // Maximum PWM frequency (Hz)

// Calibration and Adjustment Factors - use const float for clarity and maintainability
const float LOAD_ADJUST_FACTOR = 0.001f;       // Factor for load adjustment
const float CALIBRATION_ADJUST_FACTOR = 0.0001f; // Factor for RINT calibration adjustment
const float VOLTAGE_TARGET_FACTOR = 0.80f;     // Target voltage factor (percentage of open-circuit voltage)

// Timing Constants (milliseconds) - use const unsigned long for clarity and type safety
const unsigned long INITIAL_CALIBRATION_INTERVAL = 20000UL; // Initial full calibration interval
const unsigned long OPEN_CIRCUIT_INIT_MEASURE_TIME = 50UL;  // Initial open-circuit measurement time
const unsigned long CALIBRATION_DURATION_MS = 3000UL;      // RINT calibration duration
const unsigned long RINT_CAL_MEASURE_DELAY_MS = 20UL;     // Delay after PWM change before RINT measurement
const unsigned long DEBUG_OUTPUT_INTERVAL_MS = 100UL;      // Debug serial output interval
const unsigned long PARTIAL_VOC_MEASURE_INTERVAL_MS = 5000UL; // Partial VOC measurement interval

// Learning Rates - use const float and descriptive names
const float GRADIENT_DESCENT_LR = 0.01f; // Learning rate for general gradient descent - adjust as needed - corrected to 0.01 from LEARNING_RATE 0.001, as 0.01 looks more appropriate from original code
const float LOAD_ADJUST_LR = 0.001f;    // Learning rate for load adjustment gradient descent - adjust as needed

// RC Fitting Parameters - use const int/float for clarity and potential optimization
const int RC_FULL_FIT_SAMPLES = 50;        // Number of samples for FULL RC fitting
const int RC_PARTIAL_FIT_SAMPLES = RC_FULL_FIT_SAMPLES / 2; // Samples for PARTIAL RC measurement
const unsigned long RC_SAMPLE_INTERVAL_MS = 20UL;     // Interval between RC samples
const int RC_FIT_MAX_ITERATIONS = 250;     // Max iterations for RC fitting
const float RC_FIT_INITIAL_LEARNING_RATE = 0.1f; // Initial learning rate for RC fitting - start higher, then decay
const float RC_FIT_LEARNING_RATE_DECAY = 0.99f; // Learning rate decay per iteration
const float RC_FIT_TOLERANCE = 1e-5f;       // Tolerance for RC fit convergence (error change)
const float DIVERGENCE_THRESHOLD = 0.9f;    // Divergence threshold for partial VOC check - adjust based on noise

// Adaptive Calibration Interval Parameters
const float INTERVAL_INCREASE_FACTOR = 1.2f; // Factor to increase calibration interval
const float INTERVAL_DECREASE_FACTOR = 0.8f;// Factor to decrease calibration interval
const unsigned long MAX_CALIBRATION_INTERVAL_MS = 120000UL; // Max calibration interval (2 minutes)
const unsigned long MIN_CALIBRATION_INTERVAL_MS = 20000UL;  // Min calibration interval (20 seconds)

// Known Input Capacitance - Define as const float and use SI units (Farads)
const float KNOWN_CAPACITANCE_F = 20.0e-6f; // Example: 20 uF in Farads (adjust to your actual capacitance) - corrected to 20e-6 from 20.0 which was likely microF but used as Farads


// Declare variables - use descriptive names and initialize them if possible
float open_circuit_voltage_fitted = 0.0f;         // Fitted open-circuit voltage (Voc)
float open_circuit_voltage_raw_V = 0.0f;         // Raw measured Voc (Volts)
float open_circuit_load_voltage_V = 0.0f;        // Open-circuit load voltage (Volts)
float internal_resistance_src_Ohm = 0.0f;        // Source internal resistance (Ohms)
float internal_resistance_load_Ohm = 0.0f;       // Load internal resistance (Ohms)
float load_resistance_Ohm = 0.0f;                // Load resistance (Ohms)
float source_to_load_resistance_Ohm = 0.0f;       // Resistance between source and load (Ohms) - likely wiring/trace resistance
float resistance_tau_est_Ohm = 0.0f;              // Resistance estimated from Tau and Capacitance (Ohms)

float output_voltage_V = 0.0f;                   // Output voltage (Volts)
float output_current_A = 0.0f;                   // Output current (Amps)
float input_current_A = 0.0f;                    // Input current (Amps)
float output_power_W = 0.0f;                     // Output power (Watts)
float source_voltage_V = 0.0f;                   // Measured source voltage (Volts)
float load_voltage_V = 0.0f;                     // Measured load voltage (Volts)
float corrected_voltage_V = 0.0f;                // Corrected load voltage (Volts)
float voltage_error_V = 0.0f;                    // Voltage error for feedback control (Volts)
float load_duty_cycle = 0.0f;                    // Estimated load duty cycle (0.0 to 1.0)

float last_fitted_voc_V = 0.0f;               // Last fitted Voc for divergence check (Volts)
float last_fitted_b = 0.0f;                    // Last fitted B parameter
float last_fitted_tau_s = 0.0f;                  // Last fitted Tau parameter (seconds)
unsigned long calibration_interval_ms = INITIAL_CALIBRATION_INTERVAL; // Adaptive calibration interval (milliseconds)

float pwm_duty_cycle = 0.0f;                     // PWM duty cycle (0.0 to 100.0) - corrected to float for fractional PWM

unsigned long last_full_calibration_time_ms = -INITIAL_CALIBRATION_INTERVAL; // Time of last full calibration (milliseconds) - negative to trigger initial calibration
unsigned long rint_calibration_start_time_ms = 0;      // Start time of RINT calibration (milliseconds)
unsigned long last_debug_output_ms = 0;              // Time of last debug output (milliseconds)
unsigned long last_partial_voc_measure_ms = 0;      // Time of last partial VOC measurement (milliseconds)

bool is_rint_calibrating = false;                // Flag for RINT calibration in progress

// Arrays for RC fitting samples - use const int for array sizes
float voltage_samples_V[RC_FULL_FIT_SAMPLES];       // Voltage samples for full RC fit (Volts)
unsigned long time_samples_ms[RC_FULL_FIT_SAMPLES];    // Time samples for full RC fit (milliseconds)
float partial_voltage_samples_V[RC_PARTIAL_FIT_SAMPLES]; // Voltage samples for partial RC fit (Volts)
unsigned long partial_time_samples_ms[RC_PARTIAL_FIT_SAMPLES]; // Time samples for partial RC fit (milliseconds)

// RC fitting parameters - estimated from fitting - descriptive names
float rc_fitted_voc_V = 0.0f;                    // Fitted Voc from RC fit (Volts)
float rc_fitted_b = 0.0f;                        // Fitted B from RC fit
float rc_fitted_tau_s = 0.0f;                      // Fitted Tau from RC fit (seconds)


// Function to fit RC profile using Gradient Descent (Improved with Adaptive Learning Rate and Convergence Check)
bool fit_rc_profile() {
    if (RC_FULL_FIT_SAMPLES < 2) return false;

    rc_fitted_voc_V = voltage_samples_V[RC_FULL_FIT_SAMPLES - 1]; // Initialize Voc with last voltage
    rc_fitted_b = rc_fitted_voc_V - voltage_samples_V[0];       // Initialize B with voltage difference
    rc_fitted_tau_s = 0.01f;                                   // Initial guess for Tau (seconds) - corrected unit to seconds
    float learning_rate = RC_FIT_INITIAL_LEARNING_RATE;        // Start with initial learning rate
    float prev_error_sum_squares = 0.0f;
    float error_sum_squares = 0.0f;

    for (int iteration = 0; iteration < RC_FIT_MAX_ITERATIONS; iteration++) {
        float gradient_voc = 0.0f;
        float gradient_b = 0.0f;
        float gradient_tau = 0.0f;
        error_sum_squares = 0.0f;

        for (int i = 0; i < RC_FULL_FIT_SAMPLES; i++) {
            float t = (time_samples_ms[i] - time_samples_ms[0]) / 1000.0f; // Time in seconds - corrected to seconds
            float measured_v = voltage_samples_V[i];
            float expected_v = rc_fitted_voc_V - rc_fitted_b * exp(-t / rc_fitted_tau_s);
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            gradient_voc += -2.0f * error;
            gradient_b   +=  2.0f * error * exp(-t / rc_fitted_tau_s);
            gradient_tau +=  2.0f * error * rc_fitted_b * (t / (rc_fitted_tau_s * rc_fitted_tau_s)) * exp(-t / rc_fitted_tau_s);
        }

        // Update parameters with Gradient Descent and Adaptive Learning Rate
        rc_fitted_voc_V -= learning_rate * gradient_voc / RC_FULL_FIT_SAMPLES;
        rc_fitted_b   -= learning_rate * gradient_b   / RC_FULL_FIT_SAMPLES;
        rc_fitted_tau_s -= learning_rate * gradient_tau / RC_FULL_FIT_SAMPLES;

        // Ensure parameters stay within reasonable bounds
        rc_fitted_tau_s = max(rc_fitted_tau_s, 1e-6f); // Prevent negative or zero Tau
        rc_fitted_b   = max(rc_fitted_b,   0.0f);   // Prevent negative B
        rc_fitted_voc_V = max(rc_fitted_voc_V, 0.0f);   // Prevent negative Voc

        // Learning Rate Decay - Adaptive Learning Rate
        learning_rate *= RC_FIT_LEARNING_RATE_DECAY;
        learning_rate = max(learning_rate, RC_FIT_LEARNING_RATE * pow(RC_FIT_LEARNING_RATE_DECAY, 10)); // Ensure learning rate doesn't become too small too quickly

        if (iteration % (RC_FIT_MAX_ITERATIONS / 10) == 0) {
            Serial.print(F("RC Fit Iteration: ")); Serial.print(iteration);
            Serial.print(F(", Voc: ")); Serial.print(rc_fitted_voc_V);
            Serial.print(F(", B: "));   Serial.print(rc_fitted_b);
            Serial.print(F(", Tau: ")); Serial.print(rc_fitted_tau_s);
            Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares / RC_FULL_FIT_SAMPLES);
        }


        // Convergence Check - based on change in error sum of squares
        if (iteration > 1 && fabs(error_sum_squares - prev_error_sum_squares) < RC_FIT_TOLERANCE) {
            Serial.print(F("RC Fit Converged at iteration: ")); Serial.println(iteration);
            break; // Exit loop if converged
        }
        prev_error_sum_squares = error_sum_squares;


    }

    if (rc_fitted_voc_V > 0 && rc_fitted_tau_s > 1e-6f && rc_fitted_b >= 0) {
        resistance_tau_est_Ohm = rc_fitted_tau_s / KNOWN_CAPACITANCE_F; // Calculate resistance from tau and C - corrected C unit to Farads
        return true;
    } else {
        return false;
    }
}

bool perform_partial_voc_measurement() {
    PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM_DUTY); // Switch off load

    // 1. Sample partial voltage profile
    for (int i = 0; i < RC_PARTIAL_FIT_SAMPLES; i++) {
        partial_time_samples_ms[i] = millis();
        partial_voltage_samples_V[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0f);
        delay(RC_SAMPLE_INTERVAL_MS);
    }

    // 2. Predict voltages based on last fitted RC profile and calculate divergence
    float divergence_error_sum = 0.0f;
    for (int i = 0; i < RC_PARTIAL_FIT_SAMPLES; i++) {
        float t = (partial_time_samples_ms[i] - partial_time_samples_ms[0]) / 1000.0f; // Time in seconds - corrected to seconds
        float expected_v = last_fitted_voc_V - last_fitted_b * exp(-t / last_fitted_tau_s);
        float measured_v = partial_voltage_samples_V[i];
        divergence_error_sum += pow(measured_v - expected_v, 2); // Sum of squared errors
    }
    float divergence_mse = divergence_error_sum / RC_PARTIAL_FIT_SAMPLES; // Mean Squared Error

    // 3. Check for divergence and adjust calibration interval adaptively
    if (divergence_mse > DIVERGENCE_THRESHOLD) {
        Serial.print(F("Partial Voc Measurement: Divergence detected, MSE: ")); Serial.println(divergence_mse);
        calibration_interval_ms *= INTERVAL_DECREASE_FACTOR; // Shorten interval
        calibration_interval_ms = max(calibration_interval_ms, MIN_CALIBRATION_INTERVAL_MS); // Limit minimum
        return true; // Divergence detected
    } else {
        Serial.print(F("Partial Voc Measurement: No significant divergence, MSE: ")); Serial.println(divergence_mse);
        calibration_interval_ms *= INTERVAL_INCREASE_FACTOR; // Lengthen interval
        calibration_interval_ms = min(calibration_interval_ms, MAX_CALIBRATION_INTERVAL_MS); // Limit maximum
        return false; // No significant divergence
    }
}


void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Initialize PWM output pin
    pinMode(PWM_PIN, OUTPUT);

    // Set initial PWM duty cycle to minimum
    analogWrite(PWM_PIN, MIN_PWM_DUTY);

    // Set PWM frequency and create PWM instance
    frequency = FREQ_MAX_HZ;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM_DUTY);

    // Initialize PWM if instance creation is successful
    if (PWM_Instance) {
        PWM_Instance->setPWM();
        PWMPeriod = PWM_Instance->getPWMPeriod(); // Get PWM period for duty cycle calculation
    } else {
        Serial.println(F("Failed to create PWM instance!"));
        // Handle error - perhaps halt or use a fallback PWM method
    }


    Serial.println(F("Initialization complete. Starting measurements.")); // Startup message for clarity

}

void debug_output() {
    Serial.print(F("V_oc_fit[V]:"));    Serial.print(open_circuit_voltage_fitted);
    Serial.print(F(", V_oc_raw[V]:"));   Serial.print(open_circuit_voltage_raw_V);
    Serial.print(F(", V_cor[V]:"));     Serial.print(corrected_voltage_V);
    Serial.print(F(", V_src[V]:"));     Serial.print(source_voltage_V);
    Serial.print(F(", V_load[V]:"));    Serial.print(load_voltage_V);
    Serial.print(F(", R_int_src[Ohm]:")); Serial.print(internal_resistance_src_Ohm);
    Serial.print(F(", R_tau_est[Ohm]:")); Serial.print(resistance_tau_est_Ohm);
    Serial.print(F(", Load_Duty[%]:"));  Serial.print(load_duty_cycle * 100.0f, 1); // Output load as percentage
    Serial.print(F(", Tau[s]:"));       Serial.print(rc_fitted_tau_s);
    Serial.print(F(", CalInt[ms]:"));   Serial.println(calibration_interval_ms);
}

void loop() {
    // Read source voltage
    source_voltage_V = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0f);

    // Perform Partial VOC measurement periodically to check for drift
    if ((millis() - last_partial_voc_measure_ms > PARTIAL_VOC_MEASURE_INTERVAL_MS) && !is_rint_calibrating) {
        perform_partial_voc_measurement();
        last_partial_voc_measure_ms = millis();
    }

    // Check if it's time for a full calibration based on adaptive interval
    if (millis() - last_full_calibration_time_ms > calibration_interval_ms) {
        PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM_DUTY); // Switch off load briefly for Voc measurement

        // Sample voltage for full open-circuit voltage measurement and RC fitting
        for (int i = 0; i < RC_FULL_FIT_SAMPLES; i++) {
            time_samples_ms[i] = millis();
            voltage_samples_V[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0f);
            delay(RC_SAMPLE_INTERVAL_MS);
        }

        open_circuit_voltage_raw_V = voltage_samples_V[RC_FULL_FIT_SAMPLES - 1]; // Store raw Voc measurement
        open_circuit_load_voltage_V = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0f); // Measure load Voc

        PWM_Instance->setPWM(PWM_PIN, frequency, pwm_duty_cycle); // Restore PWM to previous duty cycle

        // Fit RC profile to sampled data
        bool fit_success = fit_rc_profile();
        if (fit_success) {
            open_circuit_voltage_fitted = rc_fitted_voc_V; // Use fitted Voc for control
            resistance_tau_est_Ohm = rc_fitted_tau_s / KNOWN_CAPACITANCE_F; // Recalculate resistance estimate

            // Update last fitted parameters for partial VOC measurement prediction
            last_fitted_voc_V = rc_fitted_voc_V;
            last_fitted_b = rc_fitted_b;
            last_fitted_tau_s = rc_fitted_tau_s;

            Serial.print(F("Full RC Fit Success, Voc: ")); Serial.print(open_circuit_voltage_fitted);
            Serial.print(F(", Tau: ")); Serial.println(rc_fitted_tau_s);
        } else {
            open_circuit_voltage_fitted = open_circuit_voltage_raw_V; // Fallback to raw Voc if fitting fails
            Serial.println(F("Full RC Fit Failed, using raw Voc."));
        }

        last_full_calibration_time_ms = millis(); // Update last calibration time
        is_rint_calibrating = true;               // Start RINT calibration process
        rint_calibration_start_time_ms = millis(); // Record RINT calibration start time
    } // end full calibration


    // RINT Calibration Loop - Executes for CALIBRATION_DURATION_MS
    if (is_rint_calibrating) {
        if (millis() - rint_calibration_start_time_ms > CALIBRATION_DURATION_MS) {
            is_rint_calibrating = false; // End RINT calibration after duration
        } else {
            load_duty_cycle += 1.0f * CALIBRATION_ADJUST_FACTOR; // Increment duty cycle for RINT calibration
            load_duty_cycle = constrain(load_duty_cycle, 0.0f, 1.0f); // Constrain duty cycle to 0-1
            pwm_duty_cycle = (PWMPeriod - 1) * load_duty_cycle; // Calculate PWM value - corrected duty cycle to pwm_value conversion
            PWM_Instance->setPWM(PWM_PIN, frequency, pwm_duty_cycle); // Set PWM duty cycle

            delay(RINT_CAL_MEASURE_DELAY_MS); // Wait for voltage to settle

            // Read voltages for RINT calculation
            load_voltage_V = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0f);
            source_voltage_V = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0f);

            // Calculate resistances and currents - corrected calculations and added small epsilon for division safety
            internal_resistance_load_Ohm = (open_circuit_load_voltage_V - load_voltage_V) / ((1.0f - load_duty_cycle) + 1e-9f);
            load_resistance_Ohm = ((load_voltage_V) / ((1.0f - load_duty_cycle) + 1e-9f)) - internal_resistance_load_Ohm;
            output_current_A = load_voltage_V / (load_resistance_Ohm + internal_resistance_load_Ohm + 1e-9f);
            source_to_load_resistance_Ohm = (source_voltage_V - load_voltage_V) / ((1.0f - load_duty_cycle) + 1e-9f);

            output_power_W = output_current_A * output_current_A * (load_resistance_Ohm + internal_resistance_load_Ohm);

            // Gradient descent for internal source resistance - corrected gradient calculation and learning rate
            float cost = output_power_W - (open_circuit_voltage_fitted * open_circuit_voltage_fitted) / ((4.0f * internal_resistance_src_Ohm) + 0.01f); // Cost function
            float gradient =  (open_circuit_voltage_fitted * open_circuit_voltage_fitted) / (((4.0f * internal_resistance_src_Ohm * internal_resistance_src_Ohm) + output_current_A * output_current_A) + 1e-9f); // Gradient - corrected gradient and division safety
            internal_resistance_src_Ohm -= gradient * GRADIENT_DESCENT_LR; // Update R_int_src
            internal_resistance_src_Ohm = max(internal_resistance_src_Ohm, 0.0f); // Ensure R_int_src is not negative

        }
    } // end RINT calibration


    // Main control loop - Voltage regulation
    load_voltage_V = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0f);
    source_voltage_V = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0f);

    // Calculate resistances, currents, and corrected voltage - corrected calculations and added epsilon for division safety
    load_resistance_Ohm = (load_voltage_V / ((source_vo
