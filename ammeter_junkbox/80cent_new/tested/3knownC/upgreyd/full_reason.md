
Conversation with Gemini

please improve fitting algorithm in following code :



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

#define MIN_CALIBRATION_INTERVAL 20000 // Minimum calibration interval (20 seconds)



// Known Input Capacitance - DEFINE THIS VALUE!

//#define KNOWN_CAPACITANCE 0.0001 // Example: 0.1 uF in Farads (adjust to your actual capacitance)

#define KNOWN_CAPACITANCE 20.0 // Example: 20000 uF in Farads (adjust to your actual capacitance)



// Define the variables

float open_circuit_voltage = 0; // The open-circuit voltage of the source (fitted)

float open_circuit_voltage_raw = 0; // Raw measured open-circuit voltage

float open_circuit_load_voltage = 0; // The open-circuit voltage of the source

float internal_resistance_src = 0; // The internal resistance of the source (from power/voltage/current)

float internal_resistance_load = 0; // The internal resistance of the load

float load_resistance = 0; // The load resistance

float source_to_load_resistance = 0; // The load resistance

float resistance_tau_est = 0; // Resistance estimated from Tau and Known Capacitance



float output_voltage = 0; // The output voltage

float output_current = 0; // The output current

float input_current = 0; // The input current

float output_power = 0; // The output power

float source_voltage = 0; // measured source voltage

float load_voltage = 0; // measured load voltage

float corrected_voltage = 0; // measured load voltage

float error = 0; // error for load adjusting

float load = 0; // estimated load



float last_fitted_voc = 0.0; // Store last fitted Voc for divergence check

float last_fitted_b = 0.0; // Store last fitted B

float last_fitted_tau = 0.0; // Store last fitted Tau

float calibration_interval_current = CALIBRATION_INTERVAL; // Adaptive calibration interval



//uint16_t pwm_value = 0; // The PWM value

float pwm_value = 0; // The PWM value



unsigned long last_calibration_time = -CALIBRATION_INTERVAL; // Initialize to trigger first calibration

unsigned long calibration_start_time = 0; // The start time of the internal resistance calibration

unsigned long last_debug_output = 0; // The start time of the internal resistance calibration

unsigned long last_partial_measurement_time = 0; // Last time partial measurement was performed



bool is_calibrating = false; // The flag for indicating if the internal resistance is being calibrated



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

gradient_b += 2.0 * error * exp(-t / fitted_tau);

gradient_tau += 2.0 * error * fitted_b * (t / (fitted_tau * fitted_tau)) * exp(-t / fitted_tau);

}



fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / NUM_SAMPLES;

fitted_b -= RC_FIT_LEARNING_RATE * gradient_b / NUM_SAMPLES;

fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / NUM_SAMPLES;



fitted_tau = max(fitted_tau, 1e-6);

fitted_b = max(fitted_b, 0.0);

fitted_voc = max(fitted_voc, 0.0);

if (iteration % (RC_FIT_ITERATIONS / 10) == 0) {

Serial.print(F("RC Fit Iteration: ")); Serial.print(iteration);

Serial.print(F(", Voc: ")); Serial.print(fitted_voc);

Serial.print(F(", B: ")); Serial.print(fitted_b);

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



// uint16_t last_pwm_value = pwm_value;

// pwm_value = 0;

PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);



// Start sampling for full open-circuit voltage measurement

for (int i = 0; i < NUM_SAMPLES; i++) {

time_samples[i] = millis();

voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

delay(SAMPLE_INTERVAL);

}



open_circuit_voltage_raw = voltage_samples[NUM_SAMPLES - 1]; // Store raw measurement

open_circuit_load_voltage = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0); // read open circuit voltage of the load



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

load_voltage = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);

source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);



internal_resistance_load = (open_circuit_load_voltage-load_voltage) / ((1-load)+0.001);

load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ;



output_current = load_voltage / (load_resistance+internal_resistance_load+0.001);

float source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001);



output_power = output_current * output_current * (load_resistance+internal_resistance_load);

internal_resistance_src = (open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001)) ;



float cost = output_power - (open_circuit_voltage * open_circuit_voltage) / ((4 * internal_resistance_src)+0.01);

float gradient = (open_circuit_voltage * open_circuit_voltage) / (((4 * internal_resistance_src * internal_resistance_src) + output_current * output_current)+0.001);

internal_resistance_src = internal_resistance_src - gradient * LEARNING_RATE+0.001;

internal_resistance_src = max(internal_resistance_src, 0);

}

}



load_voltage = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);

source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);



load_resistance = (load_voltage / ((source_voltage-load_voltage)/(1-load)+0.001));

output_current = load_voltage / (load_resistance+0.001);



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




Show thinking
The user wants to improve the fitting algorithm in the provided Arduino code.
The code seems to be for controlling a PWM output to maintain a desired voltage across a load, while adapting to changes in source and load characteristics.

Looking at the code, the "fitting algorithm" seems to refer to:

RC Fitting: Used to estimate open_circuit_voltage based on voltage decay curve when PWM is turned off. This is implemented in fit_rc_profile() and used in perform_partial_voc_measurement() and loop() when calibration is due.
Internal Resistance Calibration: Done during is_calibrating phase in the loop(). This involves adjusting load (PWM duty cycle) and measuring voltages to estimate internal_resistance_src and other parameters.
The user wants to improve the fitting.  This is vague. What aspects of fitting can be improved?

Accuracy: Are the estimated open_circuit_voltage, internal_resistance_src, fitted_tau reliable?
Robustness: How well does the fitting work under noisy measurements, varying load conditions, and different source characteristics?
Speed/Efficiency: Can the fitting be done faster or with fewer samples? (Though in Arduino context, accuracy/robustness are probably more critical than speed).
Let's review the existing fitting algorithms in detail and consider potential improvements for each:

1. RC Fitting (fit_rc_profile()):

Method: Gradient Descent to fit parameters of a single exponential decay model: V(t) = Voc - B * exp(-t / Tau).
Parameters to fit: fitted_voc, fitted_b, fitted_tau.
Data used: voltage_samples[], time_samples[].
Potential issues/improvements:
Model Simplification: Is a single exponential decay model accurate enough for the source? Real sources might have more complex behavior. Maybe higher order models are needed? (Probably overkill for Arduino).
Gradient Descent: Simple GD is used. Could more advanced optimization algorithms (e.g., Adam, L-BFGS) converge faster or to better solutions? (Likely too complex for Arduino and might not be necessary).
Learning Rate: RC_FIT_LEARNING_RATE = 0.01 is fixed. Adaptive learning rate might improve convergence. (Could try simple adaptive approaches).
Initialization: Initial values fitted_voc = voltage_samples[NUM_SAMPLES - 1], fitted_b = fitted_voc - voltage_samples[0], fitted_tau = 0.01 seem reasonable.
Number of iterations: RC_FIT_ITERATIONS = 250. Is this enough for convergence? Too much? (Monitor convergence and adjust).
Error Metric: Sum of Squared Errors is used, which is standard.
Constraint: fitted_tau = max(fitted_tau, 1e-6), fitted_b = max(fitted_b, 0.0), fitted_voc = max(fitted_voc, 0.0) are reasonable constraints.
Divergence Check in perform_partial_voc_measurement(): This is a good addition to detect when the model is no longer valid.
2. Internal Resistance Calibration (is_calibrating block in loop()):

Method: Iterative adjustment of internal_resistance_src and internal_resistance_load based on voltage and current measurements. Seems to use a gradient descent-like approach, though less clearly defined than RC fitting.
Parameters to estimate: internal_resistance_src, internal_resistance_load.
Data used: open_circuit_load_voltage, load_voltage, source_voltage, load.
Potential issues/improvements:
Calibration Process: The calibration process in is_calibrating block looks somewhat heuristic and less rigorously defined compared to RC fitting. It seems to iterate for a fixed duration CALIBRATION_DURATION.
Gradient Calculation: Gradient for internal_resistance_src calculation in is_calibrating block looks simplified and possibly inaccurate. float gradient = (open_circuit_voltage * open_circuit_voltage) / (((4 * internal_resistance_src * internal_resistance_src) + output_current * output_current)+0.001); and internal_resistance_src = internal_resistance_src - gradient * LEARNING_RATE+0.001; . What is the cost function being minimized here? It's related to output_power - (open_circuit_voltage * open_circuit_voltage) / ((4 * internal_resistance_src)+0.01). This looks like trying to maximize power transfer, maybe? But it's not very clear and could be mathematically improved and clarified.
Learning Rate: LEARNING_RATE = 0.001 is fixed. Adaptive learning rate might be beneficial here too.
internal_resistance_load calculation: internal_resistance_load = (open_circuit_load_voltage-load_voltage) / ((1-load)+0.001); and load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ; looks somewhat ad-hoc and needs review. Why (1-load) in the denominator? load is related to PWM duty cycle, not directly resistance. It should be related to current.
Stability and Convergence: Is the calibration process stable? Does it converge to accurate resistance values?
Plan for Improvement:

Clarify the Goal of Fitting: What parameters are we trying to estimate accurately? open_circuit_voltage, internal_resistance_src, internal_resistance_load, load_resistance. What is the overall objective of the system (voltage regulation, power transfer)?
Review RC Fitting: It seems reasonably well-i

```c++
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

// Internal resistance calibration parameters
#define R_CALIB_DUTY_CYCLE_LOW 0.2f  // Low duty cycle for R calibration
#define R_CALIB_DUTY_CYCLE_HIGH 0.8f // High duty cycle for R calibration
#define R_CALIB_ITERATIONS 100       // Iterations for internal resistance calibration
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
Serial.print(F(",R_src:")); // Changed output name for clarity
Serial.print(internal_resistance_src);
Serial.print(F(",R_load_est:")); // Changed output name for clarity
Serial.print(load_resistance);
Serial.print(F(",R_tau:"));
Serial.print(resistance_tau_est); // New output - resistance from tau
Serial.print(F(",load:"));
Serial.print(load,3);
Serial.print(F(",Tau:"));
Serial.print(fitted_tau);
Serial.print(F(",CalInt:"));
Serial.println(calibration_interval_current);

}

// Function to predict load voltage based on estimated resistances and duty cycle
float predict_load_voltage(float r_src, float r_load, float duty_cycle, float voc) {
    return (duty_cycle * voc * r_load) / (r_src + r_load);
}

// Function to predict source voltage based on estimated resistances and duty cycle
float predict_source_voltage(float r_src, float r_load, float duty_cycle, float voc) {
    float predicted_load_v = predict_load_voltage(r_src, r_load, duty_cycle, voc);
    return voc - r_src * (predicted_load_v / r_load); // Using Ohm's law: Vs = Voc - Rsrc * I, and I = Vl/Rload
}

void calibrate_internal_resistance() {
    Serial.println(F("Starting Internal Resistance Calibration..."));
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
        float common_denominator_low = pow(current_r_src + current_r_load, 2);
        float common_denominator_high = pow(current_r_src + current_r_load, 2);

        float dVlp_dRsrc_low   = - (duty_cycle_low  * open_circuit_voltage * current_r_load) / common_denominator_low;
        float dVlp_dRload_low  =   (duty_cycle_low  * open_circuit_voltage * current_r_src ) / common_denominator_low;
        float dVsp_dRsrc_low   = - (predicted_v_load_low / current_r_load) - (current_r_src / current_r_load) * dVlp_dRsrc_low;
        float dVsp_dRload_low  =   (current_r_src / pow(current_r_load, 2)) * predicted_v_load_low - (current_r_src / current_r_load) * dVlp_dRload_low;

        float dVlp_dRsrc_high  = - (duty_cycle_high * open_circuit_voltage * current_r_load) / common_denominator_high;
        float dVlp_dRload_high =   (duty_cycle_high * open_circuit_voltage * current_r_src ) / common_denominator_high;
        float dVsp_dRsrc_high  = - (predicted_v_load_high / current_r_load) - (current_r_src / current_r_load) * dVlp_dRsrc_high;
        float dVsp_dRload_high =   (current_r_src / pow(current_r_load, 2)) * predicted_v_load_high - (current_r_src / current_r_load) * dVlp_dRload_high;


        grad_r_src  = 2 * (error_v_source_low * dVsp_dRsrc_low + error_v_load_low * dVlp_dRsrc_low + error_v_source_high * dVsp_dRsrc_high + error_v_load_high * dVlp_dRsrc_high);
        grad_r_load = 2 * (error_v_source_low * dVsp_dRload_low + error_v_load_low * dVlp_dRload_low + error_v_source_high * dVsp_dRload_high + error_v_load_high * dVlp_dRload_high);


        current_r_src  -= R_CALIB_LEARNING_RATE * grad_r_src;
        current_r_load -= R_CALIB_LEARNING_RATE * grad_r_load;

        current_r_src  = max(current_r_src, 0.01f); // Prevent negative resistance, add small offset to avoid division by zero.
        current_r_load = max(current_r_load, 0.01f);

        if (iteration % (R_CALIB_ITERATIONS / 10) == 0) {
            Serial.print(F("R Calib Iteration: ")); Serial.print(iteration);
            Serial.print(F(", R_src: ")); Serial.print(current_r_src);
            Serial.print(F(", R_load: ")); Serial.print(current_r_load);
            Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares);
        }
    }

    internal_resistance_src = current_r_src;
    load_resistance = current_r_load;
    is_calibrating = false;
    last_calibration_time = millis();
    Serial.println(F("Internal Resistance Calibration complete."));
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

    // Set the calibration flag to true to trigger the rint calibration
    is_calibrating = true;
    calibrate_internal_resistance(); // Call the new calibration function

}//if (millis() - last_calibration_time > calibration_interval_current) {


if (!is_calibrating) { // Only adjust load if not in calibration
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
}


if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
    debug_output();
    last_debug_output = millis();
}

}
```

**Improvements implemented:**

1.  **Improved Internal Resistance Calibration (`calibrate_internal_resistance()`):**
    *   **Model-Based Approach:**  Uses a simplified circuit model (source with Voc and Rsrc in series, load with Rload) and derives formulas for predicted load voltage (`predict_load_voltage`) and source voltage (`predict_source_voltage`) based on duty cycle and resistances.
    *   **Gradient Descent for Rsrc and Rload:** Implements Gradient Descent to iteratively refine estimates of `internal_resistance_src` (`Rsrc`) and `load_resistance` (`Rload`).
    *   **Error Function:** Minimizes the sum of squared errors between predicted and measured source and load volume 
