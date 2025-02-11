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
#define CALIBRATION_INTERVAL 20000 // The interval for calibrating the open-circuit voltage in milliseconds
#define OPEN_CIRCUIT_MEASUREMENT_TIME 50 // Reduced time for initial measurement, will be followed by sampling
#define CALIBRATION_DURATION 3000 // The duration for calibrating the internal resistance in milliseconds
#define RINT_CALIBRATION_MEASUREMENT_TIME 20 // time to wait after changing pwm value before measuring voltage across load
#define LEARNING_RATE 0.001 // The learning rate for the gradient descent algorithm
#define LEARNING_RATE_LOAD 0.001 // The learning rate for the gradient descent algorithm - load adjustment

#define DEBUG_SERIAL_OUTPUT_INTERVAL 100 // interval for debug output for visualization

// RC fitting parameters
#define NUM_SAMPLES 20 // Number of samples for RC fitting
#define SAMPLE_INTERVAL 5 // Interval between samples in milliseconds
#define RC_FIT_ITERATIONS 100 // Number of iterations for RC fitting
#define RC_FIT_LEARNING_RATE 0.1 // Learning rate for RC fitting - adjust as needed

// Define the variables
float open_circuit_voltage = 0; // The open-circuit voltage of the source (fitted)
float open_circuit_voltage_raw = 0; // Raw measured open-circuit voltage
float open_circuit_load_voltage = 0; // The open-circuit voltage of the source
float internal_resistance_src = 0; // The internal resistance of the source
float internal_resistance_load = 0; // The internal resistance of the source
float load_resistance = 0; // The load resistance
float source_to_load_resistance = 0; // The load resistance

float output_voltage = 0; // The output voltage
float output_current = 0; // The output current
float input_current = 0; // The input current
float output_power = 0; // The output power
float source_voltage = 0; // measured source voltage
float load_voltage = 0; // measured load voltage
float corrected_voltage = 0; // measured load voltage
float error = 0 ; // error for load adjusting
float load = 0 ; // estimated load

//uint16_t pwm_value = 0; // The PWM value
float pwm_value = 0; // The PWM value

unsigned long last_calibration_time = CALIBRATION_INTERVAL; // The last time the open-circuit voltage was calibrated
unsigned long calibration_start_time = 0; // The start time of the internal resistance calibration
unsigned long last_debug_output = 0; // The start time of the internal resistance calibration

bool is_calibrating = false; // The flag for indicating if the internal resistance is being calibrated

// Arrays for RC fitting samples
float voltage_samples[NUM_SAMPLES];
unsigned long time_samples[NUM_SAMPLES];

// RC fitting parameters - estimated from fitting
float fitted_voc = 0.0;
float fitted_b = 0.0;
float fitted_tau = 0.0;

// Function to fit RC profile using Gradient Descent
bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false; // Not enough samples

    // Initialize parameters for fitting (crude initial guesses)
    fitted_voc = voltage_samples[NUM_SAMPLES - 1]; // Last sample as initial Voc
    fitted_b = fitted_voc - voltage_samples[0];     // Initial B (approx. voltage difference)
    fitted_tau = 0.01;                             // Initial Tau (time constant, e.g., 10ms)

    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
        float gradient_voc = 0.0;
        float gradient_b = 0.0;
        float gradient_tau = 0.0;
        float error_sum_squares = 0.0;

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] - time_samples[0]) / 1000.0; // Time in seconds
            float measured_v = voltage_samples[i];
            float expected_v = fitted_voc - fitted_b * exp(-t / fitted_tau); // RC model: V(t) = Voc - B*exp(-t/tau)
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            // Gradients for each parameter (derived from the RC model)
            gradient_voc += -2.0 * error;
            gradient_b   +=  2.0 * error * exp(-t / fitted_tau);
            gradient_tau +=  2.0 * error * fitted_b * (t / (fitted_tau * fitted_tau)) * exp(-t / fitted_tau);
        }

        // Update parameters using gradient descent
        fitted_voc -= RC_FIT_LEARNING_RATE * gradient_voc / NUM_SAMPLES; // Average gradient
        fitted_b   -= RC_FIT_LEARNING_RATE * gradient_b   / NUM_SAMPLES;
        fitted_tau -= RC_FIT_LEARNING_RATE * gradient_tau / NUM_SAMPLES;

        // Ensure tau and B are positive and Voc is reasonable (add constraints if needed)
        fitted_tau = max(fitted_tau, 1e-6); // Avoid zero or negative tau
        fitted_b   = max(fitted_b,   0.0);
        fitted_voc = max(fitted_voc, 0.0);
        if (iteration % (RC_FIT_ITERATIONS / 10) == 0) { // Debug output every 10% iterations
          Serial.print(F("RC Fit Iteration: ")); Serial.print(iteration);
          Serial.print(F(", Voc: ")); Serial.print(fitted_voc);
          Serial.print(F(", B: "));   Serial.print(fitted_b);
          Serial.print(F(", Tau: ")); Serial.print(fitted_tau);
          Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares/NUM_SAMPLES);
        }

    }

    if (fitted_voc > 0 && fitted_tau > 1e-6 && fitted_b >= 0) {
        return true; // Fitting successful
    } else {
        return false; // Fitting failed - parameters not reasonable
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
    Serial.print(F(",load:"));
    Serial.print(load,3);
    Serial.print(F(",Tau:"));
    Serial.print(fitted_tau);
    Serial.println();
}

void loop() {
    // Read the source voltage
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

    // Check if it is time to calibrate the open-circuit voltage and rint
    if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
        // Set the PWM value to zero
        pwm_value = 0;
        PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);

        // Start sampling for open-circuit voltage measurement
        for (int i = 0; i < NUM_SAMPLES; i++) {
            time_samples[i] = millis();
            voltage_samples[i] = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
            delay(SAMPLE_INTERVAL);
        }

        open_circuit_voltage_raw = voltage_samples[NUM_SAMPLES - 1]; // Store raw measurement

        // Fit RC profile using Gradient Descent
        bool fit_success = fit_rc_profile();
        if (fit_success) {
            open_circuit_voltage = fitted_voc; // Use fitted Voc
            Serial.print(F("RC Fit Success, Voc: ")); Serial.print(open_circuit_voltage); Serial.print(F(", Tau: ")); Serial.println(fitted_tau);
        } else {
            open_circuit_voltage = open_circuit_voltage_raw; // Fallback to raw if fitting fails
            Serial.println(F("RC Fit Failed, using raw Voc."));
        }


        open_circuit_load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);     // read open circuit voltage of the load

        // Update the last calibration time
        last_calibration_time = millis();
        // Set the calibration flag to true to trigger the rint calibration
        is_calibrating = true;
        // Set the rint calibration start time
        calibration_start_time = millis();
    }//if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {


    // Check if the internal resistance is being calibrated
    if (is_calibrating) {
        // Check if the calibration duration is over
        if (millis() - calibration_start_time > CALIBRATION_DURATION) {
            // Set the calibration flag to false
            is_calibrating = false;
            // Reset the PWM value
            //   load = 0;
        } else {
            // Increase the PWM value by CALIBRATION_FACTOR
            load = load + 1 * CALIBRATION_FACTOR;
            // Constrain the PWM value within the limits
            load = constrain(load, 0, 1);
            // Write the PWM value to the pin
            pwm_value = (PWMPeriod-1)*load ;
            PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);

            delay(RINT_CALIBRATION_MEASUREMENT_TIME); // delay for some time to allow load voltage to change
            // Read the load voltage
            load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
            source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

            // Calculate the load resistance
            internal_resistance_load = (open_circuit_load_voltage-load_voltage) / ((1-load)+0.001);
            load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ; // only resistance of the load itself

            // Calculate the output current
            output_current = load_voltage / (load_resistance+internal_resistance_load+0.001);
            float source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001);

            // Calculate the output power
            output_power = output_current * output_current * (load_resistance+internal_resistance_load);
            internal_resistance_src = (open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001)) ;

            // Calculate the internal resistance using the gradient descent algorithm
            float cost = output_power - (open_circuit_voltage * open_circuit_voltage) / ((4 * internal_resistance_src)+0.01);
            float gradient =  (open_circuit_voltage * open_circuit_voltage) / (((4 * internal_resistance_src * internal_resistance_src) + output_current * output_current)+0.001);
            internal_resistance_src = internal_resistance_src - gradient * LEARNING_RATE+0.001;
            // Constrain the internal resistance to be positive
            internal_resistance_src = max(internal_resistance_src, 0);
        }
    }

    load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

    // Calculate the load resistance
    load_resistance = (load_voltage / ((source_voltage-load_voltage)/(1-load)+0.001)); //total
    output_current  = load_voltage / (load_resistance+0.001);

    source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001);
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);
    load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ; // only resistance of the load itself

    input_current = source_voltage / (load_resistance + source_to_load_resistance+0.001);
    internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);

    // Calculate the output power
    output_power = output_current * load_voltage;

    internal_resistance_src = (internal_resistance_src *0.9 + 0.1*(open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001))) ;


    // Use the open-circuit voltage corrected by the internal resistance and the load
    corrected_voltage = (open_circuit_voltage - input_current * internal_resistance_src);


    error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - corrected_voltage)/2;

    // Adjust the load value using gradient descent algorithm
    load = load - LEARNING_RATE_LOAD * error;
    // Constrain the load value between 0 and 1
    load = constrain(load, 0, 1);
    // Write the load value to the PWM pin
    pwm_value = (PWMPeriod)*load ;
    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);


    if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        debug_output(); // print debug output to serial out
        last_debug_output = millis();
    }
}
