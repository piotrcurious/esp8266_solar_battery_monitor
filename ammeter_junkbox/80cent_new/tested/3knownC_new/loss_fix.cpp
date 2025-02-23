#include "AVR_PWM.h"
#include <math.h>   // for exp(), pow()
#include <Arduino.h>

AVR_PWM* PWM_Instance;
float frequency;
unsigned long last_debug_output = 0;
#define DEBUG_SERIAL_OUTPUT_INTERVAL 1000  // Debug output interval (ms)

// Define the pins
#define PWM_PIN 9           // PWM output to the load
#define LOAD_PIN A1         // Analog input for load voltage
#define LOAD_VOLTAGE_RANGE 30.0  // Volts corresponding to maximum analog input for load
#define SOURCE_PIN A0       // Analog input for source voltage (solar panel)
#define SOURCE_VOLTAGE_RANGE 28.0 // Volts corresponding to maximum analog input for source

// Constants and control parameters
#define MAX_PWM 99.0
#define MIN_PWM 0.0
uint16_t PWMPeriod = 0;
#define FREQ_MAX 14000.0

#define LEARNING_RATE_LOAD 0.01

// Weights for the loss function error components
#define ALPHA 0.5   // weight for steady-state error
#define BETA  0.3   // weight for RC dynamic error
#define GAMMA 0.2   // weight for internal resistance error

#define DT 100.0    // Time delay (ms) used to evaluate the RC response

// Global state variables (assumed to be updated by calibration elsewhere)
float open_circuit_voltage = 28.0;  // Fitted open-circuit voltage (Voc)
float fitted_tau = 1500.0;          // Fitted RC time constant (τ) in ms
float fitted_Rint = 2.0;            // Fitted internal resistance in ohms

// Variables for live measurements
float source_voltage = 0.0;
float load_voltage = 0.0;
float estimated_rint = 0.0;
float load = 0.5;  // Normalized load value (0 to 1)

// This function computes the loss based on three error components and adjusts the load accordingly.
void update_pwm_based_on_loss() {
    // 1. Steady-State Error: target is 80% of the fitted Voc.
    float target_voltage = 0.8 * open_circuit_voltage;
    float steady_state_error = target_voltage - source_voltage;
    
    // 2. RC Dynamic Error: predicted voltage from the RC model after time DT.
    // For an RC circuit: V(t) = Voc * (1 - exp(-t/τ))
    float predicted_voltage = open_circuit_voltage * (1 - exp(-DT / fitted_tau));
    float rc_response_error = predicted_voltage - source_voltage;
    
    // 3. Internal Resistance Error: difference between estimated Rint and the fitted Rint.
    estimated_rint = (source_voltage - load_voltage) / (load_voltage / LOAD_VOLTAGE_RANGE + 0.001);
    float rint_error = estimated_rint - fitted_Rint;
    
    // Combine the errors (squared) with their respective weights.
    float loss = ALPHA * (steady_state_error * steady_state_error) +
                 BETA  * (rc_response_error * rc_response_error) +
                 GAMMA * (rint_error * rint_error);
    
    // Adjust the load parameter to reduce the loss.
    load = load - LEARNING_RATE_LOAD * loss;
    load = constrain(load, 0, 1);
    
    // Calculate PWM value based on the updated load.
    float pwm_value = PWMPeriod * load;
    PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);
}

void loop() {
    // Update measured voltages.
    load_voltage = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    
    // NOTE: The RC fitting/calibration routine should run periodically and update:
    // open_circuit_voltage, fitted_tau, and fitted_Rint.
    
    // Update PWM control based on the improved loss function.
    update_pwm_based_on_loss();
    
    // Debug output for monitoring.
    if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        Serial.print(F(\"V_oc: \")); Serial.print(open_circuit_voltage);
        Serial.print(F(\", V_src: \")); Serial.print(source_voltage);
        Serial.print(F(\", V_load: \")); Serial.print(load_voltage);
        Serial.print(F(\", τ: \")); Serial.print(fitted_tau);
        Serial.print(F(\", R_int (fitted): \")); Serial.print(fitted_Rint);
        Serial.print(F(\", R_int (est): \")); Serial.println(estimated_rint);
        last_debug_output = millis();
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, MIN_PWM);
    
    frequency = FREQ_MAX;
    PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);
    if (PWM_Instance) {
        PWM_Instance->setPWM();
    }
    PWMPeriod = PWM_Instance->getPWMPeriod();
    
    // Initialize calibration parameters (replace these with your calibration results)
    open_circuit_voltage = 28.0;
    fitted_tau = 1500.0;  // Example time constant in ms
    fitted_Rint = 2.0;    // Example internal resistance in ohms
          }
