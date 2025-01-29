#include <Arduino.h>

// Configuration structure to hold all settings
// Pin definitions - Adjust according to STM8 pinout
#define PWM_PIN           23    // Adjust based on available PWM pins on STM8
#define LOAD_PIN         5    // Analog input pin
#define SOURCE_PIN       4    // Analog input pin

// Voltage ranges
#define LOAD_VOLTAGE_RANGE    30.0f
#define SOURCE_VOLTAGE_RANGE  28.0f

// PWM limits
#define MAX_PWM              99.0f
#define MIN_PWM              0.0f
#define FREQ_MAX            14000.0f
#define MAX_DUTY         255

// Timing constants (in milliseconds)
#define CALIBRATION_INTERVAL                20000UL
#define OPEN_CIRCUIT_MEASUREMENT_TIME       1000UL
#define CALIBRATION_DURATION                3000UL
#define RINT_CALIBRATION_MEASUREMENT_TIME   20UL
#define DEBUG_SERIAL_OUTPUT_INTERVAL        100UL

// Control factors
#define LOAD_FACTOR          0.001f
#define CALIBRATION_FACTOR   0.0001f
#define VOLTAGE_FACTOR       0.80f
#define LEARNING_RATE        0.001f
#define LEARNING_RATE_LOAD   0.001f

// Global variables for state
static float open_circuit_voltage = 0;
static float open_circuit_load_voltage = 0;
static float internal_resistance_src = 0;
static float internal_resistance_load = 0;
static float load_resistance = 0;
static float source_to_load_resistance = 0;

// Measurement variables
static float output_voltage = 0;
static float output_current = 0;
static float input_current = 0;
static float output_power = 0;
static float source_voltage = 0;
static float load_voltage = 0;
static float corrected_voltage = 0;
static float error = 0;
static float load = 0;
static uint16_t pwm_value = 0;

// Timing variables
static unsigned long last_calibration_time = CALIBRATION_INTERVAL;
static unsigned long calibration_start_time = 0;
static unsigned long last_debug_output = 0;
static bool is_calibrating = false;

// Helper functions
float readVoltage(uint8_t pin, float voltage_range) {
    return analogRead(pin) * (voltage_range / 1023.0f);
}

void updateMeasurements(void) {
    source_voltage = readVoltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
    load_voltage = readVoltage(LOAD_PIN, LOAD_VOLTAGE_RANGE);
}

void calculateResistances(void) {
    load_resistance = (load_voltage / ((source_voltage - load_voltage)/(1-load) + 0.001f));
    source_to_load_resistance = (source_voltage - load_voltage) / ((1-load) + 0.001f);
    internal_resistance_load = (load_voltage - open_circuit_load_voltage) / ((1-load) + 0.001f);
    load_resistance = (load_voltage / ((1-load) + 0.001f)) - internal_resistance_load;
}

void calculateCurrentsAndPower(void) {
    output_current = load_voltage / (load_resistance + internal_resistance_load + 0.001f);
    input_current = source_voltage / (load_resistance + source_to_load_resistance + 0.001f);
    output_power = output_current * load_voltage;
}

void updateInternalResistance(void) {
    float new_resistance = (open_circuit_voltage - source_voltage) /
        (source_voltage / ((source_to_load_resistance + load_resistance + internal_resistance_load) + 0.001f));
    internal_resistance_src = internal_resistance_src * 0.9f + 0.1f * new_resistance;
}

void adjustLoad(void) {
    corrected_voltage = open_circuit_voltage - input_current * internal_resistance_src;
    error = (error + (VOLTAGE_FACTOR * open_circuit_voltage - corrected_voltage)) / 2;
    
    // Constrain load between 0 and 1
    float new_load = load - LEARNING_RATE_LOAD * error;
    load = new_load < 0 ? 0 : (new_load > 1 ? 1 : new_load);
    
    // Calculate PWM value and set it
    pwm_value = (uint16_t)(MAX_DUTY * load);
    analogWrite(PWM_PIN, pwm_value);
}

void calibrate(void) {
    analogWrite(PWM_PIN, 0);  // Set PWM to minimum
    delay(OPEN_CIRCUIT_MEASUREMENT_TIME);
    
    open_circuit_voltage = readVoltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
    open_circuit_load_voltage = readVoltage(LOAD_PIN, LOAD_VOLTAGE_RANGE);
    
    last_calibration_time = millis();
    is_calibrating = true;
    calibration_start_time = millis();
}

void handleCalibration(void) {
    if (millis() - calibration_start_time > CALIBRATION_DURATION) {
        is_calibrating = false;
        return;
    }
    
    // Constrain load between 0 and 1
    float new_load = load + CALIBRATION_FACTOR;
    load = new_load < 0 ? 0 : (new_load > 1 ? 1 : new_load);
    
    pwm_value = (uint16_t)(FREQ_MAX * load);
    analogWrite(PWM_PIN, pwm_value);
    
    delay(RINT_CALIBRATION_MEASUREMENT_TIME);
    updateMeasurements();
    calculateResistances();
    calculateCurrentsAndPower();
}

void printDebugInfo(void) {
    Serial_print_s("V_oc:");
    Serial_print_f(open_circuit_voltage);
    Serial_print_s(",V_cor:");
    Serial_print_f(corrected_voltage);
    Serial_print_s(",V_src:");
    Serial_print_f(source_voltage);
    Serial_print_s(",V_load:");
    Serial_print_f(load_voltage);
    Serial_print_s(",R_int:");
    Serial_print_f(internal_resistance_src);
    Serial_print_s(",load:");
    Serial_println_f(load);
}

void setup() {
    // Initialize serial communication
    Serial_begin(115200);
    
    // Configure pins
    pinMode(PWM_PIN, OUTPUT);
    
    // Initialize PWM
    analogWrite(PWM_PIN, 0);
}

void loop() {
    if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
        calibrate();
    }
    
    if (is_calibrating) {
        handleCalibration();
    } else {
        updateMeasurements();
        calculateResistances();
        calculateCurrentsAndPower();
        updateInternalResistance();
        adjustLoad();
    }
    
    if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
        printDebugInfo();
        last_debug_output = millis();
    }
}
