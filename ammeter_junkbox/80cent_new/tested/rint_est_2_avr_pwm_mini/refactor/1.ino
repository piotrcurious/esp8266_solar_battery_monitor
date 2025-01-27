#include "AVR_PWM.h"

// Configuration structure to hold all settings
struct Config {
    // Pin definitions
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    
    // Voltage ranges
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    
    // PWM limits
    static constexpr float MAX_PWM = 99.0;
    static constexpr float MIN_PWM = 0.0;
    static constexpr float FREQ_MAX = 14000.0;
    
    // Timing constants (in milliseconds)
    static const unsigned long CALIBRATION_INTERVAL = 20000;
    static const unsigned long OPEN_CIRCUIT_MEASUREMENT_TIME = 1000;
    static const unsigned long CALIBRATION_DURATION = 3000;
    static const unsigned long RINT_CALIBRATION_MEASUREMENT_TIME = 20;
    static const unsigned long DEBUG_SERIAL_OUTPUT_INTERVAL = 100;
    
    // Control factors
    static constexpr float LOAD_FACTOR = 0.001;
    static constexpr float CALIBRATION_FACTOR = 0.0001;
    static constexpr float VOLTAGE_FACTOR = 0.80;
    static constexpr float LEARNING_RATE = 0.001;
    static constexpr float LEARNING_RATE_LOAD = 0.001;
};

class PowerController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;
    
    // State variables
    float open_circuit_voltage = 0;
    float open_circuit_load_voltage = 0;
    float internal_resistance_src = 0;
    float internal_resistance_load = 0;
    float load_resistance = 0;
    float source_to_load_resistance = 0;
    
    // Measurement variables
    float output_voltage = 0;
    float output_current = 0;
    float input_current = 0;
    float output_power = 0;
    float source_voltage = 0;
    float load_voltage = 0;
    float corrected_voltage = 0;
    float error = 0;
    float load = 0;
    float pwm_value = 0;
    
    // Timing variables
    unsigned long last_calibration_time = Config::CALIBRATION_INTERVAL;
    unsigned long calibration_start_time = 0;
    unsigned long last_debug_output = 0;
    bool is_calibrating = false;

    void updateMeasurements() {
        source_voltage = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        load_voltage = readVoltage(Config::LOAD_PIN, Config::LOAD_VOLTAGE_RANGE);
    }
    
    float readVoltage(uint8_t pin, float voltage_range) {
        return analogRead(pin) * (voltage_range / 1023.0);
    }
    
    void calculateResistances() {
        load_resistance = (load_voltage / ((source_voltage - load_voltage)/(1-load) + 0.001));
        source_to_load_resistance = (source_voltage - load_voltage) / ((1-load) + 0.001);
        internal_resistance_load = (load_voltage - open_circuit_load_voltage) / ((1-load) + 0.001);
        load_resistance = (load_voltage / ((1-load) + 0.001)) - internal_resistance_load;
    }
    
    void calculateCurrentsAndPower() {
        output_current = load_voltage / (load_resistance + internal_resistance_load + 0.001);
        input_current = source_voltage / (load_resistance + source_to_load_resistance + 0.001);
        output_power = output_current * load_voltage;
    }
    
    void updateInternalResistance() {
        float new_resistance = (open_circuit_voltage - source_voltage) /
            (source_voltage / ((source_to_load_resistance + load_resistance + internal_resistance_load) + 0.001));
        internal_resistance_src = internal_resistance_src * 0.9 + 0.1 * new_resistance;
    }
    
    void adjustLoad() {
        corrected_voltage = open_circuit_voltage - input_current * internal_resistance_src;
        error = (error + (Config::VOLTAGE_FACTOR * open_circuit_voltage - corrected_voltage)) / 2;
        load = constrain(load - Config::LEARNING_RATE_LOAD * error, 0, 1);
        pwm_value = pwm_period * load;
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, pwm_value);
    }

public:
    PowerController() : pwm_instance(nullptr), pwm_period(0) {}
    
    void begin() {
        Serial.begin(115200);
        pinMode(Config::PWM_PIN, OUTPUT);
        
        pwm_instance = new AVR_PWM(Config::PWM_PIN, Config::FREQ_MAX, Config::MIN_PWM);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }
    }
    
    void calibrate() {
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, Config::MIN_PWM);
        delay(Config::OPEN_CIRCUIT_MEASUREMENT_TIME);
        
        open_circuit_voltage = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        open_circuit_load_voltage = readVoltage(Config::LOAD_PIN, Config::LOAD_VOLTAGE_RANGE);
        
        last_calibration_time = millis();
        is_calibrating = true;
        calibration_start_time = millis();
    }
    
    void update() {
        if (millis() - last_calibration_time > Config::CALIBRATION_INTERVAL) {
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
        
        if (millis() - last_debug_output > Config::DEBUG_SERIAL_OUTPUT_INTERVAL) {
            printDebugInfo();
            last_debug_output = millis();
        }
    }
    
    void handleCalibration() {
        if (millis() - calibration_start_time > Config::CALIBRATION_DURATION) {
            is_calibrating = false;
            return;
        }
        
        load = constrain(load + Config::CALIBRATION_FACTOR, 0, 1);
        pwm_value = (pwm_period - 1) * load;
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, pwm_value);
        
        delay(Config::RINT_CALIBRATION_MEASUREMENT_TIME);
        updateMeasurements();
        calculateResistances();
        calculateCurrentsAndPower();
    }
    
    void printDebugInfo() {
        Serial.print(F("V_oc:"));
        Serial.print(open_circuit_voltage);
        Serial.print(F(",V_cor:"));
        Serial.print(corrected_voltage);
        Serial.print(F(",V_src:"));
        Serial.print(source_voltage);
        Serial.print(F(",V_load:"));
        Serial.print(load_voltage);
        Serial.print(F(",R_int:"));
        Serial.print(internal_resistance_src);
        Serial.print(F(",load:"));
        Serial.println(load, 3);
    }
};

PowerController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
