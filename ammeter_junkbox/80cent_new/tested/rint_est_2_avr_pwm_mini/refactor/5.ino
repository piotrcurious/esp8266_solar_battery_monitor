#include "AVR_PWM.h"

struct Config {
    // Pin and voltage definitions
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    
    // Buck converter parameters
    static constexpr float L = 100e-6;    // Inductor value in H
    static constexpr float C_IN = 0.01;   // Input capacitor (10000uF)
    static constexpr float R_L = 0.1;     // Inductor DCR
    static constexpr float F_SW = 14000.0; // Switching frequency in Hz
    
    // Calibration timing
    static const unsigned long CALIBRATION_INTERVAL = 20000;
    static const unsigned long OPEN_CIRCUIT_MEASUREMENT_TIME = 1000;
    static const unsigned long CALIBRATION_DURATION = 3000;
    static const unsigned long RINT_MEASUREMENT_TIME = 20;
    
    // Control parameters
    static constexpr float LEARNING_RATE = 0.001;
    static constexpr float LEARNING_RATE_LOAD = 0.001;
    static constexpr float VOLTAGE_FACTOR = 0.80;  // Applied to Vin
    static constexpr float CALIBRATION_FACTOR = 0.0001;
};

class BuckController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;
    
    // State variables for Voc and Rint
    float open_circuit_voltage = 0;
    float open_circuit_load_voltage = 0;
    float internal_resistance_src = 0;
    float internal_resistance_load = 0;
    
    // Operating point variables
    float source_voltage = 0;
    float load_voltage = 0;
    float inductor_current = 0;
    float input_current = 0;
    float duty_cycle = 0;
    float error = 0;
    
    // Timing variables
    unsigned long last_calibration_time = 0;
    unsigned long calibration_start_time = 0;
    bool is_calibrating = false;

    // Switching-aware impedance estimation
    float calculateEffectiveRint() {
        float T = 1.0f / Config::F_SW;
        float D = duty_cycle;
        
        // Calculate RMS currents considering switching
        float i_rms_input = input_current * sqrt(D);
        float di_L = (source_voltage * D * (1-D) * T) / Config::L;
        float i_ripple_rms = di_L / (2 * sqrt(3));
        
        // Total RMS current including ripple
        float i_total_rms = sqrt(sq(i_rms_input) + sq(i_ripple_rms));
        
        // Calculate effective resistance including switching effects
        float v_drop = open_circuit_voltage - source_voltage;
        return v_drop / i_total_rms;
    }
    
    void performCalibration() {
        // First measure open circuit voltages
        setPWM(0);  // Turn off PWM
        delay(Config::OPEN_CIRCUIT_MEASUREMENT_TIME);
        
        open_circuit_voltage = readVoltage(Config::SOURCE_PIN);
        open_circuit_load_voltage = readVoltage(Config::LOAD_PIN);
        
        // Now perform stepped load measurements for Rint calibration
        float prev_current = 0;
        float prev_voltage = open_circuit_voltage;
        
        for(float load = 0; load <= 0.3; load += Config::CALIBRATION_FACTOR) {
            setPWM(load);
            delay(Config::RINT_MEASUREMENT_TIME);
            
            source_voltage = readVoltage(Config::SOURCE_PIN);
            load_voltage = readVoltage(Config::LOAD_PIN);
            
            // Calculate current through the inductor
            float L_current = load_voltage / (load_resistance + Config::R_L);
            // Input current depends on duty cycle
            input_current = L_current * duty_cycle;
            
            if(input_current > prev_current) {
                float delta_v = prev_voltage - source_voltage;
                float delta_i = input_current - prev_current;
                
                // Update internal resistance estimate with switching effects
                float rint_sample = calculateEffectiveRint();
                internal_resistance_src = internal_resistance_src * 0.9 + rint_sample * 0.1;
                
                prev_current = input_current;
                prev_voltage = source_voltage;
            }
        }
        
        is_calibrating = false;
    }
    
    void updateControl() {
        // Calculate target input voltage based on Voc and voltage factor
        float target_vin = open_circuit_voltage * Config::VOLTAGE_FACTOR;
        
        // Calculate error based on input voltage
        error = (error + (target_vin - source_voltage)) / 2;
        
        // Consider switching losses in gradient calculation
        float switching_loss_factor = 1.0 - (duty_cycle * (1 - duty_cycle) * Config::F_SW * Config::L) / 
                                    (2 * source_voltage);
        
        // Update duty cycle using gradient descent
        float gradient = error * switching_loss_factor;
        duty_cycle = duty_cycle - Config::LEARNING_RATE_LOAD * gradient;
        duty_cycle = constrain(duty_cycle, 0.0f, 0.95f);
        
        setPWM(duty_cycle);
    }
    
    void setPWM(float value) {
        uint16_t pwm_value = value * pwm_period;
        pwm_instance->setPWM(Config::PWM_PIN, Config::F_SW, pwm_value);
    }
    
    float readVoltage(uint8_t pin) {
        return analogRead(pin) * (Config::SOURCE_VOLTAGE_RANGE / 1023.0);
    }
    
    float load_resistance = 10.0; // Default load resistance

public:
    BuckController() : pwm_instance(nullptr), pwm_period(0) {}
    
    void begin() {
        Serial.begin(115200);
        pinMode(Config::PWM_PIN, OUTPUT);
        
        pwm_instance = new AVR_PWM(Config::PWM_PIN, Config::F_SW, 0);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }
    }
    
    void update() {
        unsigned long now = millis();
        
        // Check if calibration is needed
        if (now - last_calibration_time >= Config::CALIBRATION_INTERVAL) {
            is_calibrating = true;
            calibration_start_time = now;
            performCalibration();
            last_calibration_time = now;
            return;
        }
        
        // Normal operation
        if (!is_calibrating) {
            source_voltage = readVoltage(Config::SOURCE_PIN);
            load_voltage = readVoltage(Config::LOAD_PIN);
            
            updateControl();
            
            // Debug output
            printDebugInfo();
        }
    }
    
    void printDebugInfo() {
        Serial.print(F("V_oc:")); Serial.print(open_circuit_voltage);
        Serial.print(F(",V_src:")); Serial.print(source_voltage);
        Serial.print(F(",V_load:")); Serial.print(load_voltage);
        Serial.print(F(",R_int:")); Serial.print(internal_resistance_src);
        Serial.print(F(",Duty:")); Serial.print(duty_cycle);
        Serial.print(F(",Error:")); Serial.println(error);
    }
};

BuckController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
