#include "AVR_PWM.h"

    // Pin and voltage definitions
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    static const uint8_t TEMP_PIN = A2;  // Added temperature sensing
    
    // Voltage ranges and references
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    static constexpr float TEMP_REFERENCE = 25.0;  // Reference temperature in °C
    
    // Buck converter parameters
    static constexpr float L = 100e-6;     // Inductor value in H
    static constexpr float C_IN = 0.01;    // Input capacitor (10000uF)
    static constexpr float C_OUT = 470e-6; // Output capacitor
    static constexpr float R_L = 0.1;      // Inductor DCR at reference temp
    static constexpr float R_DS_ON = 0.1;  // MOSFET on-resistance at reference temp
    static constexpr float TC_R_L = 0.004; // Temperature coefficient for inductor DCR
    static constexpr float TC_R_DS = 0.006;// Temperature coefficient for MOSFET
    
    // Switching parameters
    static constexpr float F_SW = 14000.0;  // Switching frequency in Hz
    static constexpr float T_SW = 1.0/F_SW; // Switching period
    static constexpr float T_DEAD = 1.0e-6; // Dead time
    
    // Protection thresholds
    static constexpr float I_MAX = 10.0;    // Maximum current limit
    static constexpr float V_MAX = 35.0;    // Maximum voltage limit
    static constexpr float TEMP_MAX = 85.0; // Maximum temperature
    
    // Control parameters
    static constexpr float LEARNING_RATE = 0.0001;
    static constexpr float VOLTAGE_FACTOR = 0.80;
    static constexpr float MIN_DUTY = 0.00;
    static constexpr float MAX_DUTY = 0.95;
    
    // Timing constants
    static const unsigned long STARTUP_DELAY = 1000;
    static const unsigned long CALIBRATION_INTERVAL = 20000;
    static const unsigned long PROTECTION_CHECK_INTERVAL = 100;
    static const unsigned long DEBUG_SERIAL_OUTPUT_INTERVAL = 100;


class OptimizedBuckController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;
    
    // State variables
    float temperature =  TEMP_REFERENCE;
    float duty_cycle = 0;
    float source_voltage = 0;
    float OC_voltage = 0 ; 
    float target_voltage = 0; 
    float load_voltage = 0;
    float inductor_current = 0;
    float input_current = 0;
    float error = 0;
    
    // Impedance model
    struct ImpedanceModel {
        float R_source;    // Source internal resistance
        float R_inductor;  // Temperature-compensated inductor resistance
        float R_fet;       // Temperature-compensated FET resistance
        float X_L;         // Inductive reactance
        float X_C;         // Capacitive reactance
        float Z_total;     // Total impedance magnitude
    } impedance;
    
    // Protection state
    bool fault_condition = false;
    unsigned long last_protection_check = 0;
    // calibration state
    unsigned long last_calibration = 0;
    // debug out
    unsigned long last_debug_output = 0;

    
    void updateTemperatureCompensation() {
        float temp_diff = temperature -  TEMP_REFERENCE;
        impedance.R_inductor =  R_L * (1 +  TC_R_L * temp_diff);
        impedance.R_fet =  R_DS_ON * (1 +  TC_R_DS * temp_diff);
    }
    
    float calculateSwitchingLosses() {
        float P_sw = 0;
        // Switching losses
        P_sw += 0.5 * source_voltage * inductor_current *  F_SW * 
                ( T_DEAD +  T_SW * (duty_cycle + (1 - duty_cycle)));
        // Conduction losses
        P_sw += impedance.R_fet * sq(inductor_current) * duty_cycle;
        return P_sw;
    }
    
    void updateImpedanceModel() {
        float omega = 2 * PI *  F_SW;
        
        // Update reactances
        impedance.X_L = omega *  L;
        impedance.X_C = 1 / (omega *  C_IN);
        
        // Calculate total impedance including switching effects
        float Z_dc = impedance.R_source + impedance.R_inductor + impedance.R_fet * duty_cycle;
        float Z_ac = sqrt(sq(impedance.X_L - impedance.X_C) + sq(Z_dc));
        
        impedance.Z_total = Z_dc * (1 - duty_cycle) + Z_ac * duty_cycle;
    }
    
    bool checkProtection() {
        if (source_voltage >  V_MAX || 
            load_voltage >  V_MAX || 
            inductor_current >  I_MAX || 
            temperature >  TEMP_MAX) {
            fault_condition = true;
            setPWM(0); // Shutdown PWM
            return true;
        }
        return false;
    }
    
    void updateControl() {
        if (fault_condition) return;
        
        // Calculate target voltage considering losses
        float switching_losses = calculateSwitchingLosses();
        target_voltage =  VOLTAGE_FACTOR * OC_voltage * 
                             (1 - switching_losses / (source_voltage * input_current+0.0001));
        
        // Update error with impedance compensation
        float voltage_drop = impedance.Z_total * input_current;
        //error = (error + (target_voltage - (OC_voltage + voltage_drop))) / 2;
        error = (error+ (target_voltage-source_voltage))/2;
        // Update duty cycle with adaptive learning rate
        //float adaptive_rate =  LEARNING_RATE * (1 - duty_cycle * duty_cycle);
        //duty_cycle = fabs(duty_cycle + adaptive_rate * error);
        duty_cycle = constrain(duty_cycle -  LEARNING_RATE * error, 0, 1);
        //Serial.println(error);
        duty_cycle = constrain(duty_cycle,  MIN_DUTY,  MAX_DUTY);
        
        setPWM(duty_cycle);
    }
    
    void setPWM(float value) {
        uint16_t pwm_value = value * pwm_period;
        pwm_instance->setPWM( PWM_PIN,  F_SW, pwm_value);
    }
    
    float readVoltage(uint8_t pin, float range) {
        return analogRead(pin) * (range / 1023.0);
    }
    
    float readTemperature() {
        // Simple temperature reading - adjust based on your sensor
        return 25;
//        return (analogRead( TEMP_PIN) * (5.0 / 1023.0) - 0.5) * 100;
    }

public:
    OptimizedBuckController() : pwm_instance(nullptr), pwm_period(0) {}
    
    void begin() {
        Serial.begin(115200);
        pinMode( PWM_PIN, OUTPUT);
        
        // Soft start initialization
        delay( STARTUP_DELAY);
        
        pwm_instance = new AVR_PWM( PWM_PIN,  F_SW, 0);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }
        
        // Initial calibration
        calibrate();
    }
    
    void calibrate() {
        // Perform initial measurements
        setPWM(0);
        delay(1000);
        
        // Measure open circuit characteristics
        OC_voltage = readVoltage( SOURCE_PIN,  SOURCE_VOLTAGE_RANGE);
        temperature = readTemperature();
        
        // Update temperature compensation
        updateTemperatureCompensation();
        
        // Start with safe duty cycle
        duty_cycle = 0.1;
        fault_condition = false;
    }
    
    void update() {
        unsigned long now = millis();
        
        // Regular measurements
        source_voltage = readVoltage( SOURCE_PIN,  SOURCE_VOLTAGE_RANGE);
        load_voltage = readVoltage( LOAD_PIN,  LOAD_VOLTAGE_RANGE);
        temperature = readTemperature();
        
        // Update models
        updateTemperatureCompensation();
        updateImpedanceModel();
        
        // Protection check
        if (now - last_protection_check >=  PROTECTION_CHECK_INTERVAL) {
            checkProtection();
            last_protection_check = now;
        }
              // calibration 
        if (now - last_calibration >=  CALIBRATION_INTERVAL) {
            calibrate();
            last_calibration = now;
        }
        
        // Normal operation
        if (!fault_condition) {
            updateControl();
        }
        
        // Debug output
        if (millis() - last_debug_output >  DEBUG_SERIAL_OUTPUT_INTERVAL) {
            printDebugInfo();
            last_debug_output = millis();
        }
    }
    
    void printDebugInfo() {
        Serial.print(F("V_OC:")); Serial.print(OC_voltage);
        Serial.print(F(",V_target:")); Serial.print(target_voltage);
        Serial.print(F(",V_src:")); Serial.print(source_voltage);
        Serial.print(F(",V_load:")); Serial.print(load_voltage);
        Serial.print(F(",I_L:")); Serial.print(inductor_current);
        Serial.print(F(",Duty:")); Serial.print(duty_cycle);
        Serial.print(F(",Temp:")); Serial.print(temperature);
        Serial.print(F(",Fault:")); Serial.println(fault_condition);
    }
};

OptimizedBuckController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
