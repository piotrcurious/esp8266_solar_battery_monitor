#include "AVR_PWM.h"

struct Config {
    // Pin and voltage definitions (as before)
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    
    // Buck converter parameters
    static constexpr float L = 100e-6;    // Inductor value in H
    static constexpr float C_IN = 0.01;   // Input capacitor (10000uF)
    static constexpr float C_OUT = 470e-6; // Output capacitor
    static constexpr float R_L = 0.1;     // Inductor DCR
    static constexpr float R_DS_ON = 0.1; // MOSFET on-resistance
    static constexpr float R_D = 0.05;    // Diode forward resistance
    static constexpr float V_D = 0.7;     // Diode forward voltage
    
    // Switching parameters
    static constexpr float F_SW = 14000.0;        // Switching frequency in Hz
    static constexpr float T_SW = 1.0/F_SW;       // Switching period
    static constexpr float T_DEAD = 1.0e-6;       // Dead time
    static constexpr float T_RISE = 100.0e-9;     // Rise time
    static constexpr float T_FALL = 100.0e-9;     // Fall time
    
    // Control parameters
    static constexpr float LEARNING_RATE = 0.001;
    static constexpr float VOLTAGE_FACTOR = 0.80;
};

class SwitchingBuckController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;
    
    // State variables
    float duty_cycle = 0;
    float v_in = 0;
    float v_out = 0;
    float i_L = 0;
    float i_L_peak = 0;
    float i_L_valley = 0;
    float v_c_in = 0;
    float i_c_in = 0;
    float error = 0;
    
    struct SwitchingImpedance {
        // Fundamental components
        float R_eq;     // Equivalent resistance
        float L_eq;     // Equivalent inductance
        float C_eq;     // Equivalent capacitance
        
        // Frequency-dependent components
        float Z_sw;     // Switching frequency impedance
        float phi_sw;   // Phase at switching frequency
        
        // Harmonic components
        static const int N_HARMONICS = 5;
        float Z_h[N_HARMONICS];    // Harmonic impedances
        float phi_h[N_HARMONICS];  // Harmonic phases
    };
    
    SwitchingImpedance Z_in;   // Input impedance
    SwitchingImpedance Z_out;  // Output impedance

    // Calculate switching ripple and average values
    void calculateSwitchingRipple() {
        float D = duty_cycle;
        float T = Config::T_SW;
        float L = Config::L;
        float V_in = v_in;
        float V_out = v_out;
        
        // Inductor current ripple
        float di_L = (V_in * D * (1-D) * T) / L;
        i_L_peak = i_L + di_L/2;
        i_L_valley = i_L - di_L/2;
        
        // Input capacitor current ripple
        float i_in_avg = i_L * D;
        i_c_in = i_in_avg - i_L * D; // Capacitor current is difference between average and instantaneous
        
        // Input capacitor voltage ripple
        float dv_c_in = (i_c_in * D * T) / Config::C_IN;
        
        // Update equivalent impedances based on ripple
        updateSwitchingImpedances(D, di_L, dv_c_in);
    }
    
    void updateSwitchingImpedances(float D, float di_L, float dv_c_in) {
        float omega_sw = 2 * PI * Config::F_SW;
        
        // Calculate effective duty-cycle dependent resistance
        Z_in.R_eq = Config::R_DS_ON * D + Config::R_D * (1-D) + Config::R_L;
        
        // Calculate switching frequency impedance
        float X_L = omega_sw * Config::L;
        float X_C = 1 / (omega_sw * Config::C_IN);
        
        // Input impedance at switching frequency
        Z_in.Z_sw = sqrt(sq(Z_in.R_eq) + sq(X_L - X_C));
        Z_in.phi_sw = atan2(X_L - X_C, Z_in.R_eq);
        
        // Calculate harmonic impedances
        for(int n = 1; n <= SwitchingImpedance::N_HARMONICS; n++) {
            float omega_h = n * omega_sw;
            float X_L_h = omega_h * Config::L;
            float X_C_h = 1 / (omega_h * Config::C_IN);
            
            // Harmonic impedance magnitude
            Z_in.Z_h[n-1] = sqrt(sq(Z_in.R_eq) + sq(X_L_h - X_C_h));
            // Harmonic phase
            Z_in.phi_h[n-1] = atan2(X_L_h - X_C_h, Z_in.R_eq);
        }
        
        // Calculate effective input impedance including switching effects
        float Z_eff = Z_in.Z_sw * (1 + 
            pow(di_L / i_L, 2) + // Current ripple effect
            pow(dv_c_in / v_c_in, 2)); // Voltage ripple effect
            
        // Update control parameters based on effective impedance
        updateControlParameters(Z_eff, D);
    }
    
    void updateControlParameters(float Z_eff, float D) {
        // Calculate power transfer considering switching effects
        float P_in = v_in * i_L * D;
        float P_out = v_out * (i_L * (1-D));
        float P_sw = P_in - P_out; // Switching losses
        
        // Calculate gradient considering switching losses
        float dP_dD = (P_in - P_sw) * (1 / Z_eff);
        
        // Update duty cycle using gradient descent
        float new_D = D + Config::LEARNING_RATE * dP_dD;
        duty_cycle = constrain(new_D, 0.0f, 0.95f);
        
        // Calculate error considering switching effects
        float v_out_desired = v_in * D * (1 - (Z_eff / (2 * Z_in.R_eq)));
        error = (Config::VOLTAGE_FACTOR * v_out_desired - v_out);
    }
    
    float readVoltage(uint8_t pin, float range) {
        return analogRead(pin) * (range / 1023.0);
    }

public:
    SwitchingBuckController() : pwm_instance(nullptr), pwm_period(0) {}
    
    void begin() {
        Serial.begin(115200);
        pinMode(Config::PWM_PIN, OUTPUT);
        
        pwm_instance = new AVR_PWM(Config::PWM_PIN, Config::F_SW, 0);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }
        
        // Initialize with safe duty cycle
        duty_cycle = 0.5;
        updatePWM();
    }
    
    void update() {
        // Read voltages
        v_in = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        v_out = readVoltage(Config::LOAD_PIN, Config::LOAD_VOLTAGE_RANGE);
        
        // Update switching model
        calculateSwitchingRipple();
        
        // Update PWM
        updatePWM();
        
        // Debug output
        printDebugInfo();
    }
    
    void updatePWM() {
        uint16_t pwm_value = duty_cycle * pwm_period;
        pwm_instance->setPWM(Config::PWM_PIN, Config::F_SW, pwm_value);
    }
    
    void printDebugInfo() {
        Serial.print(F("D:")); Serial.print(duty_cycle);
        Serial.print(F(",I_L:")); Serial.print(i_L);
        Serial.print(F(",I_Lpk:")); Serial.print(i_L_peak);
        Serial.print(F(",I_Lvl:")); Serial.print(i_L_valley);
        Serial.print(F(",Z_sw:")); Serial.print(Z_in.Z_sw);
        Serial.print(F(",V_in:")); Serial.print(v_in);
        Serial.print(F(",V_out:")); Serial.println(v_out);
    }
};

SwitchingBuckController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
