#include "AVR_PWM.h"

struct Config {
    // Pin definitions
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    
    // Voltage ranges
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    
    // PWM parameters
    static constexpr float MAX_PWM = 99.0;
    static constexpr float MIN_PWM = 0.0;
    static constexpr float FREQ_MAX = 14000.0;
    
    // Component parameters
    static constexpr float INPUT_CAP = 0.01;      // 10000uF in Farads
    static constexpr float INPUT_CAP_ESR = 0.05;  // Capacitor ESR in ohms
    static constexpr float INPUT_CAP_ESL = 20e-9; // Capacitor ESL in henries
    
    // Timing
    static const unsigned long CALIBRATION_INTERVAL = 20000;
    static const unsigned long MEASUREMENT_DELAY = 1000;
    static const unsigned long DEBUG_INTERVAL = 100;
    
    // Gradient descent parameters
    static constexpr float LEARNING_RATE = 0.001;
    static constexpr float LEARNING_RATE_LOAD = 0.001;
    static constexpr float VOLTAGE_FACTOR = 0.80;
};

class ImpedanceController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;
    
    // State variables
    float open_circuit_voltage = 0;
    float source_voltage = 0;
    float load_voltage = 0;
    float cap_voltage = 0;
    
    // Complex impedance components
    struct ComplexImpedance {
        float resistance;    // Real part
        float reactance;     // Imaginary part
        float magnitude;     // |Z|
        float phase;         // θ in radians
    };
    
    ComplexImpedance Z_source;  // Source internal impedance
    ComplexImpedance Z_load;    // Load impedance
    ComplexImpedance Z_cap;     // Input capacitor impedance
    
    // Control variables
    float load = 0;
    float pwm_value = 0;
    float error = 0;
    
    // Timing
    unsigned long last_calibration = 0;
    unsigned long last_debug = 0;

    void calculateCapacitorImpedance() {
        float omega = 2 * PI * Config::FREQ_MAX;
        
        // Capacitive reactance
        float Xc = -1.0 / (omega * Config::INPUT_CAP);
        
        // Inductive reactance of ESL
        float Xl = omega * Config::INPUT_CAP_ESL;
        
        // Total reactance
        Z_cap.reactance = Xc + Xl;
        Z_cap.resistance = Config::INPUT_CAP_ESR;
        
        // Calculate magnitude and phase
        Z_cap.magnitude = sqrt(sq(Z_cap.resistance) + sq(Z_cap.reactance));
        Z_cap.phase = atan2(Z_cap.reactance, Z_cap.resistance);
    }
    
    void estimateSourceImpedance() {
        // Get voltage and current measurements
        float v1 = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        float i1 = (v1 - load_voltage) / Z_load.magnitude;
        
        // Change load slightly
        float temp_load = load + 0.1;
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, temp_load * pwm_period);
        delay(Config::MEASUREMENT_DELAY);
        
        // Get second set of measurements
        float v2 = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        float i2 = (v2 - load_voltage) / Z_load.magnitude;
        
        // Calculate impedance components
        float delta_v = v2 - v1;
        float delta_i = i2 - i1;
        
        // Real component (resistance)
        Z_source.resistance = (delta_v * delta_i) / (sq(delta_i));
        
        // Imaginary component (reactance)
        // Using phase difference between voltage and current
        float phase_diff = acos(Z_source.resistance / sqrt(sq(delta_v/delta_i)));
        Z_source.reactance = (delta_v/delta_i) * sin(phase_diff);
        
        // Calculate magnitude and phase
        Z_source.magnitude = sqrt(sq(Z_source.resistance) + sq(Z_source.reactance));
        Z_source.phase = atan2(Z_source.reactance, Z_source.resistance);
        
        // Restore original load
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, load * pwm_period);
    }
    
    void updateLoadImpedance() {
        float current = load_voltage / Z_load.magnitude;
        float power = load_voltage * current;
        
        // Calculate total circuit impedance including capacitor
        float total_impedance = Z_source.magnitude + Z_cap.magnitude + Z_load.magnitude;
        
        // Calculate optimal load impedance for maximum power transfer
        float optimal_impedance = sqrt(sq(Z_source.magnitude) + sq(Z_cap.magnitude));
        
        // Calculate gradient for impedance matching
        float gradient = (power - last_power) / (Z_load.magnitude - last_impedance);
        
        // Update load impedance using gradient descent
        Z_load.magnitude += Config::LEARNING_RATE * gradient;
        
        // Store values for next iteration
        last_power = power;
        last_impedance = Z_load.magnitude;
    }
    
    void gradientDescentControl() {
        // Calculate voltage error considering complex impedances
        float expected_voltage = open_circuit_voltage * 
            (Z_load.magnitude / (Z_source.magnitude + Z_cap.magnitude + Z_load.magnitude));
        
        error = (error + (Config::VOLTAGE_FACTOR * expected_voltage - load_voltage)) / 2;
        
        // Update load using gradient descent
        load = load - Config::LEARNING_RATE_LOAD * error;
        load = constrain(load, 0, 1);
        
        // Update PWM
        pwm_value = pwm_period * load;
        pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, pwm_value);
    }
    
    float readVoltage(uint8_t pin, float range) {
        return analogRead(pin) * (range / 1023.0);
    }
    
    // Storage for gradient descent
    float last_power = 0;
    float last_impedance = 0;

public:
    ImpedanceController() : pwm_instance(nullptr), pwm_period(0) {}
    
    void begin() {
        Serial.begin(115200);
        pinMode(Config::PWM_PIN, OUTPUT);
        
        pwm_instance = new AVR_PWM(Config::PWM_PIN, Config::FREQ_MAX, Config::MIN_PWM);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }
        
        // Initialize impedance calculations
        calculateCapacitorImpedance();
    }
    
    void update() {
        unsigned long now = millis();
        
        // Regular measurements
        source_voltage = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
        load_voltage = readVoltage(Config::LOAD_PIN, Config::LOAD_VOLTAGE_RANGE);
        
        // Periodic calibration
        if (now - last_calibration >= Config::CALIBRATION_INTERVAL) {
            pwm_instance->setPWM(Config::PWM_PIN, Config::FREQ_MAX, 0);
            delay(Config::MEASUREMENT_DELAY);
            
            open_circuit_voltage = readVoltage(Config::SOURCE_PIN, Config::SOURCE_VOLTAGE_RANGE);
            estimateSourceImpedance();
            
            last_calibration = now;
        }
        
        // Update control
        updateLoadImpedance();
        gradientDescentControl();
        
        // Debug output
        if (now - last_debug >= Config::DEBUG_INTERVAL) {
            printDebugInfo();
            last_debug = now;
        }
    }
    
    void printDebugInfo() {
        Serial.print(F("Z_src_R:")); Serial.print(Z_source.resistance);
        Serial.print(F(",Z_src_X:")); Serial.print(Z_source.reactance);
        Serial.print(F(",Z_load_mag:")); Serial.print(Z_load.magnitude);
        Serial.print(F(",V_oc:")); Serial.print(open_circuit_voltage);
        Serial.print(F(",V_load:")); Serial.print(load_voltage);
        Serial.print(F(",Error:")); Serial.print(error);
        Serial.print(F(",Load:")); Serial.println(load);
    }
};

ImpedanceController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
