#include <Arduino.h>
#include <MovingAverage.h>  // Recommended external library for efficient moving average

// Advanced Configuration
struct SolarSystemConfig {
    // Configurable system parameters
    static constexpr float VOLTAGE_TARGET_RATIO = 0.80;
    static constexpr float TEMPERATURE_COEFFICIENT = -0.003;
    static constexpr float MAX_VOLTAGE_DECAY_RATE = 0.9995;
    static constexpr float CURRENT_ESTIMATION_THRESHOLD = 2.0;
    static constexpr unsigned long UPDATE_INTERVAL = 250;  // Increased frequency
};

class SolarMPPTController {
private:
    // Pin Configurations
    const int pwmPin1, pwmPin2;
    const int panelVoltagePin, outputVoltagePin1, outputVoltagePin2, temperaturePin;

    // Enhanced Tracking Variables
    float panelVoc = 0.0;
    float targetPanelVoltage = 0.0;
    float previousPower = 0.0;

    // Advanced PWM Control
    int dutyCycle1 = 0, dutyCycle2 = 0;
    static constexpr int MAX_DUTY = 255;

    // Adaptive PID Parameters
    float pidParams[3] = {1.5, 0.08, 0.03};  // Adaptive Kp, Ki, Kd
    float integralError = 0.0;
    float lastError = 0.0;

    // Moving Average Filters
    MovingAverage<float, 10> voltageFilter;
    MovingAverage<float, 5> currentFilter;

    // Enhanced Inference Parameters
    struct InferenceWeights {
        float dVdD = 0.4;
        float vocTrend = 0.3;
        float tempCompensation = 0.3;
    } weights;

    // Private Methods
    float readVoltage(int pin) {
        return analogRead(pin) * (5.0 / 1023.0);
    }

    float readTemperature() {
        return (readVoltage(temperaturePin) - 0.5) * 100.0;
    }

    void adaptivePIDTuning(float error) {
        // Dynamically adjust PID parameters based on error characteristics
        if (abs(error) > 0.5) {
            pidParams[0] *= 1.1;  // Increase proportional gain
            pidParams[1] *= 0.9;  // Reduce integral gain
        } else {
            pidParams[0] *= 0.95;
            pidParams[1] *= 1.05;
        }
    }

    void updateWeightingFactors() {
        float voltageRatio = readVoltage(panelVoltagePin) / panelVoc;
        
        if (voltageRatio < 0.7) {
            weights.dVdD = 0.6;
            weights.vocTrend = 0.2;
            weights.tempCompensation = 0.2;
        } else {
            weights.dVdD = 0.4;
            weights.vocTrend = 0.3;
            weights.tempCompensation = 0.3;
        }
    }

    float estimateAdvancedCurrent(float currentVoltage, float dutyCycle) {
        float dVdD = (previousPower - (currentVoltage * dutyCycle)) / dutyCycle;
        float vocTrend = (panelVoc - currentVoltage) / panelVoc;
        float tempEffect = readTemperature() * SolarSystemConfig::TEMPERATURE_COEFFICIENT;
        
        return (dVdD * weights.dVdD) + 
               (vocTrend * weights.vocTrend) + 
               (tempEffect * weights.tempCompensation);
    }

    void updateVocTracking(float panelVoltage) {
        // More robust Voc tracking with adaptive decay
        panelVoc = (panelVoltage > panelVoc) ? 
                    panelVoltage : 
                    (panelVoc * SolarSystemConfig::MAX_VOLTAGE_DECAY_RATE);
        
        targetPanelVoltage = panelVoc * SolarSystemConfig::VOLTAGE_TARGET_RATIO;
    }

    void performAdvancedPWMControl(float error) {
        integralError += error;
        float derivative = error - lastError;
        
        float pidOutput = (pidParams[0] * error) + 
                          (pidParams[1] * integralError) + 
                          (pidParams[2] * derivative);

        dutyCycle1 = constrain(dutyCycle1 + pidOutput, 0, MAX_DUTY);
        dutyCycle2 = constrain(dutyCycle2 + pidOutput, 0, MAX_DUTY);
        
        lastError = error;
        adaptivePIDTuning(error);
    }

    void intelligentLoadBalancing(float inferredCurrent) {
        if (inferredCurrent > SolarSystemConfig::CURRENT_ESTIMATION_THRESHOLD) {
            dutyCycle1 = constrain(dutyCycle1 + 15, 0, MAX_DUTY);
            dutyCycle2 = constrain(dutyCycle2 - 15, 0, MAX_DUTY);
        } else {
            dutyCycle1 = constrain(dutyCycle1 - 10, 0, MAX_DUTY);
            dutyCycle2 = constrain(dutyCycle2 + 10, 0, MAX_DUTY);
        }
    }

public:
    SolarMPPTController(
        int pwm1, int pwm2, 
        int panelVPin, int outVPin1, int outVPin2, int tempPin
    ) : 
        pwmPin1(pwm1), pwmPin2(pwm2),
        panelVoltagePin(panelVPin), 
        outputVoltagePin1(outVPin1), 
        outputVoltagePin2(outVPin2), 
        temperaturePin(tempPin) {}

    void initialize() {
        pinMode(pwmPin1, OUTPUT);
        pinMode(pwmPin2, OUTPUT);
        Serial.begin(115200);  // Higher baud rate for more data
    }

    void update() {
        float panelVoltage = readVoltage(panelVoltagePin);
        float filteredVoltage = voltageFilter.add(panelVoltage);

        updateVocTracking(filteredVoltage);
        updateWeightingFactors();

        float inferredCurrent = estimateAdvancedCurrent(filteredVoltage, dutyCycle1);
        float voltageError = targetPanelVoltage - filteredVoltage;

        performAdvancedPWMControl(voltageError);
        intelligentLoadBalancing(inferredCurrent);

        analogWrite(pwmPin1, dutyCycle1);
        analogWrite(pwmPin2, dutyCycle2);

        // Enhanced logging
        Serial.print("Voltage: "); Serial.print(filteredVoltage);
        Serial.print("V, Current Est: "); Serial.print(inferredCurrent);
        Serial.print("A, Duty1: "); Serial.print(dutyCycle1);
        Serial.print(", Duty2: "); Serial.println(dutyCycle2);
    }
};

SolarMPPTController solarController(9, 10, A0, A1, A2, A3);

void setup() {
    solarController.initialize();
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= SolarSystemConfig::UPDATE_INTERVAL) {
        lastUpdate = currentTime;
        solarController.update();
    }
}
