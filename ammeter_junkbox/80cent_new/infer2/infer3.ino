#include <Arduino.h>

// Configuration Parameters
#define NUM_CONVERTERS 2
#define PANEL_VOLTAGE_PIN A0
#define CONVERTER_VOLTAGE_PINS {A1, A2}
#define CONVERTER_PWM_PINS {9, 10}
#define CONVERTER_ENABLE_PINS {7, 8}

// Optimization Parameters
const float TARGET_VOLTAGE_RATIO = 0.80; // 80% of input panel voltage
const float MAX_PWM_DUTY_CYCLE = 0.90;
const float MIN_PWM_DUTY_CYCLE = 0.10;
const float VOLTAGE_TOLERANCE = 0.05; // 5% tolerance

class DCDCConverter {
private:
    int voltagePin;
    int pwmPin;
    int enablePin;
    int priority;
    float currentDutyCycle;

public:
    DCDCConverter(int vPin, int pwmPin, int enPin, int priorityRank) 
        : voltagePin(vPin), pwmPin(pwmPin), enablePin(enPin), priority(priorityRank), currentDutyCycle(0.5) {
        pinMode(pwmPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW); // Initially disabled
    }

    void enable() {
        digitalWrite(enablePin, HIGH);
    }

    void disable() {
        digitalWrite(enablePin, LOW);
    }

    float readOutputVoltage() {
        return analogRead(voltagePin) * (5.0 / 1023.0);
    }

    void adjustDutyCycle(float panelVoltage) {
        float outputVoltage = readOutputVoltage();
        float targetVoltage = panelVoltage * TARGET_VOLTAGE_RATIO;

        // Proportional control for duty cycle adjustment
        if (outputVoltage < targetVoltage * (1 - VOLTAGE_TOLERANCE)) {
            // Increase duty cycle
            currentDutyCycle = min(currentDutyCycle * 1.05, MAX_PWM_DUTY_CYCLE);
        } else if (outputVoltage > targetVoltage * (1 + VOLTAGE_TOLERANCE)) {
            // Decrease duty cycle
            currentDutyCycle = max(currentDutyCycle * 0.95, MIN_PWM_DUTY_CYCLE);
        }

        // Apply PWM
        int pwmValue = currentDutyCycle * 255;
        analogWrite(pwmPin, pwmValue);
    }

    float getCurrentDutyCycle() const {
        return currentDutyCycle;
    }

    int getPriority() const {
        return priority;
    }
};

class SolarPanelOptimizer {
private:
    DCDCConverter* converters[NUM_CONVERTERS];
    int panelVoltagePin;

public:
    SolarPanelOptimizer(int panelPin) : panelVoltagePin(panelPin) {
        // Initialize converters with different priorities
        converters[0] = new DCDCConverter(A1, 9, 7, 1); // Higher priority
        converters[1] = new DCDCConverter(A2, 10, 8, 2); // Lower priority
    }

    float readPanelVoltage() {
        return analogRead(panelVoltagePin) * (5.0 / 1023.0);
    }

    void optimizePowerDistribution() {
        float panelVoltage = readPanelVoltage();

        // Sort converters by priority (bubble sort)
        for (int i = 0; i < NUM_CONVERTERS - 1; i++) {
            for (int j = 0; j < NUM_CONVERTERS - i - 1; j++) {
                if (converters[j]->getPriority() > converters[j + 1]->getPriority()) {
                    DCDCConverter* temp = converters[j];
                    converters[j] = converters[j + 1];
                    converters[j + 1] = temp;
                }
            }
        }

        // Optimize converters based on priority
        for (int i = 0; i < NUM_CONVERTERS; i++) {
            if (i == 0) {
                // Highest priority converter always tries to optimize
                converters[i]->enable();
                converters[i]->adjustDutyCycle(panelVoltage);
            } else {
                // Lower priority converters only activate if higher priority ones are saturated
                if (converters[i-1]->getCurrentDutyCycle() >= MAX_PWM_DUTY_CYCLE) {
                    converters[i]->enable();
                    converters[i]->adjustDutyCycle(panelVoltage);
                } else {
                    converters[i]->disable();
                }
            }
        }
    }

    void inferPanelCurrent() {
        // Simplified current inference based on voltage changes and duty cycle
        float openCircuitVoltage = readPanelVoltage();
        
        // Simulate load variations and current changes
        for (int i = 0; i < NUM_CONVERTERS; i++) {
            float dutyCycle = converters[i]->getCurrentDutyCycle();
            float estimatedCurrent = openCircuitVoltage * (1 - dutyCycle);
            
            // Optional: Log or use estimated current
            // Serial.print("Converter "); Serial.print(i);
            // Serial.print(" Estimated Current: "); Serial.println(estimatedCurrent);
        }
    }

    ~SolarPanelOptimizer() {
        for (int i = 0; i < NUM_CONVERTERS; i++) {
            delete converters[i];
        }
    }
};

SolarPanelOptimizer* optimizer;

void setup() {
    Serial.begin(9600);
    optimizer = new SolarPanelOptimizer(PANEL_VOLTAGE_PIN);
}

void loop() {
    optimizer->optimizePowerDistribution();
    optimizer->inferPanelCurrent();
    
    delay(100); // Sampling interval
}
