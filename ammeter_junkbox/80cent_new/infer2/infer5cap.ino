#include <Arduino.h>

// Pin Definitions
#define PANEL_VOLTAGE_PIN A0        // Analog input for panel voltage
#define CONVERTER1_VOLTAGE_PIN A1   // Voltage output of first DC-DC converter
#define CONVERTER2_VOLTAGE_PIN A2   // Voltage output of second DC-DC converter
#define CONVERTER1_PWM_PIN 9        // PWM pin for first converter
#define CONVERTER2_PWM_PIN 10       // PWM pin for second converter

// Configuration Parameters
const float TARGET_VOLTAGE_RATIO = 0.80;  // Target voltage maintenance (80%)
const int NUM_CONVERTERS = 2;
const float MAX_PWM_DUTY_CYCLE = 255.0;   // 8-bit PWM
const float MIN_PWM_DUTY_CYCLE = 10.0;    // Minimum duty cycle to prevent stalling

// System Dynamics Parameters
const float PANEL_CAPACITANCE = 100e-6;   // Fixed panel capacitor (100µF)
const float SAMPLING_PERIOD = 0.5;        // Sampling period in seconds
const float PANEL_INTERNAL_RESISTANCE = 0.5; // Estimated panel internal resistance

// Converter Priority Ranks (lower number = higher priority)
const int CONVERTER_PRIORITIES[NUM_CONVERTERS] = {1, 2};

class SolarConverter {
private:
    int voltagePin;
    int pwmPin;
    float currentDutyCycle;
    int priority;
    
    // Dynamic inference variables
    float previousVoltage;
    float previousCurrent;
    unsigned long lastUpdateTime;

public:
    SolarConverter(int _voltagePin, int _pwmPin, int _priority) 
        : voltagePin(_voltagePin), pwmPin(_pwmPin), priority(_priority), 
          currentDutyCycle(MIN_PWM_DUTY_CYCLE), 
          previousVoltage(0), previousCurrent(0), lastUpdateTime(0) {}

    void initialize() {
        pinMode(pwmPin, OUTPUT);
        pinMode(voltagePin, INPUT);
    }

    float readVoltage() {
        return analogRead(voltagePin) * (5.0 / 1023.0);
    }

    float inferencePanelCurrent(float panelVoltage) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
        
        if (deltaTime < SAMPLING_PERIOD) {
            return previousCurrent;  // Return last calculated current if sampling period not met
        }

        // Voltage change due to capacitor discharge and load
        float voltageChange = previousVoltage - panelVoltage;

        // Current inference using capacitor discharge equation
        // I = C * (dV/dt) + V/R
        float currentByCapacitor = PANEL_CAPACITANCE * (voltageChange / deltaTime);
        float currentByResistance = panelVoltage / PANEL_INTERNAL_RESISTANCE;
        
        float estimatedCurrent = currentByCapacitor + currentByResistance;

        // Exponential moving average for smoother estimation
        const float ALPHA = 0.3;
        estimatedCurrent = ALPHA * estimatedCurrent + (1 - ALPHA) * previousCurrent;

        // Update tracking variables
        previousVoltage = panelVoltage;
        previousCurrent = estimatedCurrent;
        lastUpdateTime = currentTime;

        return estimatedCurrent;
    }

    void adjustDutyCycle(float panelVoltage, float estimatedCurrent) {
        float currentVoltage = readVoltage();
        float targetVoltage = panelVoltage * TARGET_VOLTAGE_RATIO;

        // Advanced duty cycle adjustment considering current
        float voltageError = targetVoltage - currentVoltage;
        float currentError = estimatedCurrent - previousCurrent;

        // Proportional-Derivative like adjustment
        float dutyCycleAdjustment = 
            (voltageError * 10.0) +  // Proportional voltage term
            (currentError * 5.0);    // Current change term

        // Update duty cycle with bounded adjustment
        currentDutyCycle = constrain(
            currentDutyCycle + dutyCycleAdjustment, 
            MIN_PWM_DUTY_CYCLE, 
            MAX_PWM_DUTY_CYCLE
        );

        analogWrite(pwmPin, currentDutyCycle);
    }

    int getPriority() const {
        return priority;
    }

    float getCurrentDutyCycle() const {
        return currentDutyCycle;
    }
};

// Array of converters
SolarConverter converters[] = {
    SolarConverter(CONVERTER1_VOLTAGE_PIN, CONVERTER1_PWM_PIN, CONVERTER_PRIORITIES[0]),
    SolarConverter(CONVERTER2_VOLTAGE_PIN, CONVERTER2_PWM_PIN, CONVERTER_PRIORITIES[1])
};

void setup() {
    Serial.begin(9600);  // For debugging
    
    // Initialize converters
    for (int i = 0; i < NUM_CONVERTERS; i++) {
        converters[i].initialize();
    }
}

void loop() {
    // Read panel voltage
    float panelVoltage = analogRead(PANEL_VOLTAGE_PIN) * (5.0 / 1023.0);
    
    // Sort converters by priority
    SolarConverter* sortedConverters[NUM_CONVERTERS];
    for (int i = 0; i < NUM_CONVERTERS; i++) {
        sortedConverters[i] = &converters[i];
    }
    
    // Simple bubble sort by priority
    for (int i = 0; i < NUM_CONVERTERS - 1; i++) {
        for (int j = 0; j < NUM_CONVERTERS - i - 1; j++) {
            if (sortedConverters[j]->getPriority() > sortedConverters[j+1]->getPriority()) {
                SolarConverter* temp = sortedConverters[j];
                sortedConverters[j] = sortedConverters[j+1];
                sortedConverters[j+1] = temp;
            }
        }
    }

    // Process converters in priority order
    for (int i = 0; i < NUM_CONVERTERS; i++) {
        // Infer panel current using system dynamics
        float estimatedCurrent = sortedConverters[i]->inferencePanelCurrent(panelVoltage);
        
        // Adjust duty cycle considering voltage and current dynamics
        sortedConverters[i]->adjustDutyCycle(panelVoltage, estimatedCurrent);
        
        // Debug output
        Serial.print("Converter ");
        Serial.print(i);
        Serial.print(": Voltage = ");
        Serial.print(sortedConverters[i]->readVoltage());
        Serial.print(", Estimated Current = ");
        Serial.print(estimatedCurrent);
        Serial.print(", Duty Cycle = ");
        Serial.println(sortedConverters[i]->getCurrentDutyCycle());
    }

    delay(500);  // Control loop update rate
}
