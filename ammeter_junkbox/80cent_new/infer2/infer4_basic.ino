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

// Converter Priority Ranks (lower number = higher priority)
const int CONVERTER_PRIORITIES[NUM_CONVERTERS] = {1, 2};

class SolarConverter {
private:
    int voltagePin;
    int pwmPin;
    float currentDutyCycle;
    int priority;
    float openCircuitVoltage;

public:
    SolarConverter(int _voltagePin, int _pwmPin, int _priority) 
        : voltagePin(_voltagePin), pwmPin(_pwmPin), priority(_priority), 
          currentDutyCycle(MIN_PWM_DUTY_CYCLE), openCircuitVoltage(0) {}

    void initialize() {
        pinMode(pwmPin, OUTPUT);
        pinMode(voltagePin, INPUT);
    }

    float readVoltage() {
        return analogRead(voltagePin) * (5.0 / 1023.0);
    }

    void measureOpenCircuitVoltage() {
        // Temporarily disable PWM to measure open circuit voltage
        analogWrite(pwmPin, 0);
        delay(50);  // Short delay to stabilize
        openCircuitVoltage = readVoltage();
        analogWrite(pwmPin, currentDutyCycle);
    }

    void adjustDutyCycle(float panelVoltage) {
        float currentVoltage = readVoltage();
        float targetVoltage = panelVoltage * TARGET_VOLTAGE_RATIO;

        // Proportional adjustment
        if (currentVoltage < targetVoltage) {
            currentDutyCycle = min(currentDutyCycle * 1.1, MAX_PWM_DUTY_CYCLE);
        } else if (currentVoltage > targetVoltage) {
            currentDutyCycle = max(currentDutyCycle * 0.9, MIN_PWM_DUTY_CYCLE);
        }

        analogWrite(pwmPin, currentDutyCycle);
    }

    float getPanelCurrent(float initialVoltage) {
        // Estimate panel current based on voltage change
        float currentVoltage = readVoltage();
        float voltageDifference = initialVoltage - currentVoltage;
        
        // Simple current estimation (needs calibration)
        return voltageDifference / 1.0;  // Placeholder, replace with actual conversion
    }

    int getPriority() const {
        return priority;
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
        // Measure open circuit voltage
        sortedConverters[i]->measureOpenCircuitVoltage();
        
        // Adjust duty cycle to maintain target voltage
        sortedConverters[i]->adjustDutyCycle(panelVoltage);
        
        // Optional: estimate and log panel current
        float panelCurrent = sortedConverters[i]->getPanelCurrent(panelVoltage);
        
        // Debug output
        Serial.print("Converter ");
        Serial.print(i);
        Serial.print(": Voltage = ");
        Serial.print(sortedConverters[i]->readVoltage());
        Serial.print(", Current = ");
        Serial.println(panelCurrent);
    }

    delay(500);  // Control loop update rate
}
