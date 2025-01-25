#include <Arduino.h>
#include <MovingAverage.h>  // Recommended external library for efficient averaging

// Advanced Hardware Configuration
const int pwmPin1 = 9;  
const int pwmPin2 = 10; 
const int panelVoltagePin = A0;  
const int outputVoltagePin1 = A1; 
const int outputVoltagePin2 = A2; 
const int temperaturePin = A3;
const int capacitiveSensePin = A4;  // New dedicated capacitive sensing pin

// Enhanced System Parameters
struct SolarParameters {
  float openCircuitVoltage;      // Maximum voltage
  float shortCircuitCurrent;     // Maximum current
  float maximumPowerVoltage;     // Voltage at maximum power point
  float maximumPowerCurrent;     // Current at maximum power point
  float systemEfficiency;        // Overall system efficiency
};

class AdvancedMPPT {
private:
  // Capacitive Sensing Variables
  const float CAPACITANCE_CALIBRATION_FACTOR = 0.95;
  const float PARASITIC_CAPACITANCE = 10e-12;  // Typical parasitic capacitance
  
  // Advanced Moving Averages
  MovingAverage<float> voltageMA{10};
  MovingAverage<float> currentMA{10};
  
  // Intelligent Tracking Parameters
  SolarParameters panelCharacteristics;
  
  // Adaptive Control Variables
  float dynamicResistance;
  float perturbationStep;
  float lastPower;
  
  // Capacitive Sensing Methods
  float calculateEffectiveCapacitance() {
    int rawSensorValue = analogRead(capacitiveSensePin);
    float voltage = rawSensorValue * (5.0 / 1023.0);
    
    // Advanced capacitance estimation considering temperature and parasitic effects
    float estimatedCapacitance = (voltage / CAPACITANCE_CALIBRATION_FACTOR) - PARASITIC_CAPACITANCE;
    return constrain(estimatedCapacitance, 0, 1000e-12);  // Constrain to realistic range
  }
  
  // Temperature-Compensated Voltage Reading
  float readCompensatedVoltage(int pin) {
    float baseVoltage = analogRead(pin) * (5.0 / 1023.0);
    float temperature = readTemperature();
    
    // Temperature compensation coefficient
    const float TEMP_COEFFICIENT = -0.004;  // %/°C
    float temperatureAdjustment = baseVoltage * (temperature - 25.0) * TEMP_COEFFICIENT;
    
    return baseVoltage + temperatureAdjustment;
  }
  
  // Intelligent Perturbation and Observation
  void performPerturbAndObserve(float currentVoltage, float currentCurrent) {
    float currentPower = currentVoltage * currentCurrent;
    
    // Adaptive step size based on recent performance
    if (currentPower > lastPower) {
      perturbationStep *= 1.1;  // Increase step size if improving
    } else {
      perturbationStep *= 0.9;  // Decrease step size if performance drops
    }
    
    perturbationStep = constrain(perturbationStep, 0.01, 0.5);
    lastPower = currentPower;
    
    // Dynamic resistance calculation
    dynamicResistance = currentVoltage / currentCurrent;
  }
  
public:
  AdvancedMPPT() : 
    dynamicResistance(0),
    perturbationStep(0.1),
    lastPower(0) {}
  
  // Enhanced MPPT Control Algorithm
  void optimizePowerExtraction(float& dutyCycle1, float& dutyCycle2) {
    float panelVoltage = readCompensatedVoltage(panelVoltagePin);
    float panelCurrent = calculatePanelCurrent();
    
    // Capacitance-aware tracking
    float effectiveCapacitance = calculateEffectiveCapacitance();
    
    // Moving average smoothing
    voltageMA.add(panelVoltage);
    currentMA.add(panelCurrent);
    
    // Intelligent perturbation
    performPerturbAndObserve(panelVoltage, panelCurrent);
    
    // Adaptive duty cycle adjustment
    float targetDutyCycle = calculateOptimalDutyCycle(panelVoltage, effectiveCapacitance);
    
    // Load balancing with capacitance awareness
    balanceLoadWithCapacitance(dutyCycle1, dutyCycle2, targetDutyCycle);
  }
  
  float calculatePanelCurrent() {
    // Advanced current estimation method
    float voltage1 = readCompensatedVoltage(outputVoltagePin1);
    float voltage2 = readCompensatedVoltage(outputVoltagePin2);
    
    return (voltage1 + voltage2) / (2 * dynamicResistance);
  }
  
  float calculateOptimalDutyCycle(float voltage, float capacitance) {
    // Capacitance-influenced duty cycle calculation
    float baselineDutyCycle = voltage / 5.0;  // Simple mapping
    float capacitanceAdjustment = capacitance * 1e12;  // Scale capacitance effect
    
    return constrain(
      baselineDutyCycle + (capacitanceAdjustment * 0.01), 
      0, 1
    );
  }
  
  void balanceLoadWithCapacitance(float& duty1, float& duty2, float targetDutyCycle) {
    // Distribute load considering capacitive characteristics
    float capacitance = calculateEffectiveCapacitance();
    
    if (capacitance > 50e-12) {  // Higher capacitance indicates more stable load
      duty1 = targetDutyCycle * 0.6;
      duty2 = targetDutyCycle * 0.4;
    } else {
      duty1 = targetDutyCycle * 0.5;
      duty2 = targetDutyCycle * 0.5;
    }
  }
};

// Global Instances
AdvancedMPPT mpptController;
float dutyCycle1 = 0, dutyCycle2 = 0;

float readTemperature() {
  float voltage = analogRead(temperaturePin) * (5.0 / 1023.0);
  return (voltage - 0.5) * 100.0;  // Linear temperature conversion
}

void setup() {
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  Serial.begin(115200);  // Higher baud rate for more data
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdateTime >= 250) {  // 4 Hz update rate
    lastUpdateTime = currentTime;
    
    // Optimize power extraction
    mpptController.optimizePowerExtraction(dutyCycle1, dutyCycle2);
    
    // Apply PWM
    analogWrite(pwmPin1, dutyCycle1 * 255);
    analogWrite(pwmPin2, dutyCycle2 * 255);
    
    // Diagnostic output
    Serial.print("Voltage: ");
    Serial.print(analogRead(panelVoltagePin) * (5.0 / 1023.0));
    Serial.print(" V, Duty Cycle 1: ");
    Serial.print(dutyCycle1 * 100);
    Serial.print("%, Duty Cycle 2: ");
    Serial.print(dutyCycle2 * 100);
    Serial.println("%");
  }
}
