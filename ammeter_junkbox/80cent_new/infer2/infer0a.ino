#include <Arduino.h>

// Pin definitions
const int pwmPin1 = 9;  
const int pwmPin2 = 10; 
const int panelVoltagePin = A0;  
const int outputVoltagePin1 = A1; 
const int outputVoltagePin2 = A2; 
const int temperaturePin = A3;  

// System parameters
float panelVoc = 0.0;  
float targetPanelVoltage = 0.0; 
const float panelVoltageTargetRatio = 0.80;  
const float tempCoefficient = -0.003;  
const float movingAvgWeight = 0.1;  

// PWM control parameters
int dutyCycle1 = 0;
int dutyCycle2 = 0;
const int maxDuty = 255;
const float pidKp = 1.2, pidKi = 0.05, pidKd = 0.02;  

// MPPT variables
float previousPower = 0.0;
float previousVoltage = 0.0;
float previousDutyCycle = 0.0;
float integralError = 0.0;
float lastError = 0.0;

// Moving average filter variables
float voltageBuffer[10] = {0};
int bufferIndex = 0;
float filteredVoltage = 0;

// Adaptive weighting factors
float weight_dVdD = 0.5;
float weight_vocTrend = 0.3;
float weight_tempComp = 0.2;

// Timing
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 500;

// Function to read voltage
float readVoltage(int pin) {
  return analogRead(pin) * (5.0 / 1023.0);
}

// Function to estimate temperature in Celsius
float readTemperature() {
  return (readVoltage(temperaturePin) - 0.5) * 100.0;
}

// Moving average filter for voltage smoothing
float movingAverageFilter(float newValue) {
  voltageBuffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % 10;

  float sum = 0.0;
  for (int i = 0; i < 10; i++) {
    sum += voltageBuffer[i];
  }
  return sum / 10.0;
}

// Dynamic adaptive weighting system
void updateWeightingFactors(float panelVoltage) {
  if (panelVoltage < (panelVoc * 0.7)) {
    weight_dVdD = 0.7;
    weight_vocTrend = 0.2;
    weight_tempComp = 0.1;
  } else {
    weight_dVdD = 0.5;
    weight_vocTrend = 0.3;
    weight_tempComp = 0.2;
  }
}

// Estimate current using multiple weighted methods
float estimateCurrent(float currentVoltage, float dutyCycle) {
  float dVdD = (previousVoltage - currentVoltage) / (previousDutyCycle - dutyCycle);
  float vocTrend = (panelVoc - currentVoltage) / panelVoc;
  float tempEffect = (readTemperature() - 25.0) * tempCoefficient * panelVoc;
  
  return (dVdD * weight_dVdD) + (vocTrend * weight_vocTrend) + (tempEffect * weight_tempComp);
}

// Peak detection with decay for Voc tracking
void updateVoc(float panelVoltage) {
  if (panelVoltage > panelVoc) {
    panelVoc = panelVoltage;
  } else {
    panelVoc *= 0.999; 
  }
  targetPanelVoltage = panelVoc * panelVoltageTargetRatio;
}

// PID control for adaptive duty cycle
void adjustPWM_PID(float error) {
  integralError += error;
  float derivative = error - lastError;
  float output = (pidKp * error) + (pidKi * integralError) + (pidKd * derivative);

  dutyCycle1 = constrain(dutyCycle1 + output, 0, maxDuty);
  dutyCycle2 = constrain(dutyCycle2 + output, 0, maxDuty);
  
  lastError = error;
}

// Load balancing algorithm
void balanceLoad(float inferredCurrent) {
  if (inferredCurrent > 2.0) {  // Assume 2A threshold for rebalancing
    dutyCycle1 = constrain(dutyCycle1 + 10, 0, maxDuty);
    dutyCycle2 = constrain(dutyCycle2 - 10, 0, maxDuty);
  } else {
    dutyCycle1 = constrain(dutyCycle1 - 10, 0, maxDuty);
    dutyCycle2 = constrain(dutyCycle2 + 10, 0, maxDuty);
  }
}

// Energy efficiency mode
void enterLowPowerMode(float panelVoltage) {
  if (panelVoltage < (panelVoc * 0.5)) {
    dutyCycle1 = 0;
    dutyCycle2 = 0;
  }
}

// Main control logic
void controlSystem(float panelVoltage) {
  float filteredPanelVoltage = movingAverageFilter(panelVoltage);

  updateVoc(filteredPanelVoltage);
  updateWeightingFactors(filteredPanelVoltage);

  float inferredCurrent = estimateCurrent(filteredPanelVoltage, dutyCycle1 + dutyCycle2);
  float voltageError = targetPanelVoltage - filteredPanelVoltage;

  adjustPWM_PID(voltageError);
  balanceLoad(inferredCurrent);
  enterLowPowerMode(filteredPanelVoltage);

  analogWrite(pwmPin1, dutyCycle1);
  analogWrite(pwmPin2, dutyCycle2);
}

void setup() {
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;

    float panelVoltage = readVoltage(panelVoltagePin);
    controlSystem(panelVoltage);

    Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
    Serial.print("V, Duty1: "); Serial.print(dutyCycle1);
    Serial.print(", Duty2: "); Serial.println(dutyCycle2);

    previousVoltage = panelVoltage;
    previousDutyCycle = dutyCycle1 + dutyCycle2;
  }
}
