#include <Arduino.h>

// Pin definitions
const int pwmPin1 = 9;  // PWM for DC-DC converter 1
const int pwmPin2 = 10; // PWM for DC-DC converter 2
const int panelVoltagePin = A0; // Solar panel voltage
const int outputVoltagePin1 = A1; // DC-DC converter 1 output
const int outputVoltagePin2 = A2; // DC-DC converter 2 output
const int temperaturePin = A3; // Temperature sensor

// System parameters
float panelVoc = 0.0;  // Estimated open-circuit voltage
float targetPanelVoltage = 0.0; 
const float panelVoltageTargetRatio = 0.80;  // 80% of Voc
const float tempCoefficient = -0.003;  // Panel voltage temp coefficient (V/°C)

// PWM control parameters
int dutyCycle1 = 0;
int dutyCycle2 = 0;
const int maxDuty = 255; // Max PWM value (100% duty cycle)
const float pidKp = 1.5, pidKi = 0.1, pidKd = 0.05; // PID gains

// MPPT parameters
float previousPower = 0.0;
float previousVoltage = 0.0;
float previousDutyCycle = 0.0;
float integralError = 0.0;
float lastError = 0.0;

// Timing
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 500;  // 500ms for faster response

// Function to read voltage
float readVoltage(int pin) {
  return analogRead(pin) * (5.0 / 1023.0);
}

// Function to estimate temperature in Celsius
float readTemperature() {
  return (readVoltage(temperaturePin) - 0.5) * 100.0;
}

// Peak detection with decay for Voc tracking
void updateVoc(float panelVoltage) {
  if (panelVoltage > panelVoc) {
    panelVoc = panelVoltage;  // Update peak Voc
  } else {
    panelVoc *= 0.999;  // Slowly decay to account for changing conditions
  }
  targetPanelVoltage = panelVoc * panelVoltageTargetRatio;
}

// Kalman filter implementation for smoothing voltage readings
float kalmanFilter(float measurement, float prevEstimate, float processNoise, float measurementNoise) {
  static float estimate = 0.0;
  static float errorEstimate = 1.0;
  float kalmanGain = errorEstimate / (errorEstimate + measurementNoise);
  estimate = prevEstimate + kalmanGain * (measurement - prevEstimate);
  errorEstimate = (1.0 - kalmanGain) * errorEstimate + processNoise;
  return estimate;
}

// MPPT Perturb and Observe (P&O)
void mpptAdjustPWM(float panelVoltage) {
  float power = panelVoltage * (dutyCycle1 + dutyCycle2) / 255.0;
  
  if (power > previousPower) {
    dutyCycle1 += 2;
    dutyCycle2 += 2;
  } else {
    dutyCycle1 -= 2;
    dutyCycle2 -= 2;
  }

  dutyCycle1 = constrain(dutyCycle1, 0, maxDuty);
  dutyCycle2 = constrain(dutyCycle2, 0, maxDuty);
  
  previousPower = power;
}

// Adaptive PID control for duty cycle adjustments
void adjustPWM_PID(float error) {
  integralError += error;
  float derivative = error - lastError;
  float output = (pidKp * error) + (pidKi * integralError) + (pidKd * derivative);

  dutyCycle1 = constrain(dutyCycle1 + output, 0, maxDuty);
  dutyCycle2 = constrain(dutyCycle2 + output, 0, maxDuty);
  
  lastError = error;
}

// Main control logic
void adjustPWM(float panelVoltage) {
  float error = targetPanelVoltage - panelVoltage;
  adjustPWM_PID(error);
  mpptAdjustPWM(panelVoltage);

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
    panelVoltage = kalmanFilter(panelVoltage, previousVoltage, 0.01, 0.1);
    
    updateVoc(panelVoltage);
    adjustPWM(panelVoltage);

    Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
    Serial.print("V, Duty 1: "); Serial.print(dutyCycle1);
    Serial.print(", Duty 2: "); Serial.println(dutyCycle2);

    previousVoltage = panelVoltage;
  }
}
