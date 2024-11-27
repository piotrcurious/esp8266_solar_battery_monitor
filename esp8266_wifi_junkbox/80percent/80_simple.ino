#include <Arduino.h>

// Pin definitions
const int pwmPin = D1;          // PWM pin to control the load
const int voltagePin = A0;      // Analog pin to monitor input voltage

// Configuration constants
const float maxNoLoadVoltage = 13.8 * 1.25; // Estimated max voltage
const float minAllowedVoltage = 13.8;      // Minimum allowed input voltage
const int pwmFrequency = 5000;             // PWM frequency (Hz)
const int pwmResolution = 8;               // PWM resolution (bits)
const float pwmMaxValue = pow(2, pwmResolution) - 1;

// Variables for state
float noLoadVoltage = 0.0;
float currentVoltage = 0.0;
float previousVoltage = 0.0;
float voltageDropRate = 0.0;
float voltageRiseRate = 0.0;
int pwmValue = 0;

// Dynamic intervals
unsigned long voltageCheckInterval = 100;   // Voltage change check interval (ms)
unsigned long noLoadCheckInterval = 5000;  // No-load voltage check interval (ms)
unsigned long lastVoltageCheck = 0;
unsigned long lastNoLoadCheck = 0;

// Function prototypes
void measureNoLoadVoltage();
void probeVoltageDropRate();
void probeVoltageRiseRate();
void controlPWM();
void updateIntervals();

void setup() {
  Serial.begin(115200);

  // Initialize PWM
  ledcSetup(0, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, 0);

  // Measure initial no-load voltage
  measureNoLoadVoltage();

  Serial.println("System initialized.");
}

void loop() {
  unsigned long currentTime = millis();

  // Check voltage change rates periodically
  if (currentTime - lastVoltageCheck >= voltageCheckInterval) {
    lastVoltageCheck = currentTime;

    previousVoltage = currentVoltage;
    currentVoltage = analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);

    probeVoltageDropRate();
    probeVoltageRiseRate();

    // Adjust intervals dynamically
    updateIntervals();
  }

  // Recalculate no-load voltage periodically
  if (currentTime - lastNoLoadCheck >= noLoadCheckInterval) {
    lastNoLoadCheck = currentTime;

    if (pwmValue == 0 && voltageRiseRate > 0.0) {
      // Recalculate no-load voltage using the rise rate
      noLoadVoltage = currentVoltage + (voltageRiseRate * (noLoadCheckInterval / 1000.0));
      Serial.print("Recalculated no-load voltage: ");
      Serial.println(noLoadVoltage);
    }
  }

  // Control the PWM to maintain voltage within bounds
  controlPWM();
}

void measureNoLoadVoltage() {
  // Average voltage measurement to determine no-load voltage
  float voltageSum = 0.0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    voltageSum += analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
    delay(10);
  }
  noLoadVoltage = voltageSum / samples;
  Serial.print("No-load voltage: ");
  Serial.println(noLoadVoltage);
}

void probeVoltageDropRate() {
  if (pwmValue > 0) {
    voltageDropRate = (previousVoltage - currentVoltage) / (voltageCheckInterval / 1000.0);
    Serial.print("Voltage drop rate: ");
    Serial.println(voltageDropRate);
  }
}

void probeVoltageRiseRate() {
  if (pwmValue == 0) {
    voltageRiseRate = (currentVoltage - previousVoltage) / (voltageCheckInterval / 1000.0);
    Serial.print("Voltage rise rate: ");
    Serial.println(voltageRiseRate);
  }
}

void controlPWM() {
  float targetVoltage = noLoadVoltage * 0.8;

  if (currentVoltage < minAllowedVoltage) {
    pwmValue = 0;  // Turn off load
  } else if (currentVoltage < targetVoltage) {
    pwmValue = min(pwmValue + 1, (int)pwmMaxValue);  // Increase load
  } else if (currentVoltage > targetVoltage) {
    pwmValue = max(pwmValue - 1, 0);  // Decrease load
  }

  ledcWrite(0, pwmValue);

  Serial.print("Current voltage: ");
  Serial.println(currentVoltage);
  Serial.print("PWM value: ");
  Serial.println(pwmValue);
}

void updateIntervals() {
  // Dynamically adjust voltage check interval based on voltage change rate
  float changeRate = max(abs(voltageDropRate), abs(voltageRiseRate));

  if (changeRate > 1.0) {
    voltageCheckInterval = 50;  // Faster checks for rapid changes
  } else if (changeRate > 0.1) {
    voltageCheckInterval = 100;  // Moderate changes
  } else {
    voltageCheckInterval = 200;  // Slower checks for stable conditions
  }

  // Adjust no-load voltage check frequency based on stability
  if (changeRate < 0.05) {
    noLoadCheckInterval = 10000;  // Infrequent checks for stable voltage
  } else {
    noLoadCheckInterval = 5000;  // More frequent checks for instability
  }

  Serial.print("Voltage check interval: ");
  Serial.println(voltageCheckInterval);
  Serial.print("No-load voltage check interval: ");
  Serial.println(noLoadCheckInterval);
}
