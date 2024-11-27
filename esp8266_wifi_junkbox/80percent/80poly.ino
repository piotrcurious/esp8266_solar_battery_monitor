#include <Arduino.h>
#include <math.h>

// Pin definitions
const int pwmPin = D1;          // PWM pin to control the load
const int voltagePin = A0;      // Analog pin to monitor input voltage

// Configuration constants
const float maxNoLoadVoltage = 13.8 * 1.25; // Estimated max voltage
const float minAllowedVoltage = 13.8;      // Minimum allowed input voltage
const int pwmFrequency = 5000;             // PWM frequency (Hz)
const int pwmResolution = 8;               // PWM resolution (bits)
const float pwmMaxValue = pow(2, pwmResolution) - 1;

// State variables
float noLoadVoltage = 0.0;           // Measured no-load voltage
float estimatedNoLoadVoltage = 0.0;  // Dynamically estimated no-load voltage
float currentVoltage = 0.0;
float previousVoltage = 0.0;
float voltageDropRate = 0.0;
float voltageRiseRate = 0.0;
int pwmValue = 0;

// Full curve data
const int curveSize = 100;
float dropCurve[curveSize];    // Stores voltage drop curve
float riseCurve[curveSize];    // Stores voltage rise curve
int dropCurveSize = 0;
int riseCurveSize = 0;

// Fitted function coefficients
const int polynomialDegree = 2;   // Degree of the polynomial to fit
float dropCoefficients[polynomialDegree + 1]; // Quadratic coefficients for voltage drop
float riseCoefficients[polynomialDegree + 1]; // Quadratic coefficients for voltage rise

// Dynamic intervals
unsigned long voltageCheckInterval = 100;      // Voltage change check interval (ms)
unsigned long noLoadCheckInterval = 5000;      // No-load voltage check interval (ms)
unsigned long fullDropCurveInterval = 10000;   // Full drop curve probe interval (ms)
unsigned long fullRiseCurveInterval = 10000;   // Full rise curve probe interval (ms)
unsigned long lastVoltageCheck = 0;
unsigned long lastNoLoadCheck = 0;
unsigned long lastFullDropCurveCheck = 0;
unsigned long lastFullRiseCurveCheck = 0;

// Function prototypes
void measureNoLoadVoltage();
void probeVoltageChangeRate(bool isDrop);
void probeFullCurve(bool isDrop, float *curve, int &size, float *coefficients);
void fitCurve(const float *curve, int size, int degree, float *coefficients);
float evaluateCurve(float x, const float *coefficients, int degree);
void controlPWM();
void updateIntervals();

void setup() {
  Serial.begin(115200);

  // Initialize PWM
  ledcSetup(0, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, 0);

  // Measure initial no-load voltage
  measureNoLoadVoltage();
  estimatedNoLoadVoltage = noLoadVoltage;

  Serial.println("System initialized.");
}

void loop() {
  unsigned long currentTime = millis();

  // Check voltage change rates periodically
  if (currentTime - lastVoltageCheck >= voltageCheckInterval) {
    lastVoltageCheck = currentTime;

    previousVoltage = currentVoltage;
    currentVoltage = analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);

    // Probe voltage drop and rise rates
    probeVoltageChangeRate(true);  // Drop
    probeVoltageChangeRate(false); // Rise

    // Dynamically estimate no-load voltage
    estimatedNoLoadVoltage = currentVoltage + (voltageRiseRate * 0.8);

    // Adjust intervals dynamically
    updateIntervals();
  }

  // Periodically measure no-load voltage
  if (currentTime - lastNoLoadCheck >= noLoadCheckInterval) {
    lastNoLoadCheck = currentTime;
    measureNoLoadVoltage();
    Serial.print("Measured no-load voltage: ");
    Serial.println(noLoadVoltage);
    Serial.print("Estimated no-load voltage: ");
    Serial.println(estimatedNoLoadVoltage);
  }

  // Periodically probe full voltage drop curve
  if (currentTime - lastFullDropCurveCheck >= fullDropCurveInterval) {
    lastFullDropCurveCheck = currentTime;
    probeFullCurve(true, dropCurve, dropCurveSize, dropCoefficients);
  }

  // Periodically probe full voltage rise curve
  if (currentTime - lastFullRiseCurveCheck >= fullRiseCurveInterval) {
    lastFullRiseCurveCheck = currentTime;
    probeFullCurve(false, riseCurve, riseCurveSize, riseCoefficients);
  }

  // Control the PWM to maintain voltage within bounds
  controlPWM();
}

void measureNoLoadVoltage() {
  float voltageSum = 0.0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    voltageSum += analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
    delay(10);
  }
  noLoadVoltage = voltageSum / samples;
}

void probeVoltageChangeRate(bool isDrop) {
  if ((isDrop && pwmValue > 0) || (!isDrop && pwmValue == 0)) {
    float rate = (currentVoltage - previousVoltage) / (voltageCheckInterval / 1000.0);
    if (isDrop) {
      voltageDropRate = rate;
      Serial.print("Voltage drop rate: ");
      Serial.println(voltageDropRate);
    } else {
      voltageRiseRate = -rate;
      Serial.print("Voltage rise rate: ");
      Serial.println(voltageRiseRate);
    }
  }
}

void probeFullCurve(bool isDrop, float *curve, int &size, float *coefficients) {
  Serial.println(isDrop ? "Probing full voltage drop curve..." : "Probing full voltage rise curve...");
  size = 0;

  pwmValue = isDrop ? pwmMaxValue : 0;
  ledcWrite(0, pwmValue);

  for (int i = 0; i < curveSize; i++) {
    curve[i] = analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
    size++;
    delay(50);

    if (isDrop && curve[i] < minAllowedVoltage) break;
  }

  pwmValue = 0;
  ledcWrite(0, pwmValue);

  fitCurve(curve, size, polynomialDegree, coefficients);
  Serial.println(isDrop ? "Voltage drop curve fitted." : "Voltage rise curve fitted.");
}

void fitCurve(const float *curve, int size, int degree, float *coefficients) {
  // Simple polynomial fitting using least squares
  // TODO: Implement matrix-based least-squares fitting for general polynomials.
}

float evaluateCurve(float x, const float *coefficients, int degree) {
  float result = 0.0;
  for (int i = 0; i <= degree; i++) {
    result += coefficients[i] * pow(x, i);
  }
  return result;
}

void controlPWM() {
  // Use the fitted curves to control PWM
  float predictedVoltage = evaluateCurve(pwmValue / pwmMaxValue, dropCoefficients, polynomialDegree);
  if (predictedVoltage < minAllowedVoltage) {
    pwmValue = max(pwmValue - 5, 0);
  } else if (currentVoltage < 0.8 * estimatedNoLoadVoltage) {
    pwmValue = min(pwmValue + 5, pwmMaxValue);
  }
  ledcWrite(0, pwmValue);
}

void updateIntervals() {
  // Dynamically adjust intervals based on curve fit error, voltage stability, etc.
}
