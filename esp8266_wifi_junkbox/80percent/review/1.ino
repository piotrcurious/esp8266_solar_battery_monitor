#include <Arduino.h>
#include <math.h>
#include <Eigen.h>  // For matrix-based least squares fitting

// Pin definitions
const int pwmPin = D1;          // PWM pin to control the load
const int voltagePin = A0;      // Analog pin to monitor input voltage

// Configuration constants
const float maxNoLoadVoltage = 13.8 * 1.25; // Estimated max voltage
const float minAllowedVoltage = 13.8;       // Minimum allowed input voltage
const int pwmFrequency = 5000;              // PWM frequency (Hz)
const int pwmResolution = 8;                // PWM resolution (bits)
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
struct CurveData {
  float voltage;
  unsigned long timestamp;
};
CurveData dropCurve[curveSize];
CurveData riseCurve[curveSize];
int dropCurveSize = 0;
int riseCurveSize = 0;

// Fitted function coefficients
const int polynomialDegree = 2;   // Degree of the polynomial to fit
float dropCoefficients[polynomialDegree + 1] = {0}; // Quadratic coefficients for voltage drop
float riseCoefficients[polynomialDegree + 1] = {0}; // Quadratic coefficients for voltage rise

// Curve fitting error metrics
float dropCurveFitError = 0.0;
float riseCurveFitError = 0.0;

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
void probeFullCurve(bool isDrop, CurveData *curve, int &size, float *coefficients, float &fitError);
void fitCurve(const CurveData *curve, int size, int degree, float *coefficients, float &fitError);
float evaluateCurve(float x, const float *coefficients, int degree);
void controlPWM();
void updateIntervals();
float calculateRMSE(const CurveData *curve, int size, const float *coefficients, int degree);

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

    // Dynamically estimate no-load voltage with adaptive factor
    float adaptiveFactor = constrain(abs(voltageDropRate), 0.5, 1.5);
    estimatedNoLoadVoltage = currentVoltage + (voltageRiseRate * adaptiveFactor);

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
    probeFullCurve(true, dropCurve, dropCurveSize, dropCoefficients, dropCurveFitError);
  }

  // Periodically probe full voltage rise curve
  if (currentTime - lastFullRiseCurveCheck >= fullRiseCurveInterval) {
    lastFullRiseCurveCheck = currentTime;
    probeFullCurve(false, riseCurve, riseCurveSize, riseCoefficients, riseCurveFitError);
  }

  // Control the PWM to maintain voltage within bounds
  controlPWM();
}

void measureNoLoadVoltage() {
  float voltageSum = 0.0;
  const int samples = 10;
  const int maxMeasurementTime = 100; // ms
  unsigned long start = millis();
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    float currentSample = analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
    
    // Add some basic noise filtering
    if (currentSample > 0 && currentSample < maxNoLoadVoltage) {
      voltageSum += currentSample;
      validSamples++;
    }

    // Break if measurement time exceeds limit
    if (millis() - start > maxMeasurementTime) break;
  }

  noLoadVoltage = validSamples > 0 ? voltageSum / validSamples : noLoadVoltage;
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

void probeFullCurve(bool isDrop, CurveData *curve, int &size, float *coefficients, float &fitError) {
  Serial.println(isDrop ? "Probing full voltage drop curve..." : "Probing full voltage rise curve...");
  size = 0;

  pwmValue = isDrop ? pwmMaxValue : 0;
  ledcWrite(0, pwmValue);

  for (int i = 0; i < curveSize; i++) {
    unsigned long timestamp = millis();
    curve[i].voltage = analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
    curve[i].timestamp = timestamp;
    size++;
    delay(50);

    if (isDrop && curve[i].voltage < minAllowedVoltage) break;
  }

  pwmValue = 0;
  ledcWrite(0, pwmValue);

  // Fit curve and calculate error
  fitCurve(curve, size, polynomialDegree, coefficients, fitError);
  Serial.println(isDrop ? "Voltage drop curve fitted." : "Voltage rise curve fitted.");
  Serial.print("Curve fit error: ");
  Serial.println(fitError);
}

void fitCurve(const CurveData *curve, int size, int degree, float *coefficients, float &fitError) {
  // Use Eigen library for matrix-based least squares fitting
  Eigen::MatrixXf A(size, degree + 1);
  Eigen::VectorXf b(size);

  // Prepare matrices for least squares
  for (int i = 0; i < size; i++) {
    float x = curve[i].timestamp / 1000.0; // Convert to seconds
    float y = curve[i].voltage;
    
    for (int j = 0; j <= degree; j++) {
      A(i, j) = pow(x, j);
    }
    b(i) = y;
  }

  // Solve least squares problem
  Eigen::VectorXf x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  // Copy coefficients
  for (int i = 0; i <= degree; i++) {
    coefficients[i] = x(i);
  }

  // Calculate fitting error
  fitError = calculateRMSE(curve, size, coefficients, degree);
}

float calculateRMSE(const CurveData *curve, int size, const float *coefficients, int degree) {
  float sumSquaredError = 0.0;
  
  for (int i = 0; i < size; i++) {
    float x = curve[i].timestamp / 1000.0; // Convert to seconds
    float predicted = evaluateCurve(x, coefficients, degree);
    float error = predicted - curve[i].voltage;
    sumSquaredError += error * error;
  }

  return sqrt(sumSquaredError / size);
}

float evaluateCurve(float x, const float *coefficients, int degree) {
  float result = 0.0;
  for (int i = 0; i <= degree; i++) {
    result += coefficients[i] * pow(x, i);
  }
  return result;
}

void controlPWM() {
  // More advanced PWM control using fitted curves and fit error
  float predictedVoltage = evaluateCurve(pwmValue / pwmMaxValue, dropCoefficients, polynomialDegree);
  
  // Adjust PWM based on multiple conditions
  if (predictedVoltage < minAllowedVoltage || dropCurveFitError > 1.0) {
    pwmValue = max(pwmValue - 10, 0);
  } else if (currentVoltage < 0.8 * estimatedNoLoadVoltage) {
    pwmValue = min(pwmValue + 10, pwmMaxValue);
  }

  // Implement safety limits
  pwmValue = constrain(pwmValue, 0, pwmMaxValue);
  ledcWrite(0, pwmValue);

  // Log PWM adjustment for debugging
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);
}

void updateIntervals() {
  // Dynamically adjust intervals based on voltage stability and curve fit error
  float stabilityFactor = abs(voltageDropRate) + abs(voltageRiseRate);
  
  // Adjust voltage check interval
  voltageCheckInterval = constrain(
    100 * (1 + stabilityFactor), 
    50,   // Minimum interval
    500   // Maximum interval
  );

  // Adjust curve probing intervals based on fit error
  float dropFitErrorFactor = constrain(dropCurveFitError, 0.1, 2.0);
  float riseFitErrorFactor = constrain(riseCurveFitError, 0.1, 2.0);

  fullDropCurveInterval = constrain(
    10000 * dropFitErrorFactor, 
    5000,   // Minimum interval
    30000   // Maximum interval
  );

  fullRiseCurveInterval = constrain(
    10000 * riseFitErrorFactor, 
    5000,   // Minimum interval
    30000   // Maximum interval
  );
}
