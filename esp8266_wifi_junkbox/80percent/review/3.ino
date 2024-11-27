#include <Arduino.h>
#include <math.h>
#include <ArduinoEigen.h>
#include <MovingAverage.h>  // For smoothing sensor readings

// Configuration Macros
#define DEBUG_MODE 1  // Enable/disable debug logging
#define SAFETY_TIMEOUT 5000  // 5 seconds safety timeout
#define MAX_VOLTAGE_DEVIATION 1.5  // Maximum allowed voltage deviation

// Enum for system states
enum SystemState {
  STATE_INIT,
  STATE_NORMAL,
  STATE_FAULT,
  STATE_RECOVERY
};

// Advanced Voltage Control Class
class VoltageController {
private:
  // Hardware Configuration
  const int pwmPin;
  const int voltagePin;
  
  // Configuration Constants
  const float maxNoLoadVoltage;
  const float minAllowedVoltage;
  const int pwmFrequency;
  const int pwmResolution;
  
  // State Variables
  SystemState currentState;
  float noLoadVoltage;
  float estimatedNoLoadVoltage;
  float currentVoltage;
  float previousVoltage;
  float voltageDropRate;
  float voltageRiseRate;
  int pwmValue;
  unsigned long lastStateChangeTime;

  // Moving Average Filters
  MovingAverage<float, 10> voltageFilter;
  MovingAverage<float, 5> dropRateFilter;
  MovingAverage<float, 5> riseRateFilter;

  // Curve Fitting Data
  static const int curveSize = 100;
  static const int polynomialDegree = 2;
  
  struct CurveData {
    float voltage;
    unsigned long timestamp;
  };
  
  CurveData dropCurve[curveSize];
  CurveData riseCurve[curveSize];
  int dropCurveSize = 0;
  int riseCurveSize = 0;
  
  float dropCoefficients[polynomialDegree + 1] = {0};
  float riseCoefficients[polynomialDegree + 1] = {0};
  float dropCurveFitError = 0.0;
  float riseCurveFitError = 0.0;

  // Internal Methods
  void logDebug(const String& message) {
    #if DEBUG_MODE
      Serial.println(message);
    #endif
  }

  void measureNoLoadVoltage() {
    const int samples = 20;
    const int maxMeasurementTime = 200; // ms
    float voltageSum = 0.0;
    unsigned long start = millis();
    int validSamples = 0;

    for (int i = 0; i < samples; i++) {
      float currentSample = readVoltage();
      
      // Advanced noise filtering
      if (currentSample > 0 && currentSample < maxNoLoadVoltage) {
        voltageSum += currentSample;
        validSamples++;
      }

      if (millis() - start > maxMeasurementTime) break;
    }

    noLoadVoltage = validSamples > 0 
      ? voltageSum / validSamples 
      : noLoadVoltage;
    
    logDebug("No-load voltage: " + String(noLoadVoltage, 2) + "V");
  }

  float readVoltage() {
    // Enhanced voltage reading with optional voltage divider compensation
    return analogRead(voltagePin) * (maxNoLoadVoltage / 1023.0);
  }

  void probeVoltageChangeRate() {
    float currentRate = (currentVoltage - previousVoltage) / 0.1;  // 100ms interval
    
    if (pwmValue > 0) {
      voltageDropRate = dropRateFilter.add(currentRate);
      logDebug("Voltage drop rate: " + String(voltageDropRate, 3));
    } else {
      voltageRiseRate = -riseRateFilter.add(currentRate);
      logDebug("Voltage rise rate: " + String(voltageRiseRate, 3));
    }
  }

  void fitCurve(const CurveData *curve, int size, float *coefficients, float &fitError) {
    // Robust curve fitting with error handling
    if (size <= polynomialDegree + 1) {
      logDebug("Insufficient data points for curve fitting");
      return;
    }

    MatrixXf A(size, polynomialDegree + 1);
    VectorXf b(size);

    for (int i = 0; i < size; i++) {
      float x = curve[i].timestamp / 1000.0;
      float y = curve[i].voltage;
      
      for (int j = 0; j <= polynomialDegree; j++) {
        A(i, j) = pow(x, j);
      }
      b(i) = y;
    }

    // Solve using Singular Value Decomposition
    VectorXf x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

    // Copy and validate coefficients
    for (int i = 0; i <= polynomialDegree; i++) {
      coefficients[i] = isfinite(x(i)) ? x(i) : 0.0;
    }

    // Calculate Root Mean Square Error
    float sumSquaredError = 0.0;
    for (int i = 0; i < size; i++) {
      float x = curve[i].timestamp / 1000.0;
      float predicted = evaluateCurve(x, coefficients, polynomialDegree);
      float error = predicted - curve[i].voltage;
      sumSquaredError += error * error;
    }
    
    fitError = sqrt(sumSquaredError / size);
    logDebug("Curve fit error: " + String(fitError, 4));
  }

  float evaluateCurve(float x, const float *coefficients, int degree) {
    float result = 0.0;
    for (int i = 0; i <= degree; i++) {
      result += coefficients[i] * pow(x, i);
    }
    return result;
  }

  void updateSystemState() {
    unsigned long currentTime = millis();

    // State transition logic
    switch (currentState) {
      case STATE_INIT:
        if (abs(currentVoltage - noLoadVoltage) < MAX_VOLTAGE_DEVIATION) {
          currentState = STATE_NORMAL;
          lastStateChangeTime = currentTime;
        }
        break;

      case STATE_NORMAL:
        if (abs(currentVoltage - estimatedNoLoadVoltage) > MAX_VOLTAGE_DEVIATION) {
          currentState = STATE_FAULT;
          lastStateChangeTime = currentTime;
        }
        break;

      case STATE_FAULT:
        if (currentTime - lastStateChangeTime > SAFETY_TIMEOUT) {
          currentState = STATE_RECOVERY;
          lastStateChangeTime = currentTime;
        }
        break;

      case STATE_RECOVERY:
        if (abs(currentVoltage - estimatedNoLoadVoltage) < MAX_VOLTAGE_DEVIATION / 2) {
          currentState = STATE_NORMAL;
          lastStateChangeTime = currentTime;
        }
        break;
    }
  }

  void controlPWM() {
    // Advanced PWM control with state-based logic
    switch (currentState) {
      case STATE_INIT:
        pwmValue = 0;  // Safe initialization
        break;

      case STATE_NORMAL:
        {
          float predictedVoltage = evaluateCurve(
            pwmValue / pow(2, pwmResolution - 1), 
            dropCoefficients, 
            polynomialDegree
          );
          
          if (predictedVoltage < minAllowedVoltage || dropCurveFitError > 1.0) {
            pwmValue = max(pwmValue - 15, 0);
          } else if (currentVoltage < 0.8 * estimatedNoLoadVoltage) {
            pwmValue = min(pwmValue + 15, static_cast<int>(pow(2, pwmResolution) - 1));
          }
        }
        break;

      case STATE_FAULT:
        pwmValue = 0;  // Emergency shutdown
        break;

      case STATE_RECOVERY:
        pwmValue = min(pwmValue + 10, static_cast<int>(pow(2, pwmResolution) - 1));
        break;
    }

    // Apply PWM with hardware-specific method
    ledcWrite(0, pwmValue);
    logDebug("PWM Value: " + String(pwmValue));
  }

public:
  // Constructor
  VoltageController(
    int pwmPin, 
    int voltagePin, 
    float maxVoltage = 13.8 * 1.25, 
    float minVoltage = 13.8
  ) : 
    pwmPin(pwmPin), 
    voltagePin(voltagePin),
    maxNoLoadVoltage(maxVoltage),
    minAllowedVoltage(minVoltage),
    pwmFrequency(5000),
    pwmResolution(8),
    currentState(STATE_INIT),
    noLoadVoltage(0),
    estimatedNoLoadVoltage(0),
    currentVoltage(0),
    previousVoltage(0),
    voltageDropRate(0),
    voltageRiseRate(0),
    pwmValue(0),
    lastStateChangeTime(0)
  {}

  void begin() {
    // Initialize hardware
    ledcSetup(0, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPin, 0);
    
    // Initial measurements
    measureNoLoadVoltage();
    estimatedNoLoadVoltage = noLoadVoltage;
    
    logDebug("Voltage Control System Initialized");
  }

  void update() {
    // Main update loop
    unsigned long currentTime = millis();
    
    // Read and filter voltage
    previousVoltage = currentVoltage;
    currentVoltage = voltageFilter.add(readVoltage());
    
    // Probe voltage change rates
    probeVoltageChangeRate();
    
    // Update system state
    updateSystemState();
    
    // Control PWM based on current state
    controlPWM();
  }
};

// Global instance
VoltageController voltageController(D1, A0);

void setup() {
  Serial.begin(115200);
  voltageController.begin();
}

void loop() {
  voltageController.update();
  delay(100);  // Adjust based on performance needs
}
