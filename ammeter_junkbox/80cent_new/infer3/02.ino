#include <Arduino.h>
#include <EEPROM.h>

// Configuration and Calibration
struct SolarConfig {
  float vocCalibration;
  float tempCoefficient;
  float pidParameters[3];  // Kp, Ki, Kd
  float voltageTargetRatio;
};

// Advanced logging and debugging
enum LogLevel {
  LOG_ERROR = 0,
  LOG_WARNING = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3
};

class SolarMPPTController {
private:
  // Pin Definitions
  static const int PIN_PWM1 = 9;
  static const int PIN_PWM2 = 10;
  static const int PIN_PANEL_VOLTAGE = A0;
  static const int PIN_OUTPUT_VOLTAGE1 = A1;
  static const int PIN_OUTPUT_VOLTAGE2 = A2;
  static const int PIN_TEMPERATURE = A3;

  // System State Variables
  float panelVoc = 0.0;
  float targetPanelVoltage = 0.0;
  int dutyCycle1 = 0;
  int dutyCycle2 = 0;

  // Configuration Parameters
  SolarConfig config;
  LogLevel currentLogLevel = LOG_WARNING;

  // Advanced Filtering
  static const int MOVING_AVG_WINDOW = 10;
  float voltageBuffer[MOVING_AVG_WINDOW] = {0};
  int bufferIndex = 0;

  // MPPT and Control Variables
  float previousPower = 0.0;
  float previousVoltage = 0.0;
  float integralError = 0.0;
  float lastError = 0.0;

  // Adaptive Parameters
  float weight_dVdD = 0.5;
  float weight_vocTrend = 0.3;
  float weight_tempComp = 0.2;

  // Performance Tracking
  unsigned long systemUptime = 0;
  unsigned long mpptCycles = 0;
  float totalEnergyGenerated = 0.0;

  // Private Utility Methods
  float readVoltage(int pin) {
    return analogRead(pin) * (5.0 / 1023.0);
  }

  float readTemperature() {
    return (readVoltage(PIN_TEMPERATURE) - 0.5) * 100.0;
  }

  void log(LogLevel level, const String& message) {
    if (level <= currentLogLevel) {
      Serial.println(message);
    }
  }

  float movingAverageFilter(float newValue) {
    voltageBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % MOVING_AVG_WINDOW;

    float sum = 0.0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
      sum += voltageBuffer[i];
    }
    return sum / MOVING_AVG_WINDOW;
  }

  void loadConfigFromEEPROM() {
    EEPROM.get(0, config);
    
    // Validate configuration or use defaults
    if (isnan(config.vocCalibration) || config.vocCalibration <= 0) {
      config.vocCalibration = 20.0;  // Default open-circuit voltage
      config.tempCoefficient = -0.003;
      config.pidParameters[0] = 1.2;  // Kp
      config.pidParameters[1] = 0.05; // Ki
      config.pidParameters[2] = 0.02; // Kd
      config.voltageTargetRatio = 0.80;
    }
  }

  void saveConfigToEEPROM() {
    EEPROM.put(0, config);
  }

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

  float estimateCurrent(float currentVoltage, float dutyCycle) {
    float dVdD = (previousVoltage - currentVoltage) / 
                 (max(1.0, previousDutyCycle - dutyCycle));
    float vocTrend = (panelVoc - currentVoltage) / panelVoc;
    float tempEffect = (readTemperature() - 25.0) * 
                       config.tempCoefficient * panelVoc;
    
    return (dVdD * weight_dVdD) + 
           (vocTrend * weight_vocTrend) + 
           (tempEffect * weight_tempComp);
  }

  void updateVoc(float panelVoltage) {
    if (panelVoltage > panelVoc) {
      panelVoc = panelVoltage;
    } else {
      panelVoc *= 0.999; 
    }
    targetPanelVoltage = panelVoc * config.voltageTargetRatio;
  }

  void adjustPWM_PID(float error) {
    const float Kp = config.pidParameters[0];
    const float Ki = config.pidParameters[1];
    const float Kd = config.pidParameters[2];

    integralError += error;
    float derivative = error - lastError;
    float output = (Kp * error) + (Ki * integralError) + (Kd * derivative);

    dutyCycle1 = constrain(dutyCycle1 + output, 0, 255);
    dutyCycle2 = constrain(dutyCycle2 + output, 0, 255);
    
    lastError = error;
  }

  void balanceLoad(float inferredCurrent) {
    const float LOAD_BALANCE_THRESHOLD = 2.0;
    if (inferredCurrent > LOAD_BALANCE_THRESHOLD) {
      dutyCycle1 = constrain(dutyCycle1 + 10, 0, 255);
      dutyCycle2 = constrain(dutyCycle2 - 10, 0, 255);
    } else {
      dutyCycle1 = constrain(dutyCycle1 - 10, 0, 255);
      dutyCycle2 = constrain(dutyCycle2 + 10, 0, 255);
    }
  }

  void enterLowPowerMode(float panelVoltage) {
    if (panelVoltage < (panelVoc * 0.5)) {
      dutyCycle1 = 0;
      dutyCycle2 = 0;
      log(LOG_INFO, "Low Power Mode Activated");
    }
  }

public:
  SolarMPPTController() {
    loadConfigFromEEPROM();
  }

  void initialize() {
    pinMode(PIN_PWM1, OUTPUT);
    pinMode(PIN_PWM2, OUTPUT);
    Serial.begin(115200);  // Higher baud rate for better performance
    log(LOG_INFO, "MPPT Solar Controller Initialized");
  }

  void run() {
    float panelVoltage = readVoltage(PIN_PANEL_VOLTAGE);
    float filteredPanelVoltage = movingAverageFilter(panelVoltage);

    updateVoc(filteredPanelVoltage);
    updateWeightingFactors(filteredPanelVoltage);

    float inferredCurrent = estimateCurrent(filteredPanelVoltage, dutyCycle1 + dutyCycle2);
    float voltageError = targetPanelVoltage - filteredPanelVoltage;

    adjustPWM_PID(voltageError);
    balanceLoad(inferredCurrent);
    enterLowPowerMode(filteredPanelVoltage);

    analogWrite(PIN_PWM1, dutyCycle1);
    analogWrite(PIN_PWM2, dutyCycle2);

    // Performance tracking
    mpptCycles++;
    systemUptime += millis();
    totalEnergyGenerated += panelVoltage * inferredCurrent;

    // Optional detailed logging
    if (currentLogLevel >= LOG_DEBUG) {
      Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
      Serial.print("V, Duty1: "); Serial.print(dutyCycle1);
      Serial.print(", Duty2: "); Serial.println(dutyCycle2);
    }
  }

  void printPerformanceReport() {
    Serial.println("--- MPPT Performance Report ---");
    Serial.print("System Uptime: "); Serial.print(systemUptime / 1000.0); Serial.println(" seconds");
    Serial.print("MPPT Cycles: "); Serial.println(mpptCycles);
    Serial.print("Total Energy Generated: "); Serial.print(totalEnergyGenerated); Serial.println(" Wh");
    Serial.print("Open Circuit Voltage: "); Serial.print(panelVoc); Serial.println(" V");
  }

  void setLogLevel(LogLevel level) {
    currentLogLevel = level;
  }
};

// Global instance
SolarMPPTController solarController;

void setup() {
  solarController.initialize();
  solarController.setLogLevel(LOG_DEBUG);
}

void loop() {
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 500;  // 500ms update cycle

  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;
    solarController.run();
  }

  // Optional: Print performance report every 5 minutes
  if (currentTime % 300000 == 0) {
    solarController.printPerformanceReport();
  }
}
