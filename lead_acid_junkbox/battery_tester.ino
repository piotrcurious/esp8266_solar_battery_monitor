//generated by Claude 
//simple battery tester to determine float and outgassing voltages and currents

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleKalmanFilter.h>

// Pin definitions
const int PWM_PIN = 25;  // PWM output pin
const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;
const int TEMP_SENSOR_PIN = 4;  // DS18B20 temperature sensor pin

// Battery parameters
const float CHARGING_THRESHOLD = 12.8;  // Voltage below which charging is considered complete
const float INITIAL_TEST_CURRENT = 0.1;  // Initial test current in Amps
const int CHARGING_WAIT_TIME = 3600000;  // 1 hour in milliseconds
const int NUM_TESTS = 10;  // Number of times to test for float and outgassing voltages
const float FLOAT_VOLTAGE_NOMINAL = 13.5;  // Nominal float voltage at 25°C
const float TEMP_COEFFICIENT = -0.003;  // Temperature coefficient in V/°C
const float FLOAT_VOLTAGE_WINDOW = 0.05;  // Acceptable window around float voltage
const float OUTGAS_VOLTAGE_RATE = 0.1;  // V/hour, rate of voltage increase for outgassing detection
const float OUTGAS_PLATEAU_THRESHOLD = 0.01;  // V/hour, threshold for detecting voltage plateau

// Current control parameters
const float CURRENT_STEP = 0.01;  // Current increase step in Amps
const float CURRENT_TOLERANCE = 0.005;  // Acceptable current error in Amps
const int CURRENT_SETTLE_TIME = 100;  // Time to wait for current to settle in ms
const float KP = 0.1;  // Proportional gain for current control
const float KI = 0.05;  // Integral gain for current control

// Safeguards
const float MAX_CURRENT = 5.0;  // Maximum allowable current in Amps
const float MAX_VOLTAGE = 15.0;  // Maximum allowable voltage in Volts
const unsigned long MAX_TEST_DURATION = 3600000;  // Maximum duration for a single test (1 hour)

// Kalman filter parameters
const float MEASUREMENT_UNCERTAINTY = 0.01;
const float ESTIMATION_UNCERTAINTY = 0.01;
const float PROCESS_NOISE = 0.01;

// Global variables
Adafruit_INA219 ina219;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
SimpleKalmanFilter voltageKalman(MEASUREMENT_UNCERTAINTY, ESTIMATION_UNCERTAINTY, PROCESS_NOISE);
SimpleKalmanFilter currentKalman(MEASUREMENT_UNCERTAINTY, ESTIMATION_UNCERTAINTY, PROCESS_NOISE);
float batteryVoltage = 0.0;
float currentSetpoint = INITIAL_TEST_CURRENT;
float floatVoltage = 0.0;
float outgassingVoltage = 0.0;
float batteryTemperature = 0.0;
float integralError = 0.0;
float minFloatCurrent = 0.0;
float minOutgassingCurrent = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  
  tempSensor.begin();
  
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  
  Serial.println("Battery Tester Initialized");
}

void loop() {
  waitForChargingComplete();
  
  for (int i = 0; i < NUM_TESTS; i++) {
    Serial.print("Test #");
    Serial.println(i + 1);
    
    if (!determineBatteryThresholds()) {
      Serial.println("Test aborted due to timeout or safety limits");
      continue;
    }
    
    Serial.print("Float Voltage: ");
    Serial.print(floatVoltage, 3);
    Serial.println(" V");
    
    Serial.print("Outgassing Voltage: ");
    Serial.print(outgassingVoltage, 3);
    Serial.println(" V");
    
    determineMinimumCurrents();
    
    Serial.print("Minimum Float Current: ");
    Serial.print(minFloatCurrent, 3);
    Serial.println(" A");
    
    Serial.print("Minimum Outgassing Current: ");
    Serial.print(minOutgassingCurrent, 3);
    Serial.println(" A");
    
    delay(5000);  // Wait between tests
  }
  
  Serial.println("All tests completed");
  while(1) { delay(1000); }  // Stop execution
}

void waitForChargingComplete() {
  Serial.println("Waiting for charging to complete...");
  unsigned long startTime = millis();
  bool chargingComplete = false;
  
  while (!chargingComplete) {
    batteryVoltage = getFilteredVoltage();
    
    if (batteryVoltage < CHARGING_THRESHOLD) {
      if (millis() - startTime >= CHARGING_WAIT_TIME) {
        chargingComplete = true;
      }
    } else {
      startTime = millis();  // Reset timer if voltage goes above threshold
    }
    
    delay(1000);  // Check every second
  }
  
  Serial.println("Charging complete. Starting tests.");
}

float getTemperatureCompensatedFloatVoltage() {
  tempSensor.requestTemperatures();
  batteryTemperature = tempSensor.getTempCByIndex(0);
  return FLOAT_VOLTAGE_NOMINAL + TEMP_COEFFICIENT * (batteryTemperature - 25.0);
}

bool determineBatteryThresholds() {
  unsigned long testStartTime = millis();
  float compensatedFloatVoltage = getTemperatureCompensatedFloatVoltage();
  float lowerFloatThreshold = compensatedFloatVoltage - FLOAT_VOLTAGE_WINDOW / 2;
  float upperFloatThreshold = compensatedFloatVoltage + FLOAT_VOLTAGE_WINDOW / 2;
  
  // Find float voltage
  while (batteryVoltage < lowerFloatThreshold) {
    if (!setCurrentWithFeedback(currentSetpoint)) return false;
    batteryVoltage = getFilteredVoltage();
    currentSetpoint += CURRENT_STEP;
    if (currentSetpoint > MAX_CURRENT || batteryVoltage > MAX_VOLTAGE || 
        (millis() - testStartTime) > MAX_TEST_DURATION) {
      return false;
    }
    delay(CURRENT_SETTLE_TIME);
  }
  
  if (batteryVoltage <= upperFloatThreshold) {
    floatVoltage = batteryVoltage;
    Serial.println("Float voltage found");
  } else {
    Serial.println("Float voltage not found within expected range");
    return false;
  }
  
  // Find outgassing voltage
  float initialVoltage = batteryVoltage;
  float lastVoltage = initialVoltage;
  unsigned long startTime = millis();
  bool outgassingDetected = false;
  
  while (!outgassingDetected) {
    if (!setCurrentWithFeedback(currentSetpoint)) return false;
    batteryVoltage = getFilteredVoltage();
    
    float elapsedHours = (millis() - startTime) / 3600000.0;
    float voltageRate = (batteryVoltage - initialVoltage) / elapsedHours;
    
    if (voltageRate >= OUTGAS_VOLTAGE_RATE) {
      // Hold current constant and wait for plateau
      unsigned long plateauStartTime = millis();
      while ((millis() - plateauStartTime) < 3600000) {  // Check for 1 hour
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        float plateauRate = (batteryVoltage - lastVoltage) / ((millis() - plateauStartTime) / 3600000.0);
        
        if (plateauRate < OUTGAS_PLATEAU_THRESHOLD) {
          outgassingVoltage = batteryVoltage;
          outgassingDetected = true;
          break;
        }
        
        lastVoltage = batteryVoltage;
        
        if (batteryVoltage > MAX_VOLTAGE || (millis() - testStartTime) > MAX_TEST_DURATION) {
          return false;
        }
        
        delay(60000);  // Check every minute
      }
    }
    
    currentSetpoint += CURRENT_STEP;
    if (currentSetpoint > MAX_CURRENT || batteryVoltage > MAX_VOLTAGE || 
        (millis() - testStartTime) > MAX_TEST_DURATION) {
      return false;
    }
    delay(CURRENT_SETTLE_TIME);
  }
  
  // Stop current
  setCurrentWithFeedback(0);
  return true;
}

bool setCurrentWithFeedback(float targetCurrent) {
  float currentError;
  float measuredCurrent;
  int pwmValue = 0;
  unsigned long startTime = millis();
  
  do {
    measuredCurrent = getFilteredCurrent();
    currentError = targetCurrent - measuredCurrent;
    
    // PI control
    float proportional = KP * currentError;
    integralError += KI * currentError;
    
    // Adjust PWM value
    pwmValue += proportional + integralError;
    pwmValue = constrain(pwmValue, 0, 255);
    
    ledcWrite(PWM_CHANNEL, pwmValue);
    
    if (measuredCurrent > MAX_CURRENT || getFilteredVoltage() > MAX_VOLTAGE || 
        (millis() - startTime) > 10000) {  // 10 second timeout for current adjustment
      return false;
    }
    
    delay(10);  // Small delay for system to respond
  } while (abs(currentError) > CURRENT_TOLERANCE);
  
  return true;
}

float getFilteredVoltage() {
  float rawVoltage = ina219.getBusVoltage_V();
  return voltageKalman.updateEstimate(rawVoltage);
}

float getFilteredCurrent() {
  float rawCurrent = ina219.getCurrent_mA() / 1000.0;  // Convert mA to A
  return currentKalman.updateEstimate(rawCurrent);
}

void determineMinimumCurrents() {
  // Determine minimum float current
  while (batteryVoltage >= floatVoltage - FLOAT_VOLTAGE_WINDOW / 2) {
    if (!setCurrentWithFeedback(currentSetpoint)) break;
    batteryVoltage = getFilteredVoltage();
    currentSetpoint -= CURRENT_STEP;
    if (currentSetpoint < 0) break;
    delay(CURRENT_SETTLE_TIME);
  }
  minFloatCurrent = currentSetpoint + CURRENT_STEP;
  
  // Reset current for outgassing test
  currentSetpoint = INITIAL_TEST_CURRENT;
  
  // Determine minimum outgassing current
  while (batteryVoltage < outgassingVoltage) {
    if (!setCurrentWithFeedback(currentSetpoint)) break;
    batteryVoltage = getFilteredVoltage();
    currentSetpoint += CURRENT_STEP;
    if (currentSetpoint > MAX_CURRENT) break;
    delay(CURRENT_SETTLE_TIME);
  }
  minOutgassingCurrent = currentSetpoint;
}