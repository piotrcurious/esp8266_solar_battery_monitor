#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleKalmanFilter.h>

// Pin definitions
const int PWM_PIN = 25;
const int TEMP_SENSOR_PIN = 4;

// PWM configuration
const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;

// Battery parameters
const float CHARGING_THRESHOLD = 12.8;
const float INITIAL_TEST_CURRENT = 0.1;
const float FLOAT_VOLTAGE_NOMINAL = 13.5;
const float TEMP_COEFFICIENT = -0.003;
const float FLOAT_VOLTAGE_WINDOW = 0.05;
const float OUTGAS_VOLTAGE_RATE = 0.1;
const float OUTGAS_PLATEAU_THRESHOLD = 0.01;

// Current control parameters
const float CURRENT_STEP = 0.01;
const float CURRENT_TOLERANCE = 0.005;
const int CURRENT_SETTLE_TIME = 100;
const float KP = 0.1;
const float KI = 0.05;

// Safeguards
const float MAX_CURRENT = 5.0;
const float MAX_VOLTAGE = 15.0;
const unsigned long MAX_TEST_DURATION = 3600000;  // 1 hour
const unsigned long CHARGING_WAIT_TIME = 3600000;  // 1 hour
const unsigned long CURRENT_ADJUST_TIMEOUT = 10000;  // 10 seconds

// Kalman filter parameters
const float MEASUREMENT_UNCERTAINTY = 0.01;
const float ESTIMATION_UNCERTAINTY = 0.01;
const float PROCESS_NOISE = 0.01;

// Test configuration
const int NUM_TESTS = 10;

// Global objects
Adafruit_INA219 ina219;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
SimpleKalmanFilter voltageKalman(MEASUREMENT_UNCERTAINTY, ESTIMATION_UNCERTAINTY, PROCESS_NOISE);
SimpleKalmanFilter currentKalman(MEASUREMENT_UNCERTAINTY, ESTIMATION_UNCERTAINTY, PROCESS_NOISE);

// Global variables
float batteryVoltage = 0.0;
float currentSetpoint = INITIAL_TEST_CURRENT;
float batteryTemperature = 0.0;
float integralError = 0.0;

// Struct to hold test results
struct TestResult {
    float floatVoltage;
    float outgassingVoltage;
    float minFloatCurrent;
    float minOutgassingCurrent;
    bool success;
};

void setup() {
    Serial.begin(115200);
    initializeHardware();
}

void loop() {
    if (waitForChargingComplete()) {
        runBatteryTests();
    }
    Serial.println("All tests completed. Restarting in 1 hour.");
    delay(3600000);  // Wait for 1 hour before restarting tests
}

void initializeHardware() {
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

bool waitForChargingComplete() {
    Serial.println("Waiting for charging to complete...");
    unsigned long startTime = millis();
    while (true) {
        batteryVoltage = getFilteredVoltage();
        if (batteryVoltage < CHARGING_THRESHOLD) {
            if (millis() - startTime >= CHARGING_WAIT_TIME) {
                Serial.println("Charging complete. Starting tests.");
                return true;
            }
        } else {
            startTime = millis();  // Reset timer if voltage goes above threshold
        }
        if (millis() - startTime >= MAX_TEST_DURATION) {
            Serial.println("Timeout waiting for charging to complete.");
            return false;
        }
        delay(1000);  // Check every second
    }
}

void runBatteryTests() {
    for (int i = 0; i < NUM_TESTS; i++) {
        Serial.printf("Starting Test #%d\n", i + 1);
        TestResult result = performBatteryTest();
        if (result.success) {
            printTestResults(result);
        } else {
            Serial.println("Test aborted due to timeout or safety limits");
        }
        delay(5000);  // Wait between tests
    }
}

TestResult performBatteryTest() {
    TestResult result = {0, 0, 0, 0, false};
    unsigned long testStartTime = millis();
    
    float compensatedFloatVoltage = getTemperatureCompensatedFloatVoltage();
    
    if (!findFloatVoltage(compensatedFloatVoltage, result, testStartTime)) return result;
    if (!findOutgassingVoltage(result, testStartTime)) return result;
    if (!determineMinimumCurrents(result, testStartTime)) return result;
    
    result.success = true;
    return result;
}

bool findFloatVoltage(float compensatedFloatVoltage, TestResult& result, unsigned long testStartTime) {
    float lowerFloatThreshold = compensatedFloatVoltage - FLOAT_VOLTAGE_WINDOW / 2;
    float upperFloatThreshold = compensatedFloatVoltage + FLOAT_VOLTAGE_WINDOW / 2;
    
    while (batteryVoltage < lowerFloatThreshold) {
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        currentSetpoint += CURRENT_STEP;
        if (isTestLimitExceeded(testStartTime)) return false;
        delay(CURRENT_SETTLE_TIME);
    }
    
    if (batteryVoltage <= upperFloatThreshold) {
        result.floatVoltage = batteryVoltage;
        Serial.println("Float voltage found");
        return true;
    } else {
        Serial.println("Float voltage not found within expected range");
        return false;
    }
}

bool findOutgassingVoltage(TestResult& result, unsigned long testStartTime) {
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
            outgassingDetected = waitForOutgassingPlateau(lastVoltage, result, testStartTime);
        }
        
        currentSetpoint += CURRENT_STEP;
        if (isTestLimitExceeded(testStartTime)) return false;
        delay(CURRENT_SETTLE_TIME);
    }
    
    setCurrentWithFeedback(0);  // Stop current
    return outgassingDetected;
}

bool waitForOutgassingPlateau(float& lastVoltage, TestResult& result, unsigned long testStartTime) {
    unsigned long plateauStartTime = millis();
    while ((millis() - plateauStartTime) < 3600000) {  // Check for 1 hour
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        float plateauRate = (batteryVoltage - lastVoltage) / ((millis() - plateauStartTime) / 3600000.0);
        
        if (plateauRate < OUTGAS_PLATEAU_THRESHOLD) {
            result.outgassingVoltage = batteryVoltage;
            return true;
        }
        
        lastVoltage = batteryVoltage;
        
        if (isTestLimitExceeded(testStartTime)) return false;
        
        delay(60000);  // Check every minute
    }
    return false;
}

bool determineMinimumCurrents(TestResult& result, unsigned long testStartTime) {
    // Determine minimum float current
    while (batteryVoltage >= result.floatVoltage - FLOAT_VOLTAGE_WINDOW / 2) {
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        currentSetpoint -= CURRENT_STEP;
        if (currentSetpoint < 0 || isTestLimitExceeded(testStartTime)) break;
        delay(CURRENT_SETTLE_TIME);
    }
    result.minFloatCurrent = currentSetpoint + CURRENT_STEP;
    
    // Reset current for outgassing test
    currentSetpoint = INITIAL_TEST_CURRENT;
    
    // Determine minimum outgassing current
    while (batteryVoltage < result.outgassingVoltage) {
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        currentSetpoint += CURRENT_STEP;
        if (isTestLimitExceeded(testStartTime)) break;
        delay(CURRENT_SETTLE_TIME);
    }
    result.minOutgassingCurrent = currentSetpoint;
    
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
        
        float proportional = KP * currentError;
        integralError += KI * currentError;
        
        pwmValue += proportional + integralError;
        pwmValue = constrain(pwmValue, 0, 255);
        
        ledcWrite(PWM_CHANNEL, pwmValue);
        
        if (isCurrentAdjustmentLimitExceeded(startTime, measuredCurrent)) return false;
        
        delay(10);  // Small delay for system to respond
    } while (abs(currentError) > CURRENT_TOLERANCE);
    
    return true;
}

float getTemperatureCompensatedFloatVoltage() {
    tempSensor.requestTemperatures();
    batteryTemperature = tempSensor.getTempCByIndex(0);
    return FLOAT_VOLTAGE_NOMINAL + TEMP_COEFFICIENT * (batteryTemperature - 25.0);
}

float getFilteredVoltage() {
    float rawVoltage = ina219.getBusVoltage_V();
    return voltageKalman.updateEstimate(rawVoltage);
}

float getFilteredCurrent() {
    float rawCurrent = ina219.getCurrent_mA() / 1000.0;  // Convert mA to A
    return currentKalman.updateEstimate(rawCurrent);
}

bool isTestLimitExceeded(unsigned long testStartTime) {
    return (currentSetpoint > MAX_CURRENT || 
            batteryVoltage > MAX_VOLTAGE || 
            (millis() - testStartTime) > MAX_TEST_DURATION);
}

bool isCurrentAdjustmentLimitExceeded(unsigned long startTime, float measuredCurrent) {
    return (measuredCurrent > MAX_CURRENT || 
            getFilteredVoltage() > MAX_VOLTAGE || 
            (millis() - startTime) > CURRENT_ADJUST_TIMEOUT);
}

void printTestResults(const TestResult& result) {
    Serial.println("Test Results:");
    Serial.printf("Float Voltage: %.3f V\n", result.floatVoltage);
    Serial.printf("Outgassing Voltage: %.3f V\n", result.outgassingVoltage);
    Serial.printf("Minimum Float Current: %.3f A\n", result.minFloatCurrent);
    Serial.printf("Minimum Outgassing Current: %.3f A\n", result.minOutgassingCurrent);
}
