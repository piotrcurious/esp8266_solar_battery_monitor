// Pin definitions
#define PANEL_VOLTAGE_PIN 5
#define CONV1_VOLTAGE_PIN 4
#define CONV2_VOLTAGE_PIN 3
#define PRIORITY_PIN A0    // New analog pin for priority ratio

#define CONV1_PWM_PIN 23   // Higher priority converter
#define CONV2_PWM_PIN 24   // Lower priority converter

// Voltage measurement scaling factors
const float VREF = 30;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution

// PWM and control constants
const int PWM_MIN = 0;
const int PWM_MAX = 255;
const float Kp = 0.5;     // Proportional gain
const float Ki = 0.1;     // Integral gain
const float Kd = 0.05;    // Derivative gain

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float conv1Voltage, conv2Voltage;
float panelCurrent = 1.0;
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Error tracking
float errorSum = 0;         // For integral control
float lastError = 0;        // For derivative control
float maxErrorSum = 100;    // Anti-windup limit

// Priority control
float priorityRatio = 0.7;  // Default priority ratio (70-30 split)
const float MIN_PRIORITY = 0.1;  // Minimum priority for any converter

// Timing variables
unsigned long mpptInterval = 100;
unsigned long vocInterval = 30000;
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;
unsigned long lastPriorityCheck = 0;
const unsigned long PRIORITY_CHECK_INTERVAL = 1000;

// System state
float openCircuitVoltage = 0.0;
float targetVoltage;
float predictedVoc = 0.0;

// PID control for PWM adjustment
float calculatePIDOutput(float error) {
    float pTerm = Kp * error;
    
    // Integral term with anti-windup
    errorSum = constrain(errorSum + error, -maxErrorSum, maxErrorSum);
    float iTerm = Ki * errorSum;
    
    // Derivative term
    float dTerm = Kd * (error - lastError);
    lastError = error;
    
    return pTerm + iTerm + dTerm;
}

// Read and update priority ratio from analog input
void updatePriorityRatio() {
    int rawPriority = analogRead(PRIORITY_PIN);
    // Convert to ratio between MIN_PRIORITY and (1-MIN_PRIORITY)
    priorityRatio = MIN_PRIORITY + (1 - 2*MIN_PRIORITY) * (float)rawPriority / ADC_RES;
}

// Enhanced voltage reading with filtering
float readVoltage(int pin) {
    const int SAMPLES = 5;
    float sum = 0;
    for(int i = 0; i < SAMPLES; i++) {
        sum += (analogRead(pin) * VREF / ADC_RES);
        delayMicroseconds(100);
    }
    return sum / SAMPLES;
}

// Improved PWM adjustment with priority consideration
void adjustPWMWithPriority(float error) {
    float pidOutput = calculatePIDOutput(error);
    
    // Calculate PWM adjustments based on priority ratio
    float conv1Adjustment = pidOutput * priorityRatio;
    float conv2Adjustment = pidOutput * (1 - priorityRatio);
    
    // Apply adjustments with limits
    pwmConv1 = constrain(pwmConv1 + conv1Adjustment, PWM_MIN, PWM_MAX);
    pwmConv2 = constrain(pwmConv2 + conv2Adjustment, PWM_MIN, PWM_MAX);
    
    // Ensure minimum power handling capability
    if(pwmConv1 < PWM_MIN * 1.2) pwmConv1 = PWM_MIN;
    if(pwmConv2 < PWM_MIN * 1.2) pwmConv2 = PWM_MIN;
    
    analogWrite(CONV1_PWM_PIN, pwmConv1);
    analogWrite(CONV2_PWM_PIN, pwmConv2);
}

void measureOpenCircuitVoltage() {
    // Gradually reduce PWM to minimize voltage spikes
    for(int i = pwmConv1; i >= PWM_MIN; i--) {
        analogWrite(CONV1_PWM_PIN, i);
        delay(5);
    }
    for(int i = pwmConv2; i >= PWM_MIN; i--) {
        analogWrite(CONV2_PWM_PIN, i);
        delay(5);
    }
    
    delay(500);  // Allow voltage to stabilize
    openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN);
    targetVoltage = openCircuitVoltage * 0.80;
    
    float predictionError = fabs(openCircuitVoltage - predictedVoc);
    vocInterval = constrain(
        predictionError > 1.0 ? vocInterval / 2 : vocInterval * 1.5,
        10000, 60000
    );
    
    predictedVoc = openCircuitVoltage;
    
    Serial.print("Voc: "); Serial.println(openCircuitVoltage);
}

void performMpptAdjustment() {
    prevPower = power;
    prevPanelVoltage = panelVoltage;
    
    // Read all voltages
    panelVoltage = readVoltage(PANEL_VOLTAGE_PIN);
    conv1Voltage = readVoltage(CONV1_VOLTAGE_PIN);
    conv2Voltage = readVoltage(CONV2_VOLTAGE_PIN);
    
    // Calculate total power and error
    power = panelVoltage * panelCurrent;
    float voltageError = targetVoltage - panelVoltage;
    
    // Adjust PWM based on error and priority
    adjustPWMWithPriority(voltageError);
    
    // Dynamic interval adjustment based on voltage stability
    float voltageChange = fabs(panelVoltage - prevPanelVoltage);
    mpptInterval = constrain(
        voltageChange > 0.5 ? mpptInterval / 2 : mpptInterval * 1.5,
        10, 200
    );
    
    // Debug output
    Serial.print("V:"); Serial.print(panelVoltage);
    Serial.print(" P:"); Serial.print(power);
    Serial.print(" PWM1:"); Serial.print(pwmConv1);
    Serial.print(" PWM2:"); Serial.print(pwmConv2);
    Serial.print(" Pri:"); Serial.println(priorityRatio);
}

void setup() {
    pinMode(CONV1_PWM_PIN, OUTPUT);
    pinMode(CONV2_PWM_PIN, OUTPUT);
    pinMode(PRIORITY_PIN, INPUT);
    Serial.begin(115200);
    measureOpenCircuitVoltage();
}

void loop() {
    unsigned long currentMillis = millis();
    
    // Update priority ratio
    if(currentMillis - lastPriorityCheck >= PRIORITY_CHECK_INTERVAL) {
        updatePriorityRatio();
        lastPriorityCheck = currentMillis;
    }
    
    // Measure open-circuit voltage
    if(currentMillis - lastVocCheck >= vocInterval) {
        measureOpenCircuitVoltage();
        lastVocCheck = currentMillis;
    }
    
    // Perform MPPT adjustment
    if(currentMillis - lastMpptCheck >= mpptInterval) {
        performMpptAdjustment();
        lastMpptCheck = currentMillis;
    }
}
