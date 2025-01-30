// Pin definitions
#define PANEL_VOLTAGE_PIN 5
#define CONV1_VOLTAGE_PIN 4
#define CONV2_VOLTAGE_PIN 3

#define CONV1_PWM_PIN 23  // Higher priority converter
#define CONV2_PWM_PIN 24  // Lower priority converter

// Voltage measurement scaling factors
const float VREF = 30.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023;  // 10-bit ADC resolution

// PWM limits
const int PWM_MIN = 0;
const int PWM_MAX = 255;
int PWM_STEP = 10;

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;  // Assuming constant current source (optional)
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Priority ratio (user-adjustable, e.g., 70% power to Conv1, 30% to Conv2)
float priorityRatio = 0.7;  

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;  
float predictedVoc = 0.0;  
float targetVoltage;

// Dynamic timing intervals
unsigned long mpptInterval = 100;  
unsigned long vocInterval = 30000;  
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;

// Function to read and scale voltage
float readVoltage(int pin) {
    int raw = analogRead(pin);
    return (raw * VREF / ADC_RES);
}

// Function to adjust PWM with limits
int adjustPWM(int pwm, int adjustment) {
    pwm += adjustment;
    return constrain(pwm, PWM_MIN, PWM_MAX);
}

// Measure open-circuit voltage
void measureOpenCircuitVoltage() {
    analogWrite(CONV1_PWM_PIN, PWM_MIN);
    analogWrite(CONV2_PWM_PIN, PWM_MIN);
    delay(1000);

    openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN);
    targetVoltage = openCircuitVoltage * 0.80;

    Serial.print("Measured Voc: ");
    Serial.println(openCircuitVoltage);

    float error = fabs(openCircuitVoltage - predictedVoc);
    if (error > 1.0) { 
        vocInterval = max(10000, vocInterval / 2);
    } else {
        vocInterval = min(60000, vocInterval * 2);
    }

    predictedVoc = openCircuitVoltage;
}

// Cost function to be minimized
float computeCost(float voltageError, float power) {
    return fabs(voltageError) + 0.01 * power;  
}

// Perform MPPT adjustment with Gradient Descent + Monte Carlo
void performMpptAdjustment() {
    prevPower = power;
    prevPanelVoltage = panelVoltage;

    panelVoltage = readVoltage(PANEL_VOLTAGE_PIN);
    power = panelVoltage * panelCurrent * ((float)pwmConv1 / PWM_MAX) + 
            panelVoltage * panelCurrent * ((float)pwmConv2 / PWM_MAX);

    float voltageError = panelVoltage - targetVoltage;
    float prevCost = computeCost(voltageError, prevPower);

    // Gradient Descent: Adjust PWM in the direction that minimizes cost
    int pwmChange1 = (voltageError < 0) ? PWM_STEP : -PWM_STEP;
    int pwmChange2 = -pwmChange1 * (1 - priorityRatio); 

    pwmConv1 = adjustPWM(pwmConv1, pwmChange1);
    pwmConv2 = adjustPWM(pwmConv2, pwmChange2);

    float newCost = computeCost(panelVoltage - targetVoltage, power);

    // Monte Carlo: Introduce a random perturbation if stuck
    if (newCost > prevCost) {
        pwmConv1 = adjustPWM(pwmConv1, random(-PWM_STEP, PWM_STEP));
        pwmConv2 = adjustPWM(pwmConv2, random(-PWM_STEP, PWM_STEP));
    }

    analogWrite(CONV1_PWM_PIN, pwmConv1);
    analogWrite(CONV2_PWM_PIN, pwmConv2);

    Serial.print("Vin: ");
    Serial.print(panelVoltage);
    Serial.print(", Power: ");
    Serial.print(power);
    Serial.print(", PWM1: ");
    Serial.print(pwmConv1);
    Serial.print(", PWM2: ");
    Serial.println(pwmConv2);

    // Adaptive MPPT timing
    float voltageChange = fabs(panelVoltage - prevPanelVoltage);
    if (fabs(panelVoltage - targetVoltage) > 2) {
        PWM_STEP = min(8, PWM_STEP * fabs(panelVoltage - targetVoltage));  
    } else {
        PWM_STEP = max(1, PWM_STEP / 2);  
    }

    mpptInterval = (voltageChange > 0.5) ? max(10, mpptInterval / 2) : min(100, mpptInterval * 2);
}

void setup() {
    pinMode(CONV1_PWM_PIN, OUTPUT);
    pinMode(CONV2_PWM_PIN, OUTPUT);
    Serial.begin(115200);
    measureOpenCircuitVoltage();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastVocCheck >= vocInterval) {
        measureOpenCircuitVoltage();
        lastVocCheck = currentMillis;
    }

    if (currentMillis - lastMpptCheck >= mpptInterval) {
        performMpptAdjustment();
        lastMpptCheck = currentMillis;
    }
}
