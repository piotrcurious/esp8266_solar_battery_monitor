// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_VOLTAGE_PIN A1
#define CONV2_VOLTAGE_PIN A2

#define CONV1_PWM_PIN 9   // Higher priority converter
#define CONV2_PWM_PIN 10  // Lower priority converter

// Voltage measurement scaling factors
const float VREF = 5.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 6.0; // Voltage divider ratio

// PWM limits
const int PWM_MIN = 0;
int PWM_MAX = 255;
int PWM_STEP = 10;

// Frequency range
const int FREQ_MIN = 1000;
const int FREQ_MAX = 20000;
const int FREQ_STEP = 1000;
const int FREQ_ENTRIES = (FREQ_MAX - FREQ_MIN) / FREQ_STEP + 1;

int pwmFrequency = FREQ_MIN;

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;  // Assuming constant current source
float power, prevPower, maxPower = 0;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;
float targetVoltage = 0.0;
const float TARGET_VOLTAGE_GAIN = 0.1; // Adjust gain for smoother tracking

// Dynamic timing intervals
unsigned long mpptInterval = 100;
unsigned long vocInterval = 30000;
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;

// Function to set PWM frequency (Timer1 on pins 9 & 10 for Uno)
void setPWMFrequency(int frequency) {
    int prescaler;
    int topValue;

    if (frequency >= 16000) {
        prescaler = 1; topValue = F_CPU / (2 * frequency) - 1;
    } else if (frequency >= 4000) {
        prescaler = 8; topValue = F_CPU / (2 * 8 * frequency) - 1;
    } else {
        prescaler = 64; topValue = F_CPU / (2 * 64 * frequency) - 1;
    }

    TCCR1B = (TCCR1B & 0b11111000) | prescaler;
    ICR1 = topValue;

    PWM_MAX = topValue;
    PWM_STEP = max(1, topValue / 20);
}

// Function to read and scale voltage
float readVoltage(int pin, float ratio) {
    int raw = analogRead(pin);
    return (raw * VREF / ADC_RES) * ratio;
}

// Function to adjust PWM with limits
int adjustPWM(int pwm, int adjustment) {
    pwm += adjustment;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    return pwm;
}

// Measure open-circuit voltage by disabling loads temporarily
void measureOpenCircuitVoltage() {
    analogWrite(CONV1_PWM_PIN, PWM_MIN);
    analogWrite(CONV2_PWM_PIN, PWM_MIN);
    delay(1000);

    float measuredVoc = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);

    // Update target voltage with exponential moving average
    targetVoltage = TARGET_VOLTAGE_GAIN * (measuredVoc * 0.80) + (1 - TARGET_VOLTAGE_GAIN) * targetVoltage;

    Serial.print("Measured Voc: ");
    Serial.print(measuredVoc);
    Serial.print(" V, Updated Target: ");
    Serial.println(targetVoltage);
}

// Perform Perturb & Observe MPPT tracking
void performMpptAdjustment() {
    prevPower = power;
    prevPanelVoltage = panelVoltage;

    panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
    power = panelVoltage * panelCurrent * ((float)pwmConv1 / PWM_MAX) +
            panelVoltage * panelCurrent * ((float)pwmConv2 / PWM_MAX);

    Serial.print("Panel Voltage: ");
    Serial.print(panelVoltage);
    Serial.print(" V, Target: ");
    Serial.print(targetVoltage);
    Serial.print(" V, Power: ");
    Serial.println(power);

    float voltageMargin = 0.05 * targetVoltage; // 5% margin for stability

    if (panelVoltage < targetVoltage - voltageMargin) {
        if (power > prevPower) {
            pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
        } else {
            pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
        }
    } else if (panelVoltage > targetVoltage + voltageMargin) {
        pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    }

    if (pwmConv1 >= PWM_MAX) {
        pwmConv2 = adjustPWM(pwmConv2, PWM_STEP);
    }

    analogWrite(CONV1_PWM_PIN, pwmConv1);
    analogWrite(CONV2_PWM_PIN, pwmConv2);

    Serial.print(" PWM Conv1: ");
    Serial.print(pwmConv1);
    Serial.print(" PWM Conv2: ");
    Serial.println(pwmConv2);

    // Adjust MPPT interval dynamically based on voltage fluctuations
    float voltageChange = fabs(panelVoltage - prevPanelVoltage);
    if (voltageChange > 0.5) {
        mpptInterval = max(10, mpptInterval / 2);
    } else {
        mpptInterval = min(200, mpptInterval * 2);
    }
}

void setup() {
    pinMode(CONV1_PWM_PIN, OUTPUT);
    pinMode(CONV2_PWM_PIN, OUTPUT);
    Serial.begin(115200);

    setPWMFrequency(pwmFrequency);
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
