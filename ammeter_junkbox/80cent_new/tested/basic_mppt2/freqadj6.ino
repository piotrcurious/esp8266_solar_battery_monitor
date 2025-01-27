// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_VOLTAGE_PIN A1
#define CONV2_VOLTAGE_PIN A2

#define CONV1_PWM_PIN 9  // Higher priority converter
#define CONV2_PWM_PIN 10 // Lower priority converter

// Voltage measurement scaling factors
const float VREF = 5.0;  // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 6.0; // Voltage divider ratio

// PWM frequency range
const int PWM_FREQ_MIN = 1000;
const int PWM_FREQ_MAX = 20000;
int pwmFrequency = 5000; // Initial frequency

// PWM limits
int PWM_MAX = 255; 
int PWM_STEP = 10;

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0; // Assuming constant current source
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;
float predictedVoc = 0.0;
float targetVoltage;

// Dynamic timing intervals
unsigned long mpptInterval = 100; // MPPT tracking interval (ms)
unsigned long vocInterval = 30000; // Voc measurement interval (ms)
unsigned long freqAdjustInterval = 60000; // Frequency adjustment interval (ms)
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;
unsigned long lastFreqCheck = 0;

// Function to set PWM frequency in Fast PWM mode
void setPWMFrequency(int pin, int freq) {
    int prescaler;
    if (freq >= 15000) {
        prescaler = 1;  // No prescaler
        PWM_MAX = 255;
    } else if (freq >= 4000) {
        prescaler = 8;  
        PWM_MAX = 255;
    } else if (freq >= 1000) {
        prescaler = 64; 
        PWM_MAX = 255;
    } else {
        prescaler = 256; 
        PWM_MAX = 255;
    }

    PWM_STEP = max(1, PWM_MAX / 25); // Dynamic step adjustment

    if (pin == 9 || pin == 10) {
        TCCR1B = (TCCR1B & 0b11111000) | prescaler;
    }
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
    if (pwm < 0) pwm = 0;
    return pwm;
}

// Measure open-circuit voltage
void measureOpenCircuitVoltage() {
    analogWrite(CONV1_PWM_PIN, 0);
    analogWrite(CONV2_PWM_PIN, 0);
    delay(1000);

    openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
    targetVoltage = openCircuitVoltage * 0.80;

    Serial.print("Measured Voc: ");
    Serial.println(openCircuitVoltage);

    // Adjust Voc measurement interval
    float error = fabs(openCircuitVoltage - predictedVoc);
    if (error > 1.0) {
        vocInterval = max(10000, vocInterval / 2);
    } else {
        vocInterval = min(60000, vocInterval * 2);
    }

    predictedVoc = openCircuitVoltage;
}

// Calculate estimated losses for frequency optimization
float calculateLossFunction(int freq) {
    float switchingLoss = freq * 0.00001;  // Example: proportional to frequency
    float conductionLoss = power * 0.01;  // Example: fixed conduction loss
    return switchingLoss + conductionLoss;
}

// Optimize PWM frequency to minimize losses
void optimizePWMFrequency() {
    if (fabs(panelVoltage - prevPanelVoltage) < 0.05) {  // Check stability
        int bestFreq = pwmFrequency;
        float minLoss = calculateLossFunction(pwmFrequency);

        for (int freq = PWM_FREQ_MIN; freq <= PWM_FREQ_MAX; freq += 1000) {
            float loss = calculateLossFunction(freq);
            if (loss < minLoss) {
                minLoss = loss;
                bestFreq = freq;
            }
        }

        if (bestFreq != pwmFrequency) {
            pwmFrequency = bestFreq;
            setPWMFrequency(CONV1_PWM_PIN, pwmFrequency);
            setPWMFrequency(CONV2_PWM_PIN, pwmFrequency);
            Serial.print("New optimized PWM frequency: ");
            Serial.println(pwmFrequency);
        }
    }
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
    Serial.print(" V, Power: ");
    Serial.println(power);

    if (panelVoltage < targetVoltage) {
        if (power > prevPower) {
            pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
        } else {
            pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
        }
    } else {
        if (power < prevPower) {
            pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP / 2);
        } else {
            pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
        }
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

    // Adjust MPPT interval dynamically
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

    measureOpenCircuitVoltage();
    setPWMFrequency(CONV1_PWM_PIN, pwmFrequency);
    setPWMFrequency(CONV2_PWM_PIN, pwmFrequency);
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

    if (currentMillis - lastFreqCheck >= freqAdjustInterval) {
        optimizePWMFrequency();
        lastFreqCheck = currentMillis;
    }
}
