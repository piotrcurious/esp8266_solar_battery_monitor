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

// PWM and frequency control
const unsigned long FREQ_MIN = 1000;  // 1 kHz
const unsigned long FREQ_MAX = 20000; // 20 kHz
unsigned long currentFreq = 10000;    // Start at 10 kHz
const unsigned long FREQ_STEP = 1000; // 1 kHz step
unsigned long bestFreq = 10000;       // Frequency with highest power
float bestPower = 0;                  // Highest power achieved

// Timer configuration
unsigned int timerTopValue = 0;     // ICR1 value
const int PWM_MIN = 0;
int PWM_MAX = 255;                  // Will be updated based on frequency
int PWM_STEP = 10;                  // Will be scaled based on PWM_MAX

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;  // Assuming constant current source
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;  
float predictedVoc = 0.0;  
float targetVoltage;

// Dynamic timing intervals
unsigned long mpptInterval = 100;     // MPPT tracking interval (ms)
unsigned long vocInterval = 30000;    // Voc measurement interval (ms)
unsigned long freqInterval = 5000;    // Frequency adjustment interval (ms)
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;
unsigned long lastFreqCheck = 0;

// Calculate and set PWM frequency, returns actual frequency achieved
unsigned long setPwmFrequency(unsigned long targetFreq) {
    unsigned long prescaler = 1;
    unsigned long ocr = F_CPU / (2UL * targetFreq) - 1;
    
    // Find suitable prescaler
    while (ocr > 65535 && prescaler < 1024) {
        if (prescaler == 1) prescaler = 8;
        else if (prescaler == 8) prescaler = 64;
        else if (prescaler == 64) prescaler = 256;
        else if (prescaler == 256) prescaler = 1024;
        ocr = F_CPU / (2UL * targetFreq * prescaler) - 1;
    }
    
    // Configure Timer1
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // Phase correct PWM mode
    
    // Set prescaler
    if (prescaler == 1) TCCR1B = _BV(WGM13) | _BV(CS10);
    else if (prescaler == 8) TCCR1B = _BV(WGM13) | _BV(CS11);
    else if (prescaler == 64) TCCR1B = _BV(WGM13) | _BV(CS11) | _BV(CS10);
    else if (prescaler == 256) TCCR1B = _BV(WGM13) | _BV(CS12);
    else TCCR1B = _BV(WGM13) | _BV(CS12) | _BV(CS10);
    
    timerTopValue = ocr;
    ICR1 = timerTopValue;
    
    // Calculate actual frequency achieved
    unsigned long actualFreq = F_CPU / (2UL * prescaler * (timerTopValue + 1));
    
    // Update PWM_MAX based on timer top value
    PWM_MAX = timerTopValue;
    
    // Scale existing PWM values to new range
    float scale = (float)timerTopValue / 255.0;
    pwmConv1 = (int)(pwmConv1 * scale);
    pwmConv2 = (int)(pwmConv2 * scale);
    
    // Scale PWM_STEP based on new PWM_MAX
    PWM_STEP = max(1, (int)(PWM_MAX * 0.05)); // 5% of max value
    
    // Update PWM outputs with scaled values
    OCR1A = pwmConv1;
    OCR1B = pwmConv2;
    
    Serial.print("Set PWM frequency to: ");
    Serial.print(actualFreq);
    Serial.print(" Hz, Top Value: ");
    Serial.print(timerTopValue);
    Serial.print(", PWM_MAX: ");
    Serial.println(PWM_MAX);
    
    return actualFreq;
}

// Function to read and scale voltage
float readVoltage(int pin, float ratio) {
    int raw = analogRead(pin);
    return (raw * VREF / ADC_RES) * ratio;
}

// Function to adjust PWM with limits and direct timer register setting
int adjustPWM(int pwm, int adjustment) {
    pwm += adjustment;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    return pwm;
}

// Measure open-circuit voltage
void measureOpenCircuitVoltage() {
    OCR1A = 0;  // Direct timer register access
    OCR1B = 0;
    delay(1000);  // Allow stabilization

    openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
    targetVoltage = openCircuitVoltage * 0.80;
    
    Serial.print("Measured Voc: "); 
    Serial.println(openCircuitVoltage);
}

// Adjust frequency for maximum power
void adjustFrequency() {
    static bool increasing = true;
    static float prevPowerAtFreq = 0;
    
    // Store current state
    float currentPower = power;
    unsigned long prevFreq = currentFreq;
    
    // Update best frequency if current power is higher
    if (currentPower > bestPower) {
        bestPower = currentPower;
        bestFreq = currentFreq;
    }
    
    // Determine direction based on power change
    if (currentPower > prevPowerAtFreq) {
        // Keep going in same direction
        if (increasing) {
            currentFreq += FREQ_STEP;
            if (currentFreq > FREQ_MAX) {
                increasing = false;
                currentFreq = bestFreq;
            }
        } else {
            currentFreq -= FREQ_STEP;
            if (currentFreq < FREQ_MIN) {
                increasing = true;
                currentFreq = bestFreq;
            }
        }
    } else {
        // Change direction
        increasing = !increasing;
        currentFreq = bestFreq; // Return to best known frequency
    }
    
    // Set new frequency and store power for next comparison
    currentFreq = setPwmFrequency(currentFreq);
    prevPowerAtFreq = currentPower;
    
    Serial.print("Best frequency: ");
    Serial.print(bestFreq);
    Serial.print(" Hz, Best power: ");
    Serial.println(bestPower);
}

// Perform MPPT tracking with direct timer register access
void performMpptAdjustment() {
    prevPower = power;
    prevPanelVoltage = panelVoltage;

    panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
    power = panelVoltage * panelCurrent * ((float)pwmConv1/PWM_MAX + (float)pwmConv2/PWM_MAX);

    if (panelVoltage < targetVoltage) {
        if (power > prevPower) {
            pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
        } else {
            pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
        }
        pwmConv2 = adjustPWM(pwmConv2, -PWM_STEP);
    } else {
        if (power < prevPower) {
            pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP/2);
        } else {
            pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
        }
        if (pwmConv1 >= PWM_MAX) {
            pwmConv2 = adjustPWM(pwmConv2, PWM_STEP);
        }
    }

    // Direct timer register access
    OCR1A = pwmConv1;
    OCR1B = pwmConv2;

    Serial.print("Panel V: "); 
    Serial.print(panelVoltage);
    Serial.print("V, Power: "); 
    Serial.print(power);
    Serial.print("W, PWM1: ");
    Serial.print((float)pwmConv1/PWM_MAX * 100);
    Serial.print("%, PWM2: ");
    Serial.print((float)pwmConv2/PWM_MAX * 100);
    Serial.println("%");
}

void setup() {
    pinMode(CONV1_PWM_PIN, OUTPUT);
    pinMode(CONV2_PWM_PIN, OUTPUT);
    Serial.begin(115200);
    
    // Initialize PWM frequency and measure initial Voc
    currentFreq = setPwmFrequency(currentFreq);
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
    
    if (currentMillis - lastFreqCheck >= freqInterval) {
        adjustFrequency();
        lastFreqCheck = currentMillis;
    }
}
