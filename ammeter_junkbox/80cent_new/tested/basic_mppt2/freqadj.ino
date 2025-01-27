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
const int PWM_MAX = 255;
int PWM_STEP = 10;

// Frequency control
const unsigned long FREQ_MIN = 1000;  // 1 kHz
const unsigned long FREQ_MAX = 20000; // 20 kHz
unsigned long currentFreq = 10000;    // Start at 10 kHz
const unsigned long FREQ_STEP = 1000; // 1 kHz step for frequency adjustment
unsigned long bestFreq = 10000;       // Frequency with highest power
float bestPower = 0;                  // Highest power achieved

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

// Timer configuration settings for different frequencies
void setPwmFrequency(unsigned long freq) {
  // Calculate prescaler and OCR values for Timer1 (pins 9 and 10)
  unsigned long prescaler = 1;
  unsigned long ocr = F_CPU / (2 * freq * prescaler) - 1;
  
  // Adjust prescaler if OCR value is too large
  while (ocr > 65535 && prescaler < 1024) {
    if (prescaler == 1) prescaler = 8;
    else if (prescaler == 8) prescaler = 64;
    else if (prescaler == 64) prescaler = 256;
    else if (prescaler == 256) prescaler = 1024;
    ocr = F_CPU / (2 * freq * prescaler) - 1;
  }
  
  // Configure Timer1
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // Phase correct PWM mode
  
  // Set prescaler
  if (prescaler == 1) TCCR1B = _BV(WGM13) | _BV(CS10);
  else if (prescaler == 8) TCCR1B = _BV(WGM13) | _BV(CS11);
  else if (prescaler == 64) TCCR1B = _BV(WGM13) | _BV(CS11) | _BV(CS10);
  else if (prescaler == 256) TCCR1B = _BV(WGM13) | _BV(CS12);
  else TCCR1B = _BV(WGM13) | _BV(CS12) | _BV(CS10);
  
  ICR1 = ocr; // Set top value for PWM frequency
  currentFreq = freq;
  
  Serial.print("Set PWM frequency to: ");
  Serial.print(freq);
  Serial.println(" Hz");
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

// Measure open-circuit voltage
void measureOpenCircuitVoltage() {
  analogWrite(CONV1_PWM_PIN, PWM_MIN);
  analogWrite(CONV2_PWM_PIN, PWM_MIN);
  delay(1000);  // Allow stabilization

  openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
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

// Adjust PWM frequency for maximum power
void adjustFrequency() {
  static bool increasing = true;
  static unsigned long freqSearchStart = 0;
  static float powerAtFreqStart = 0;
  
  // Store power at current frequency
  float currentPower = power;
  
  // Update best frequency if current power is higher
  if (currentPower > bestPower) {
    bestPower = currentPower;
    bestFreq = currentFreq;
  }
  
  // Adjust frequency
  if (increasing) {
    currentFreq += FREQ_STEP;
    if (currentFreq >= FREQ_MAX) {
      increasing = false;
      currentFreq = bestFreq; // Return to best frequency
    }
  } else {
    currentFreq -= FREQ_STEP;
    if (currentFreq <= FREQ_MIN) {
      increasing = true;
      currentFreq = bestFreq; // Return to best frequency
    }
  }
  
  setPwmFrequency(currentFreq);
  
  Serial.print("Best frequency: ");
  Serial.print(bestFreq);
  Serial.print(" Hz, Best power: ");
  Serial.println(bestPower);
}

// Perform Perturb & Observe MPPT tracking
void performMpptAdjustment() {
  prevPower = power;
  prevPanelVoltage = panelVoltage;

  panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  power = panelVoltage * panelCurrent * ((float)pwmConv1/PWM_MAX + (float)pwmConv2/PWM_MAX);

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

  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);

  // Dynamic PWM step adjustment
  float voltageChange = fabs(panelVoltage - prevPanelVoltage);
  float error = fabs(panelVoltage - targetVoltage);
  
  if (error > 2) {
    PWM_STEP = min(8, PWM_STEP * error);  
  } else {
    PWM_STEP = max(1, PWM_STEP/2);  
  }
  
  if (voltageChange > 0.5) {
    mpptInterval = max(10, mpptInterval / 2);
  } else {
    mpptInterval = min(100, mpptInterval * 2);
    PWM_STEP = max(1, PWM_STEP/2);
  }
}

void setup() {
  pinMode(CONV1_PWM_PIN, OUTPUT);
  pinMode(CONV2_PWM_PIN, OUTPUT);
  Serial.begin(115200);
  
  // Initialize PWM frequency
  setPwmFrequency(currentFreq);
  measureOpenCircuitVoltage();
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodically measure open-circuit voltage
  if (currentMillis - lastVocCheck >= vocInterval) {
    measureOpenCircuitVoltage();
    lastVocCheck = currentMillis;
  }

  // Periodically adjust MPPT tracking
  if (currentMillis - lastMpptCheck >= mpptInterval) {
    performMpptAdjustment();
    lastMpptCheck = currentMillis;
  }
  
  // Periodically adjust frequency
  if (currentMillis - lastFreqCheck >= freqInterval) {
    adjustFrequency();
    lastFreqCheck = currentMillis;
  }
}
