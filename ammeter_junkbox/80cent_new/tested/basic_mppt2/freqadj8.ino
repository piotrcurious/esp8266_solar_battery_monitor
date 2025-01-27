// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_VOLTAGE_PIN A1
#define CONV2_VOLTAGE_PIN A2

#define CONV1_PWM_PIN 9   // OC1A - Timer1
#define CONV2_PWM_PIN 10  // OC1B - Timer1

// Voltage measurement scaling factors
const float VREF = 5.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 6.0; // Voltage divider ratio

// PWM control variables
const int PWM_MIN = 0;
int PWM_MAX = 255;  // Will be adjusted dynamically
int PWM_STEP = 10;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Frequency control
int pwmFrequency = 1000; // Initial frequency (Hz)

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;  // Assuming constant current source
float power, prevPower;

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;
float predictedVoc = 0.0;
float targetVoltage;

// Timing intervals
unsigned long mpptInterval = 100;
unsigned long vocInterval = 30000;
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;

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

// Function to set PWM frequency using Timer1 and update PWM_MAX
void setPWMFrequency(int frequency) {
  int prescaler;
  long topValue = (16000000 / (2 * frequency)) - 1; // Calculate TOP value

  if (topValue < 256) {
    prescaler = 1;   // No prescaler
  } else if (topValue < 512) {
    prescaler = 8;
    topValue /= 8;
  } else if (topValue < 4096) {
    prescaler = 64;
    topValue /= 64;
  } else if (topValue < 32768) {
    prescaler = 256;
    topValue /= 256;
  } else {
    prescaler = 1024;
    topValue /= 1024;
  }

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // Fast PWM, non-inverting
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (prescaler == 1 ? (1 << CS10) :
            prescaler == 8 ? (1 << CS11) :
            prescaler == 64 ? (1 << CS11) | (1 << CS10) :
            prescaler == 256 ? (1 << CS12) :
            (1 << CS12) | (1 << CS10));

  ICR1 = topValue;  // Set the new TOP value
  PWM_MAX = topValue;  // Dynamically update PWM max value based on frequency
}

// Measure open-circuit voltage by disabling loads temporarily
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
  float error = fabs(panelVoltage - targetVoltage);
  if (error > 2) {
    PWM_STEP = min(8, PWM_STEP * error);
  } else {
    PWM_STEP = max(1, PWM_STEP / 2);
  }

  if (voltageChange > 0.5) {
    mpptInterval = max(10, mpptInterval / 2);
  } else {
    mpptInterval = min(100, mpptInterval * 2);
  }

  // Adjust PWM frequency based on power fluctuations
  if (fabs(power - prevPower) > 0.5) {
    pwmFrequency = min(20000, pwmFrequency + 500);
  } else {
    pwmFrequency = max(1000, pwmFrequency - 500);
  }
  setPWMFrequency(pwmFrequency);
  Serial.print(" PWM Frequency: ");
  Serial.print(pwmFrequency);
  Serial.print(" Hz, PWM Max: ");
  Serial.println(PWM_MAX);
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
