// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_PWM_PIN 9   // Higher priority converter
#define CONV2_PWM_PIN 10  // Lower priority converter

// Voltage measurement scaling factors
const float VREF = 5.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 10.0; // Voltage divider ratio

// PWM limits
const int PWM_MIN = 0;
const int PWM_MAX = 255;
const int PWM_STEP = 5;

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float inferredCurrent;
float power, prevPower;
float openCircuitVoltage = 0.0;
float predictedVoc = 0.0;
float targetVoltage;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Voc trend tracking (moving average)
const int VOC_HISTORY_SIZE = 5;
float vocHistory[VOC_HISTORY_SIZE] = {0};
int vocIndex = 0;
float vocTrend = 0;

// Dynamic timing intervals
unsigned long mpptInterval = 5000;
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

// Function to infer panel current based on voltage drop and Voc trend
float inferCurrent(float measuredVoltage, float voc) {
  float voltageDrop = voc - measuredVoltage;  
  if (voltageDrop <= 0) return 0;  // No load condition

  // Use trend factor to improve accuracy
  float trendFactor = 1.0 + (vocTrend * 0.1);
  float inferredCurrent = (voltageDrop / voc) * trendFactor;
  return inferredCurrent;
}

// Update Voc trend tracking with a moving average
void updateVocTrend(float newVoc) {
  vocHistory[vocIndex] = newVoc;
  vocIndex = (vocIndex + 1) % VOC_HISTORY_SIZE;

  float sum = 0;
  for (int i = 0; i < VOC_HISTORY_SIZE; i++) {
    sum += vocHistory[i];
  }
  float averageVoc = sum / VOC_HISTORY_SIZE;

  // Trend calculation: difference between last and current average
  vocTrend = newVoc - averageVoc;
}

// Measure open-circuit voltage by disabling loads temporarily
void measureOpenCircuitVoltage() {
  analogWrite(CONV1_PWM_PIN, PWM_MIN);
  analogWrite(CONV2_PWM_PIN, PWM_MIN);
  delay(1000);  // Allow stabilization

  openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  targetVoltage = openCircuitVoltage * 0.80;

  updateVocTrend(openCircuitVoltage);

  Serial.print("Measured Voc: "); Serial.println(openCircuitVoltage);

  // Adjust interval based on prediction error
  float error = fabs(openCircuitVoltage - predictedVoc);
  vocInterval = error > 1.0 ? max(10000, vocInterval / 2) : min(60000, vocInterval * 2);
  
  predictedVoc = openCircuitVoltage;
}

// Perform Perturb & Observe MPPT tracking using inferred current
void performMpptAdjustment() {
  prevPower = power;
  prevPanelVoltage = panelVoltage;

  panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  inferredCurrent = inferCurrent(panelVoltage, openCircuitVoltage);
  power = panelVoltage * inferredCurrent;

  Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
  Serial.print(" V, Target: "); Serial.print(targetVoltage);
  Serial.print(" V, Inferred Current: "); Serial.print(inferredCurrent);
  Serial.print(" A, Power: "); Serial.println(power);

  if (panelVoltage < targetVoltage) {
    // Increase PWM if below target voltage and power is improving
    if (power > prevPower) {
      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
    } else {
      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    }
  } else {
    // Decrease PWM if voltage exceeds target or power drops
    if (power < prevPower) {
      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    } else {
      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
    }
  }

  // If Conv1 is maxed out, adjust Conv2
  if (pwmConv1 >= PWM_MAX) {
    pwmConv2 = adjustPWM(pwmConv2, PWM_STEP);
  }

  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);

  Serial.print(" PWM Conv1: "); Serial.print(pwmConv1);
  Serial.print(" PWM Conv2: "); Serial.println(pwmConv2);

  // Adjust MPPT interval dynamically based on voltage change
  float voltageChange = fabs(panelVoltage - prevPanelVoltage);
  mpptInterval = voltageChange > 0.5 ? max(2000, mpptInterval / 2) : min(10000, mpptInterval * 2);
}

void setup() {
  pinMode(CONV1_PWM_PIN, OUTPUT);
  pinMode(CONV2_PWM_PIN, OUTPUT);
  Serial.begin(9600);

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
}
