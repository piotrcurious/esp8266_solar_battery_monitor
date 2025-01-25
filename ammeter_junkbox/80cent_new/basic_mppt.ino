// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_VOLTAGE_PIN A1
#define CONV2_VOLTAGE_PIN A2

#define CONV1_PWM_PIN 9   // Higher priority converter
#define CONV2_PWM_PIN 10  // Lower priority converter

// Constants for voltage measurement and control
const float VREF = 5.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 10.0; // Divider scaling factor

// Target voltage as a percentage of open-circuit voltage
const float TARGET_RATIO = 0.80; 
float openCircuitVoltage = 0.0;  // Estimated Voc

// PWM limits
const int PWM_MIN = 0;
const int PWM_MAX = 255;
const int PWM_STEP = 5;

// MPPT tracking intervals
const unsigned long MPPT_CHECK_INTERVAL = 5000;  // Time between MPPT adjustments (ms)
const unsigned long VOC_CHECK_INTERVAL = 30000;  // Time between Voc measurements (ms)
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;

// PWM control variables
int pwmConv1 = 0;
int pwmConv2 = 0;

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

// Function to measure open-circuit voltage
void measureOpenCircuitVoltage() {
  // Disable both converters to get true Voc
  analogWrite(CONV1_PWM_PIN, PWM_MIN);
  analogWrite(CONV2_PWM_PIN, PWM_MIN);
  delay(1000);  // Allow voltage to stabilize

  openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  Serial.print("Measured Voc: "); Serial.println(openCircuitVoltage);
}

// MPPT tracking logic by load reduction test
void performMpptAdjustment() {
  float panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  float targetVoltage = openCircuitVoltage * TARGET_RATIO;

  Serial.print("Panel: "); Serial.print(panelVoltage);
  Serial.print(" V, Target: "); Serial.print(targetVoltage);
  
  // MPPT load reduction test
  if (panelVoltage < targetVoltage) {
    // Panel voltage too low, reduce load starting with low priority
    if (pwmConv2 > PWM_MIN) {
      pwmConv2 = adjustPWM(pwmConv2, -PWM_STEP);
    } else if (pwmConv1 > PWM_MIN) {
      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    }
  } else {
    // Panel voltage OK, gradually increase load in priority order
    if (pwmConv1 < PWM_MAX) {
      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
    } else if (pwmConv2 < PWM_MAX) {
      pwmConv2 = adjustPWM(pwmConv2, PWM_STEP);
    }
  }

  // Apply PWM changes
  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);

  Serial.print(" PWM Conv1: "); Serial.print(pwmConv1);
  Serial.print(" PWM Conv2: "); Serial.println(pwmConv2);
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
  if (currentMillis - lastVocCheck >= VOC_CHECK_INTERVAL) {
    measureOpenCircuitVoltage();
    lastVocCheck = currentMillis;
  }

  // Periodically adjust MPPT tracking
  if (currentMillis - lastMpptCheck >= MPPT_CHECK_INTERVAL) {
    performMpptAdjustment();
    lastMpptCheck = currentMillis;
  }
}
