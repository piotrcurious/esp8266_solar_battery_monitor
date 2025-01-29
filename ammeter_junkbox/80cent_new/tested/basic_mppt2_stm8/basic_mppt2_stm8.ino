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

// MPPT control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;  // Assuming constant current source (optional)
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;

// Open-circuit voltage tracking
float openCircuitVoltage = 0.0;  
float predictedVoc = 0.0;  // For error estimation
float targetVoltage;

// Dynamic timing intervals
unsigned long mpptInterval = 100;  // MPPT tracking interval (ms)
unsigned long vocInterval = 30000;  // Voc measurement interval (ms)
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

double fabs(double x)
{
  union {double f; uint64_t i;} u = {x};
  u.i &= -1ULL/2;
  return u.f;
}
// Measure open-circuit voltage by disabling loads temporarily
void measureOpenCircuitVoltage() {
  analogWrite(CONV1_PWM_PIN, PWM_MIN);
  analogWrite(CONV2_PWM_PIN, PWM_MIN);
  delay(1000);  // Allow stabilization

  openCircuitVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  targetVoltage = openCircuitVoltage * 0.80;

  Serial_print_s("Measured Voc: ");
  Serial_print_f(openCircuitVoltage);
  
  // Adjust interval based on prediction error
  float error = fabs(openCircuitVoltage - predictedVoc);
  if (error > 1.0) { 
    vocInterval = max(10000, vocInterval / 2);  // Increase frequency
  } else {
    vocInterval = min(60000, vocInterval * 2);  // Reduce frequency
  }
  
  predictedVoc = openCircuitVoltage;
}

// Perform Perturb & Observe MPPT tracking
void performMpptAdjustment() {
  prevPower = power;
  prevPanelVoltage = panelVoltage;

  panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  power = panelVoltage * panelCurrent * ((float)pwmConv1/PWM_MAX) +panelVoltage*panelCurrent*((float)pwmConv2/PWM_MAX);

  Serial_print_s("Vin:"); Serial_print_f(panelVoltage);
  Serial_print_s(",Vt:"); Serial_print_f(targetVoltage);
  Serial_print_s(",Power:"); Serial_print_f(power);

  if (panelVoltage < targetVoltage) {
    // Increase PWM if below target voltage
    if (power > prevPower) {
      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
//      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    } else {
      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP);
    }
      // If Conv1 is maxed out, start adjusting Conv2
   //   if (pwmConv1 >= PWM_MAX) {
      pwmConv2 = adjustPWM(pwmConv2, -PWM_STEP);
   //   }
  } else {
    // Decrease PWM if voltage exceeds target
    if (power < prevPower) {
      pwmConv1 = adjustPWM(pwmConv1, -PWM_STEP/2);
//      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);

    } else {
      pwmConv1 = adjustPWM(pwmConv1, PWM_STEP);
    }
  }
    // If Conv1 is maxed out, start adjusting Conv2
    if (pwmConv1 >= PWM_MAX) {
    pwmConv2 = adjustPWM(pwmConv2, PWM_STEP);
    }
  

  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);

  Serial_print_s(",PWM1:"); Serial_print_f(pwmConv1);
  Serial_print_s(",PWM2:"); Serial_println_f(pwmConv2);

  // Adjust MPPT interval dynamically based on voltage fluctuations
  float voltageChange = fabs(panelVoltage - prevPanelVoltage);
  float error = fabs(panelVoltage - targetVoltage);
 if (error > 2) {
    PWM_STEP = min(8,PWM_STEP*error);  
 }  else {
    PWM_STEP = max(1,PWM_STEP/2);  
 }
  
  if (voltageChange > 0.5) {
    mpptInterval = max(10, mpptInterval / 2);  // Faster adjustment
//    PWM_STEP = max(1,PWM_STEP/2);

  } else {
    mpptInterval = min(100, mpptInterval * 2);  // Slower adjustment
//    PWM_STEP = min(64,PWM_STEP*2);
    PWM_STEP = max(1,PWM_STEP/2);  

  }
}

void setup() {
  pinMode(CONV1_PWM_PIN, OUTPUT);
  pinMode(CONV2_PWM_PIN, OUTPUT);
  Serial_begin(115200);

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
