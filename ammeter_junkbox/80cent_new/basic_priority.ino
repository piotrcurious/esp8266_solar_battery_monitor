// Pin definitions
#define PANEL_VOLTAGE_PIN A0
#define CONV1_VOLTAGE_PIN A1
#define CONV2_VOLTAGE_PIN A2

#define CONV1_PWM_PIN 9   // Higher priority converter
#define CONV2_PWM_PIN 10  // Lower priority converter

// Voltage reference and scaling factors (adjust based on resistor divider)
const float VREF = 5.0;   // Arduino ADC reference voltage
const int ADC_RES = 1023; // 10-bit ADC resolution
const float PANEL_VOLTAGE_RATIO = 10.0; // Divider scaling factor (example: 100k/10k)

// Desired voltage target as a percentage of panel voltage
const float TARGET_RATIO = 0.80; 

// PWM limits
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Control variables
int pwmConv1 = 0;
int pwmConv2 = 0;

// Read analog voltage with scaling
float readVoltage(int pin, float ratio) {
  int raw = analogRead(pin);
  return (raw * VREF / ADC_RES) * ratio;
}

// Adjust PWM duty cycle with limits
int adjustPWM(int pwm, int adjustment) {
  pwm += adjustment;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  return pwm;
}

void setup() {
  pinMode(CONV1_PWM_PIN, OUTPUT);
  pinMode(CONV2_PWM_PIN, OUTPUT);
  
  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);
  
  Serial.begin(9600);
}

void loop() {
  // Read panel input voltage
  float panelVoltage = readVoltage(PANEL_VOLTAGE_PIN, PANEL_VOLTAGE_RATIO);
  float targetVoltage = panelVoltage * TARGET_RATIO;

  // Read converter outputs
  float conv1Voltage = readVoltage(CONV1_VOLTAGE_PIN, 1.0);
  float conv2Voltage = readVoltage(CONV2_VOLTAGE_PIN, 1.0);

  // Control logic for priority-based optimization
  if (panelVoltage > targetVoltage) {
    // Increase power output in priority order
    if (pwmConv1 < PWM_MAX) {
      pwmConv1 = adjustPWM(pwmConv1, 5);
    } else if (pwmConv2 < PWM_MAX) {
      pwmConv2 = adjustPWM(pwmConv2, 5);
    }
  } else {
    // Reduce power output if voltage drops
    if (pwmConv2 > PWM_MIN) {
      pwmConv2 = adjustPWM(pwmConv2, -5);
    } else if (pwmConv1 > PWM_MIN) {
      pwmConv1 = adjustPWM(pwmConv1, -5);
    }
  }

  // Apply PWM updates
  analogWrite(CONV1_PWM_PIN, pwmConv1);
  analogWrite(CONV2_PWM_PIN, pwmConv2);

  // Print debug information
  Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
  Serial.print(" V, Target: "); Serial.print(targetVoltage);
  Serial.print(" V, Conv1: "); Serial.print(conv1Voltage);
  Serial.print(" V, Conv2: "); Serial.println(conv2Voltage);

  delay(500); // Control loop delay
}
