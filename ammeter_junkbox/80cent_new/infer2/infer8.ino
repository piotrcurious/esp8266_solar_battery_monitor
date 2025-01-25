// Pin definitions
const int pwmPin1 = 9;  // PWM for DC-DC converter 1
const int pwmPin2 = 10; // PWM for DC-DC converter 2
const int panelVoltagePin = A0;  // Solar panel voltage
const int outputVoltagePin1 = A1; // DC-DC converter 1 output
const int outputVoltagePin2 = A2; // DC-DC converter 2 output
const int temperaturePin = A3;  // Temperature sensor

// System parameters
float panelVoc = 0.0;  // Estimated open-circuit voltage
float targetPanelVoltage = 0.0; 
const float panelVoltageTargetRatio = 0.80;  // 80% of Voc
const float tempCoefficient = -0.003;  // Panel voltage temp coefficient (V/°C)

// PWM control parameters
int dutyCycle1 = 0;
int dutyCycle2 = 0;
const int dutyStep = 5;  // Step change for PWM adjustment
const int maxDuty = 255; // Max PWM value (100% duty cycle)

// Converter priority (lower value = higher priority)
const int priority1 = 1; 
const int priority2 = 2; 

// Historical tracking for inference
float previousVoltage = 0.0;
float previousDutyCycle = 0.0;
float estimatedCurrent = 0.0;

// Weighted coefficients for multiple inference methods
const float weight_dVdD = 0.5;
const float weight_vocTrend = 0.3;
const float weight_tempComp = 0.2;

// Timing
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000;  // 1 second

// Function to read voltage
float readVoltage(int pin) {
  return analogRead(pin) * (5.0 / 1023.0);
}

// Function to estimate temperature in Celsius (assumed 10mV/°C sensor)
float readTemperature() {
  return (readVoltage(temperaturePin) - 0.5) * 100.0;
}

// Estimate current based on voltage change vs duty cycle change
float estimateCurrent_dVdD(float prevVoltage, float currentVoltage, float prevDuty, float currentDuty) {
  float deltaV = prevVoltage - currentVoltage;
  float deltaD = (currentDuty - prevDuty) / 255.0;
  if (deltaD == 0) return estimatedCurrent;  
  return deltaV / deltaD;  // Current approximation based on voltage change
}

// Estimate current based on Voc trend (linear approximation)
float estimateCurrent_VocTrend(float currentVoltage) {
  float deltaVoc = panelVoc - currentVoltage;
  return deltaVoc / (panelVoc * (1 - panelVoltageTargetRatio));
}

// Estimate temperature impact on current estimation
float estimateCurrent_Temperature(float currentVoltage, float temperature) {
  float tempEffect = (temperature - 25.0) * tempCoefficient * panelVoc;
  return (currentVoltage + tempEffect) / panelVoc;
}

// Compute weighted current inference
float computeCurrentInference(float currentVoltage, float dutyCycle) {
  float i1 = estimateCurrent_dVdD(previousVoltage, currentVoltage, previousDutyCycle, dutyCycle);
  float i2 = estimateCurrent_VocTrend(currentVoltage);
  float i3 = estimateCurrent_Temperature(currentVoltage, readTemperature());

  return (i1 * weight_dVdD) + (i2 * weight_vocTrend) + (i3 * weight_tempComp);
}

// Adjust PWM based on priority and target voltage
void adjustPWM() {
  float panelVoltage = readVoltage(panelVoltagePin);

  // Adjust duty cycles based on priority
  if (panelVoltage < targetPanelVoltage) {
    if (priority1 < priority2 && dutyCycle1 < maxDuty) {
      dutyCycle1 += dutyStep;
    } else if (dutyCycle2 < maxDuty) {
      dutyCycle2 += dutyStep;
    }
  } else {
    if (priority2 < priority1 && dutyCycle2 > 0) {
      dutyCycle2 -= dutyStep;
    } else if (dutyCycle1 > 0) {
      dutyCycle1 -= dutyStep;
    }
  }

  analogWrite(pwmPin1, dutyCycle1);
  analogWrite(pwmPin2, dutyCycle2);
}

void setup() {
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;

    float panelVoltage = readVoltage(panelVoltagePin);

    // Track Voc by observing peak voltage
    if (panelVoltage > panelVoc) {
      panelVoc = panelVoltage;
      targetPanelVoltage = panelVoc * panelVoltageTargetRatio;
    }

    // Estimate current using weighted methods
    float inferredCurrent = computeCurrentInference(panelVoltage, dutyCycle1 + dutyCycle2);

    adjustPWM();

    // Debug output
    Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
    Serial.print("V, Estimated Current: "); Serial.print(inferredCurrent);
    Serial.print("A, Duty 1: "); Serial.print(dutyCycle1);
    Serial.print(", Duty 2: "); Serial.println(dutyCycle2);

    // Store previous values for next iteration
    previousVoltage = panelVoltage;
    previousDutyCycle = dutyCycle1 + dutyCycle2;
  }
}
