// Pin definitions
const int pwmPin1 = 9;  // PWM for DC-DC converter 1
const int pwmPin2 = 10; // PWM for DC-DC converter 2
const int panelVoltagePin = A0; // Solar panel voltage
const int outputVoltagePin1 = A1; // DC-DC converter 1 output
const int outputVoltagePin2 = A2; // DC-DC converter 2 output

// System parameters
float panelVoc = 0.0;  // Estimated open-circuit voltage
float targetPanelVoltage = 0.0; 
const float panelVoltageTargetRatio = 0.80;  // 80% of Voc

// PWM duty cycle control
int dutyCycle1 = 0;
int dutyCycle2 = 0;
const int dutyStep = 5;  // Step change for PWM adjustment
const int maxDuty = 255; // Max PWM value (100% duty cycle)

// Converter priority (lower value = higher priority)
const int priority1 = 1; 
const int priority2 = 2; 

// Timing
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000;  // 1 second

// Function to read voltage (scaling assumed for 0-5V input)
float readVoltage(int pin) {
  return analogRead(pin) * (5.0 / 1023.0);
}

// Function to estimate panel current
float estimateCurrent(float prevVoltage, float currentVoltage, int dutyCycle) {
  float deltaV = prevVoltage - currentVoltage;
  float deltaD = dutyCycle / 255.0;
  if (deltaD == 0) return 0;  // Avoid division by zero
  return deltaV / deltaD;  // Simplified approximation of I = dV/dD
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
    static float previousVoltage = panelVoltage;

    // Infer panel current
    float estimatedCurrent = estimateCurrent(previousVoltage, panelVoltage, dutyCycle1 + dutyCycle2);

    // Update Voc estimation using a simple tracking approach
    if (panelVoltage > panelVoc) {
      panelVoc = panelVoltage;  // Track the highest voltage as Voc
      targetPanelVoltage = panelVoc * panelVoltageTargetRatio;
    }

    adjustPWM();

    // Debug output
    Serial.print("Panel Voltage: "); Serial.print(panelVoltage);
    Serial.print("V, Estimated Current: "); Serial.print(estimatedCurrent);
    Serial.print("A, Duty 1: "); Serial.print(dutyCycle1);
    Serial.print(", Duty 2: "); Serial.println(dutyCycle2);

    previousVoltage = panelVoltage;
  }
}
