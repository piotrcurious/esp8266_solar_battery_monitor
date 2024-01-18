// Define the pins and constants
#define PWM_PIN 9 // PWM output pin for load control
#define VOLTAGE_PIN A0 // Analog input pin for voltage measurement
#define LOAD_PIN A1 // Analog input pin for load voltage measurement
#define VREF 4.096 // Voltage reference for ADC
#define R1 100000 // Resistor value for voltage divider (ohms)
#define R2 10000 // Resistor value for voltage divider (ohms)
#define VMAX 0.8 // Maximum voltage ratio to keep
#define DELTA 0.1 // Load change factor for calibration
#define T_CAL 10000 // Calibration interval (ms)
#define T_MEAS 1000 // Measurement interval (ms)

// Declare global variables
float v_in; // Input voltage
float v_out; // Output voltage
float r_in; // Internal resistance
float i_out; // Output current
float p_out; // Output power
float duty; // Duty cycle for PWM
float alpha; // Learning rate for gradient descent
unsigned long t_last; // Last calibration time
unsigned long t_now; // Current time

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Initialize PWM output
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  // Initialize ADC reference
  analogReference(EXTERNAL);
  // Initialize variables
  v_in = 0;
  v_out = 0;
  r_in = 0;
  i_out = 0;
  p_out = 0;
  duty = 0;
  alpha = 0.01;
  t_last = 0;
  t_now = 0;
}

void loop() {
  // Get the current time
  t_now = millis();
  // Check if it is time to calibrate
  if (t_now - t_last >= T_CAL) {
    // Remove the load
    analogWrite(PWM_PIN, 0);
    // Measure the open-circuit voltage
    v_in = readVoltage(VOLTAGE_PIN);
    // Update the last calibration time
    t_last = t_now;
    // Print the calibration result
    Serial.print("Calibrated: v_in = ");
    Serial.println(v_in);
  }
  else {
    // Measure the output voltage
    v_out = readVoltage(LOAD_PIN);
    // Calculate the output current
    i_out = (v_in - v_out) / r_in;
    // Calculate the output power
    p_out = v_out * i_out;
    // Check if the output voltage is within the limit
    if (v_out / v_in > VMAX) {
      // Decrease the duty cycle
      duty -= alpha * p_out;
      // Constrain the duty cycle between 0 and 1
      duty = constrain(duty, 0, 1);
      // Update the PWM output
      analogWrite(PWM_PIN, duty * 255);
      // Print the adjustment result
      Serial.print("Decreased: duty = ");
      Serial.println(duty);
    }
    else {
      // Increase the duty cycle
      duty += alpha * p_out;
      // Constrain the duty cycle between 0 and 1
      duty = constrain(duty, 0, 1);
      // Update the PWM output
      analogWrite(PWM_PIN, duty * 255);
      // Print the adjustment result
      Serial.print("Increased: duty = ");
      Serial.println(duty);
    }
    // Check if the input voltage has changed significantly
    if (abs(v_in - readVoltage(VOLTAGE_PIN)) > 0.1) {
      // Increase the load briefly
      analogWrite(PWM_PIN, (duty + DELTA) * 255);
      // Wait for 1 second
      delay(T_MEAS);
      // Measure the new output voltage
      float v_new = readVoltage(LOAD_PIN);
      // Calculate the new internal resistance
      r_in = (v_in - v_new) / ((v_new / R2) / (R1 + R2));
      // Print the calibration result
      Serial.print("Recalibrated: r_in = ");
      Serial.println(r_in);
    }
  }
}

// Function to read the voltage from an analog pin
float readVoltage(int pin) {
  // Read the raw value from the pin
  int value = analogRead(pin);
  // Convert the value to voltage
  float voltage = value * VREF / 1023.0;
  // Return the voltage
  return voltage;
}
