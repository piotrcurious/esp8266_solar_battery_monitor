// Define the analog and PWM pins
#define VOLTAGE_PIN A0 // The pin to measure the voltage
#define LOAD_PIN 9 // The pin to control the load

// Define the constants
#define VREF 4.096 // The reference voltage for the ADC
#define R1 100000 // The value of the resistor R1 in the voltage divider
#define R2 10000 // The value of the resistor R2 in the voltage divider
#define VMAX 0.8 // The maximum fraction of the open-circuit voltage
#define DELTA 0.1 // The fraction to increase or decrease the load
#define T_CALIB 10000 // The interval to recalibrate the open-circuit voltage in milliseconds
#define T_MEASURE 1000 // The interval to measure the voltage drop in milliseconds

// Define the variables
float v_oc = 0; // The open-circuit voltage
float r_int = 0; // The internal resistance of the voltage source
float i_load = 0; // The current through the load
float p_load = 0; // The power dissipated by the load
float v_load = 0; // The voltage across the load
float duty = 0; // The duty cycle of the PWM signal
unsigned long t_last = 0; // The last time the open-circuit voltage was calibrated
unsigned long t_now = 0; // The current time

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);
  
  // Initialize the analog and PWM pins
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(LOAD_PIN, OUTPUT);
  
  // Calibrate the open-circuit voltage
  calibrate();
}

void loop() {
  // Get the current time
  t_now = millis();
  
  // Check if it is time to calibrate the open-circuit voltage
  if (t_now - t_last >= T_CALIB) {
    calibrate();
  }
  
  // Measure the voltage across the load
  v_load = readVoltage();
  
  // Calculate the current and power
  i_load = v_load / duty;
  p_load = v_load * i_load;
  
  // Print the results
  Serial.print("V_OC = ");
  Serial.print(v_oc, 2);
  Serial.print(" V, R_INT = ");
  Serial.print(r_int, 2);
  Serial.print(" Ohm, V_LOAD = ");
  Serial.print(v_load, 2);
  Serial.print(" V, I_LOAD = ");
  Serial.print(i_load, 3);
  Serial.print(" A, P_LOAD = ");
  Serial.print(p_load, 2);
  Serial.println(" W");
  
  // Check if the voltage is within the desired range
  if (v_load > VMAX * v_oc) {
    // Decrease the load
    duty = duty * (1 - DELTA);
    analogWrite(LOAD_PIN, duty * 255);
    // Wait for the voltage to stabilize
    delay(T_MEASURE);
  } else if (v_load < VMAX * v_oc) {
    // Increase the load
    duty = duty * (1 + DELTA);
    analogWrite(LOAD_PIN, duty * 255);
    // Wait for the voltage to stabilize
    delay(T_MEASURE);
    // Recalibrate the internal resistance
    recalibrate();
  }
}

// A function to calibrate the open-circuit voltage and the internal resistance
void calibrate() {
  // Remove the load
  duty = 0;
  analogWrite(LOAD_PIN, 0);
  // Wait for the voltage to stabilize
  delay(T_MEASURE);
  // Measure the open-circuit voltage
  v_oc = readVoltage();
  // Set an initial load
  duty = 0.5;
  analogWrite(LOAD_PIN, duty * 255);
  // Wait for the voltage to stabilize
  delay(T_MEASURE);
  // Measure the voltage drop
  v_load = readVoltage();
  // Calculate the internal resistance
  r_int = (v_oc - v_load) / (v_load / duty);
  // Update the last calibration time
  t_last = millis();
}

// A function to recalibrate the internal resistance
void recalibrate() {
  // Measure the voltage drop
  v_load = readVoltage();
  // Calculate the internal resistance
  r_int = (v_oc - v_load) / (v_load / duty);
}

// A function to read the voltage from the analog pin
float readVoltage() {
  // Read the analog value
  int value = analogRead(VOLTAGE_PIN);
  // Convert it to voltage
  float voltage = value * VREF / 1023.0;
  // Compensate for the voltage divider
  voltage = voltage * (R1 + R2) / R2;
  // Return the voltage
  return voltage;
}
