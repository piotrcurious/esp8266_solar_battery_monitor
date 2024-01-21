// Define constants and variables
#define VOLTAGE_PIN A0 // Analog pin for voltage measurement
#define PWM_PIN 9 // PWM pin for load control
#define V_REF 28.0 // Reference voltage for ADC
#define ADC_MAX 1023 // Maximum value for ADC
#define V_OC_MAX 28.0 // Maximum open-circuit voltage
#define V_OC_MIN 5.0 // Minimum open-circuit voltage
#define V_OC_TARGET 0.8 // Target ratio of output voltage to open-circuit voltage
#define V_OC_TOLERANCE 0.05 // Tolerance for output voltage deviation from target
#define V_OC_CALIBRATION_INTERVAL 10000 // Interval for open-circuit voltage calibration in milliseconds
#define LEARNING_RATE 0.01 // Learning rate for gradient descent algorithm

float v_oc = 0.0; // Estimated open-circuit voltage
float v_out = 0.0; // Measured output voltage
float i_out = 0.0; // Estimated output current
float p_out = 0.0; // Estimated output power
float load = 0.0; // Load value for PWM pin
float error = 0.0; // Error value for gradient descent algorithm
unsigned long last_calibration = 0; // Last time of open-circuit voltage calibration

// Initialize the Arduino
void setup() {
  // Set the PWM pin as output
  pinMode(PWM_PIN, OUTPUT);
  // Set the initial load value
  load = 0.5;
  // Write the load value to the PWM pin
  analogWrite(PWM_PIN, load * 255);
  // Initialize the serial monitor
  Serial.begin(115200);
}

// Main loop
void loop() {
  // Read the voltage from the analog pin
  v_out = analogRead(VOLTAGE_PIN) * V_REF / ADC_MAX;
  // Estimate the output current using Ohm's law
  i_out = v_out / (load * V_REF);
  // Estimate the output power using P = VI
  p_out = v_out * i_out;
  // Print the output voltage, current and power
  Serial.print("V_out = ");
  Serial.print(v_out);
  Serial.print(" V, I_out = ");
  Serial.print(i_out);
  Serial.print(" A, P_out = ");
  Serial.print(p_out);
  Serial.println(" W");
  // Check if it is time to calibrate the open-circuit voltage
  if (millis() - last_calibration > V_OC_CALIBRATION_INTERVAL) {
    // Remove the load
    analogWrite(PWM_PIN, 0);
    // Read the open-circuit voltage
    delay(100); //wait a little for the voltage to stabilize
    v_oc = analogRead(VOLTAGE_PIN) * V_REF / ADC_MAX;
    // Print the open-circuit voltage
    Serial.print("V_oc = ");
    Serial.print(v_oc);
    Serial.println(" V");
    //restore the load
    analogWrite(PWM_PIN, load * 255);
    // Update the last calibration time
    last_calibration = millis();
    
  } else {
    // Estimate the open-circuit voltage using gradient descent algorithm
//    v_oc = v_oc - LEARNING_RATE * (v_out - V_OC_TARGET * v_oc);
    v_oc = v_oc - LEARNING_RATE * ((V_OC_TARGET * v_oc) - v_out);

    // Print the estimated open-circuit voltage
    Serial.print("V_oc (estimated) = ");
    Serial.print(v_oc);
    Serial.println(" V");
  }
  // Calculate the error between the output voltage and the target voltage
//  error = v_out - V_OC_TARGET * v_oc;
  error = (V_OC_TARGET * v_oc) - v_out;

  // Adjust the load value using gradient descent algorithm
  load = load - LEARNING_RATE * error;
  // Constrain the load value between 0 and 1
  load = constrain(load, 0, 1);
  // Write the load value to the PWM pin
  analogWrite(PWM_PIN, load * 255);
  // Print the load value
  Serial.print("Load = ");
  Serial.println(load);
  // Add a delay
  delay(100);
}
