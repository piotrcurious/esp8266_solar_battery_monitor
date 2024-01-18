// Define the pins
#define PWM_PIN 9 // The pin for the PWM output to the load
#define LOAD_PIN A0 // The pin for measuring the load voltage
#define SOURCE_PIN A1 // The pin for measuring the source voltage

// Define the constants
#define MAX_PWM 255 // The maximum PWM value
#define MIN_PWM 0 // The minimum PWM value
#define LOAD_FACTOR 0.1 // The factor for increasing or decreasing the load
#define VOLTAGE_FACTOR 0.8 // The factor for keeping the output voltage within 80% of the open-circuit voltage
#define CALIBRATION_INTERVAL 10000 // The interval for calibrating the open-circuit voltage in milliseconds
#define CALIBRATION_DURATION 1000 // The duration for calibrating the internal resistance in milliseconds
#define LEARNING_RATE 0.01 // The learning rate for the gradient descent algorithm

// Define the variables
float open_circuit_voltage = 0; // The open-circuit voltage of the source
float internal_resistance = 0; // The internal resistance of the source
float load_resistance = 0; // The load resistance
float output_voltage = 0; // The output voltage
float output_current = 0; // The output current
float output_power = 0; // The output power
int pwm_value = 0; // The PWM value
unsigned long last_calibration_time = 0; // The last time the open-circuit voltage was calibrated
unsigned long calibration_start_time = 0; // The start time of the internal resistance calibration
bool is_calibrating = false; // The flag for indicating if the internal resistance is being calibrated

// Define the array for the rint values at various open-circuit voltages
// This is a look up table that can be used to estimate the internal resistance
// The values are in ohms and are based on some hypothetical data
// You can modify this array according to your own data
float rint[11] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);
  // Set the PWM pin as output
  pinMode(PWM_PIN, OUTPUT);
  // Set the initial PWM value to zero
  analogWrite(PWM_PIN, MIN_PWM);
}

void loop() {
  // Read the source voltage
  float source_voltage = analogRead(SOURCE_PIN) * (5.0 / 1023.0);
  // Check if it is time to calibrate the open-circuit voltage
  if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
    // Set the PWM value to zero
    analogWrite(PWM_PIN, MIN_PWM);
    // Update the open-circuit voltage
    open_circuit_voltage = source_voltage;
    // Update the last calibration time
    last_calibration_time = millis();
    // Set the calibration flag to true
    is_calibrating = true;
    // Set the calibration start time
    calibration_start_time = millis();
    // Print the open-circuit voltage
    Serial.print("Open-circuit voltage: ");
    Serial.println(open_circuit_voltage);
  }
  // Check if the internal resistance is being calibrated
  if (is_calibrating) {
    // Check if the calibration duration is over
    if (millis() - calibration_start_time > CALIBRATION_DURATION) {
      // Set the calibration flag to false
      is_calibrating = false;
      // Reset the PWM value
      pwm_value = 0;
    } else {
      // Increase the PWM value by 10%
      pwm_value = pwm_value + MAX_PWM * LOAD_FACTOR;
      // Constrain the PWM value within the limits
      pwm_value = constrain(pwm_value, MIN_PWM, MAX_PWM);
      // Write the PWM value to the pin
      analogWrite(PWM_PIN, pwm_value);
      // Read the load voltage
      float load_voltage = analogRead(LOAD_PIN) * (5.0 / 1023.0);
      // Calculate the load resistance
      load_resistance = load_voltage / (pwm_value / 255.0);
      // Calculate the output current
      output_current = load_voltage / load_resistance;
      // Calculate the output power
      output_power = output_current * output_current * load_resistance;
      // Calculate the internal resistance using the gradient descent algorithm
      // The cost function is the difference between the output power and the maximum power
      // The gradient is the derivative of the cost function with respect to the internal resistance
      // The update rule is to subtract the gradient times the learning rate from the current value
      float cost = output_power - (open_circuit_voltage * open_circuit_voltage) / (4 * internal_resistance);
      float gradient = - (open_circuit_voltage * open_circuit_voltage) / (4 * internal_resistance * internal_resistance) + output_current * output_current;
      internal_resistance = internal_resistance - gradient * LEARNING_RATE;
      // Constrain the internal resistance to be positive
      internal_resistance = max(internal_resistance, 0);
      // Print the internal resistance
      Serial.print("Internal resistance: ");
      Serial.println(internal_resistance);
    }
  } else {
    // Use the open-circuit voltage corrected by the internal resistance and the load
    float corrected_voltage = open_circuit_voltage - output_current * internal_resistance;
    // Check if the source voltage has increased
    if (source_voltage > corrected_voltage) {
      // Increase the load by 10%
      pwm_value = pwm_value + MAX_PWM * LOAD_FACTOR;
      // Constrain the PWM value within the limits
      pwm_value = constrain(pwm_value, MIN_PWM, MAX_PWM);
      // Write the PWM value to the pin
      analogWrite(PWM_PIN, pwm_value);
    }
    // Check if the source voltage has decreased
    if (source_voltage < corrected_voltage) {
      // Decrease the load to reach the 80% value back based on previous calculations
      // The target voltage is the open-circuit voltage times the voltage factor
      float target_voltage = open_circuit_voltage * VOLTAGE_FACTOR;
      // The target current is the target voltage divided by the load resistance
      float target_current = target_voltage / load_resistance;
      // The target PWM value is the target current times the load resistance times 255
      int target_pwm = target_current * load_resistance * 255;
      // Constrain the target PWM value within the limits
      target_pwm = constrain(target_pwm, MIN_PWM, MAX_PWM);
      // Write the target PWM value to the pin
      analogWrite(PWM_PIN, target_pwm);
      // Update the PWM value
      pwm_value = target_pwm;
    }
    // Read the load voltage
    float load_voltage = analogRead(LOAD_PIN) * (5.0 / 1023.0);
    // Calculate the load resistance
    load_resistance = load_voltage / (pwm_value / 255.0);
    // Calculate the output current
    output_current = load_voltage / load_resistance;
    // Calculate the output voltage
    output_voltage = source_voltage - output_current * internal_resistance;
    // Calculate the output power
    output_power = output_current * output_voltage;
    // Print the output voltage, current and power
    Serial.print("Output voltage: ");
    Serial.println(output_voltage);
    Serial.print("Output current: ");
    Serial.println(output_current);
    Serial.print("Output power: ");
    Serial.println(output_power);
  }
}
