// Define the pins and constants
#define PWM_PIN 9 // The pin for the variable load
#define LOAD_PIN A0 // The pin for measuring the load voltage
#define SOURCE_PIN A1 // The pin for measuring the source voltage
#define MAX_VOLTAGE 5.0 // The maximum voltage of the analog pins
#define RESOLUTION 1023.0 // The resolution of the analog pins
#define DELAY_TIME 1000 // The delay time for measuring the voltage drop
#define INTERVAL_TIME 10000 // The interval time for recalibrating the open-circuit voltage
#define VOLTAGE_FACTOR 0.8 // The factor for keeping the output voltage within 80% of the open-circuit voltage
#define LOAD_FACTOR 0.1 // The factor for increasing or decreasing the load
#define LEARNING_RATE 0.01 // The learning rate for the gradient descent algorithm
#define ARRAY_SIZE 10 // The size of the array for storing the rint values

// Declare the global variables
float rint[ARRAY_SIZE]; // The array for storing the rint values at different open-circuit voltages
float v_oc; // The open-circuit voltage
float v_load; // The load voltage
float i_load; // The load current
float p_load; // The load power
float r_load; // The load resistance
float error; // The error between the desired and actual output voltage
float gradient; // The gradient of the error function
int pwm_value; // The PWM value for the variable load
unsigned long previous_time; // The previous time for recalibrating the open-circuit voltage

// Initialize the pins and variables
void setup() {
  // Set the PWM pin as output
  pinMode(PWM_PIN, OUTPUT);
  // Set the analog pins as input
  pinMode(LOAD_PIN, INPUT);
  pinMode(SOURCE_PIN, INPUT);
  // Initialize the PWM value to zero
  pwm_value = 0;
  // Initialize the previous time to zero
  previous_time = 0;
  // Initialize the rint array with some random values
  for (int i = 0; i < ARRAY_SIZE; i++) {
    rint[i] = random(10, 100) / 100.0;
  }
}

// Main loop
void loop() {
  // Get the current time
  unsigned long current_time = millis();
  // Check if the interval time has elapsed
  if (current_time - previous_time >= INTERVAL_TIME) {
    // Recalibrate the open-circuit voltage
    recalibrate();
    // Update the previous time
    previous_time = current_time;
  } else {
    // Use the open-circuit voltage corrected by the rint and load
    v_oc = v_oc - i_load * rint[int(v_oc * RESOLUTION / MAX_VOLTAGE)];
  }
  // Measure the load voltage
  v_load = analogRead(LOAD_PIN) * MAX_VOLTAGE / RESOLUTION;
  // Calculate the load current
  i_load = pwm_value / 255.0;
  // Calculate the load power
  p_load = v_load * i_load;
  // Calculate the load resistance
  r_load = v_load / i_load;
  // Calculate the error between the desired and actual output voltage
  error = v_oc * VOLTAGE_FACTOR - v_load;
  // Calculate the gradient of the error function
  gradient = -2 * error / r_load;
  // Update the PWM value using the gradient descent algorithm
  pwm_value = pwm_value - LEARNING_RATE * gradient;
  // Constrain the PWM value between 0 and 255
  pwm_value = constrain(pwm_value, 0, 255);
  // Write the PWM value to the variable load
  analogWrite(PWM_PIN, pwm_value);
  // Check if the input voltage increases
  if (analogRead(SOURCE_PIN) * MAX_VOLTAGE / RESOLUTION > v_oc) {
    // Re-calibrate the rint by increasing the load briefly by 10%
    pwm_value = pwm_value * (1 + LOAD_FACTOR);
    // Constrain the PWM value between 0 and 255
    pwm_value = constrain(pwm_value, 0, 255);
    // Write the PWM value to the variable load
    analogWrite(PWM_PIN, pwm_value);
    // Delay for measuring the voltage drop
    delay(DELAY_TIME);
    // Measure the new load voltage
    float v_load_new = analogRead(LOAD_PIN) * MAX_VOLTAGE / RESOLUTION;
    // Calculate the new load current
    float i_load_new = pwm_value / 255.0;
    // Calculate the new rint
    float rint_new = (v_oc - v_load_new) / i_load_new;
    // Store the new rint in the array
    rint[int(v_oc * RESOLUTION / MAX_VOLTAGE)] = rint_new;
  }
  // Check if the input voltage decreases
  if (analogRead(SOURCE_PIN) * MAX_VOLTAGE / RESOLUTION < v_oc) {
    // Decrease the load to reach the 80% value back basing on previous calculations
    pwm_value = v_oc * VOLTAGE_FACTOR * r_load * 255.0 / MAX_VOLTAGE;
    // Constrain the PWM value between 0 and 255
    pwm_value = constrain(pwm_value, 0, 255);
    // Write the PWM value to the variable load
    analogWrite(PWM_PIN, pwm_value);
  }
}

// Function for recalibrating the open-circuit voltage
void recalibrate() {
  // Remove the load
  pwm_value = 0;
  // Write the PWM value to the variable load
  analogWrite(PWM_PIN, pwm_value);
  // Delay for measuring the open-circuit voltage
  delay(DELAY_TIME);
  // Measure the open-circuit voltage
  v_oc = analogRead(SOURCE_PIN) * MAX_VOLTAGE / RESOLUTION;
}
