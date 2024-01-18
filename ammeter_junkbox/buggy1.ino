// Define pins and constants
#define PWM_PIN 9 // Pin for variable load
#define DUMMY_PIN 10 // Pin for dummy load
#define LOAD_PIN A0 // Pin for load voltage measurement
#define SOURCE_PIN A1 // Pin for source voltage measurement
#define R_DUMMY 100 // Resistance of dummy load in ohms
#define R_LOAD_MIN 10 // Minimum resistance of variable load in ohms
#define R_LOAD_MAX 1000 // Maximum resistance of variable load in ohms
#define V_REF 5 // Reference voltage in volts
#define ALPHA 0.01 // Learning rate for gradient descent
#define TOL 0.01 // Tolerance for convergence
#define N 10 // Number of open circuit voltages to store
#define T 10000 // Time interval for recalibration in milliseconds

// Declare global variables
float v_oc[N]; // Array of open circuit voltages
float r_int[N]; // Array of internal resistances
float v_load; // Load voltage
float v_source; // Source voltage
float i_load; // Load current
float p_load; // Load power
float r_load; // Load resistance
float r_int_est; // Estimated internal resistance
float v_oc_est; // Estimated open circuit voltage
int pwm_load; // PWM value for variable load
int pwm_dummy; // PWM value for dummy load
int index; // Index for the array of voltages and resistances
unsigned long t_prev; // Previous time for recalibration

// Initialize the pins and variables
void setup() {
  // Set the PWM pins as outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DUMMY_PIN, OUTPUT);

  // Set the analog pins as inputs
  pinMode(LOAD_PIN, INPUT);
  pinMode(SOURCE_PIN, INPUT);

  // Initialize the PWM values to zero
  pwm_load = 0;
  pwm_dummy = 0;

  // Initialize the index to zero
  index = 0;

  // Initialize the previous time to zero
  t_prev = 0;

  // Initialize the array of voltages and resistances
  for (int i = 0; i < N; i++) {
    v_oc[i] = 0;
    r_int[i] = 0;
  }
}

// Main loop
void loop() {
  // Get the current time
  unsigned long t_now = millis();

  // Check if it is time to recalibrate
  if (t_now - t_prev >= T) {
    // Remove the load and dummy load
    pwm_load = 0;
    pwm_dummy = 0;
    analogWrite(PWM_PIN, pwm_load);
    analogWrite(DUMMY_PIN, pwm_dummy);

    // Measure the open circuit voltage
    v_oc[index] = analogRead(SOURCE_PIN) * V_REF / 1023;

    // Update the index
    index = (index + 1) % N;

    // Reset the previous time
    t_prev = t_now;
  }
  else {
    // Measure the load and source voltages
    v_load = analogRead(LOAD_PIN) * V_REF / 1023;
    v_source = analogRead(SOURCE_PIN) * V_REF / 1023;

    // Estimate the open circuit voltage and internal resistance
    v_oc_est = estimate_v_oc();
    r_int_est = estimate_r_int();

    // Calculate the load current and power
    i_load = v_load / r_load;
    p_load = v_load * i_load;

    // Check if the source voltage has increased
    if (v_source > v_oc_est) {
      // Increase the dummy load until the source voltage returns to the previous value
      while (v_source > v_oc_est) {
        pwm_dummy++;
        analogWrite(DUMMY_PIN, pwm_dummy);
        v_source = analogRead(SOURCE_PIN) * V_REF / 1023;
      }

      // Correct the internal resistance using Kirchoff's law
      r_int_est = (v_oc_est - v_source) / (v_source / R_DUMMY);

      // Remove the dummy load
      pwm_dummy = 0;
      analogWrite(DUMMY_PIN, pwm_dummy);
    }

    // Check if the source voltage has decreased
    if (v_source < v_oc_est) {
      // Decrease the load until the source voltage reaches 80% of the open circuit voltage
      while (v_source < 0.8 * v_oc_est) {
        pwm_load--;
        analogWrite(PWM_PIN, pwm_load);
        v_source = analogRead(SOURCE_PIN) * V_REF / 1023;
      }
    }

    // Adjust the load to maximize the output power using gradient descent
    float grad = gradient();
    while (abs(grad) > TOL) {
      pwm_load = pwm_load - ALPHA * grad;
      pwm_load = constrain(pwm_load, 0, 255);
      analogWrite(PWM_PIN, pwm_load);
      v_load = analogRead(LOAD_PIN) * V_REF / 1023;
      r_load = v_load / (v_source - v_load) * r_int_est;
      i_load = v_load / r_load;
      p_load = v_load * i_load;
      grad = gradient();
    }
  }
}

// Function to estimate the open circuit voltage using the array of voltages and resistances
float estimate_v_oc() {
  // Calculate the average of the voltages
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += v_oc[i];
  }
  float avg = sum / N;

  // Return the average
  return avg;
}

// Function to estimate the internal resistance using the array of voltages and resistances
float estimate_r_int() {
  // Calculate the average of the resistances
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += r_int[i];
  }
  float avg = sum / N;

  // Return the average
  return avg;
}

// Function to calculate the gradient of the output power with respect to the PWM value
float gradient() {
  // Calculate the derivative of the load voltage with respect to the PWM value
  float dv_du = V_REF / 1023 * R_LOAD_MIN / (R_LOAD_MIN + r_int_est) / 255;

  // Calculate the derivative of the load resistance with respect to the load voltage
  float dr_dv = r_int_est / (v_source - v_load) - v_load / (v_source - v_load) / (v_source - v_load) * r_int_est;

  // Calculate the derivative of the load current with respect to the load voltage
  float di_dv = 1 / r_load - v_load / r_load / r_load * dr_dv;

  // Calculate the derivative of the output power with respect to the load voltage
  float dp_dv = i_load + v_load * di_dv;

  // Calculate the derivative of the output power with respect to the PWM value
  float dp_du = dp_dv * dv_du;

  // Return the derivative
  return dp_du;
}
