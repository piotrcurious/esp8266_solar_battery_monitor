// Define the analog input pin for voltage measurement
#define VOLTAGE_PIN A0

// Define the PWM output pin for load control
#define LOAD_PIN 9

// Define the sampling interval in milliseconds
#define INTERVAL 100

// Define the calibration interval in milliseconds
#define CALIBRATION 10000

// Define the target voltage ratio
#define TARGET_RATIO 0.8

// Define the learning rate for gradient descent
#define LEARNING_RATE 0.01

// Define the initial load value
#define INITIAL_LOAD 128

// Define the load increment for calibration
#define LOAD_INCREMENT 25

// Define the voltage reference in volts
#define VREF 5.0

// Define the conversion factor from ADC value to voltage
#define ADC_TO_VOLT (VREF / 1024.0)

// Declare global variables
float open_circuit_voltage; // The open-circuit voltage of the source
float internal_resistance; // The internal resistance of the source
float load_resistance; // The load resistance
float load_current; // The load current
float load_power; // The load power
float error; // The error between the actual and target voltage ratios
float gradient; // The gradient of the error with respect to the load resistance
unsigned long previous_time; // The previous time of sampling
unsigned long calibration_time; // The calibration time
bool calibrating; // The flag for calibration mode
int load_value; // The PWM value for the load

// Initialize the variables and pins
void setup() {
  // Set the analog input pin as input
  pinMode(VOLTAGE_PIN, INPUT);

  // Set the PWM output pin as output
  pinMode(LOAD_PIN, OUTPUT);

  // Set the initial load value
  load_value = INITIAL_LOAD;

  // Set the load resistance as the nominal resistance of the PWM pin
  load_resistance = 100.0;

  // Set the calibration flag to false
  calibrating = false;

  // Set the previous time and calibration time to the current time
  previous_time = millis();
  calibration_time = millis();

  // Set the open-circuit voltage and internal resistance to zero
  open_circuit_voltage = 0.0;
  internal_resistance = 0.0;
}

// Main loop
void loop() {
  // Get the current time
  unsigned long current_time = millis();

  // Check if the calibration interval has elapsed
  if (current_time - calibration_time >= CALIBRATION) {
    // Set the calibration flag to true
    calibrating = true;

    // Remove the load
    load_value = 0;

    // Reset the calibration time
    calibration_time = current_time;
  }

  // Check if the sampling interval has elapsed
  if (current_time - previous_time >= INTERVAL) {
    // Read the voltage from the analog input pin
    int voltage_reading = analogRead(VOLTAGE_PIN);

    // Convert the ADC value to voltage
    float voltage = voltage_reading * ADC_TO_VOLT;

    // Check if the calibration mode is on
    if (calibrating) {
      // Update the open-circuit voltage
      open_circuit_voltage = voltage;

      // Check if the load value is zero
      if (load_value == 0) {
        // Increase the load by a small amount
        load_value += LOAD_INCREMENT;
      }
      else {
        // Calculate the internal resistance
        internal_resistance = (open_circuit_voltage - voltage) / (load_value / 255.0 * VREF / load_resistance);

        // Turn off the calibration mode
        calibrating = false;

        // Reset the load value
        load_value = INITIAL_LOAD;
      }
    }
    else {
      // Calculate the load current
      load_current = (open_circuit_voltage - voltage) / (internal_resistance + load_resistance);

      // Calculate the load power
      load_power = load_current * load_current * load_resistance;

      // Calculate the error between the actual and target voltage ratios
      error = voltage / open_circuit_voltage - TARGET_RATIO;

      // Calculate the gradient of the error with respect to the load resistance
      gradient = -2.0 * load_current * load_current * load_resistance * error;

      // Update the load resistance using gradient descent
      load_resistance -= LEARNING_RATE * gradient;

      // Constrain the load resistance to a positive value
      load_resistance = max(load_resistance, 0.0);

      // Calculate the load value from the load resistance
      load_value = (int) (load_resistance / (load_resistance + internal_resistance) * 255.0);
    }

    // Write the load value to the PWM output pin
    analogWrite(LOAD_PIN, load_value);

    // Reset the previous time
    previous_time = current_time;
  }
}
