// Define the analog input pin for measuring the output voltage
const int analogPin = A0;

// Define the PWM output pin for controlling the load
const int pwmPin = 9;

// Define the learning rate
const float alpha = 0.01;

// Define the stopping criterion
const float epsilon = 0.01;

// Define the desired voltage ratio
const float ratio = 0.8;

// Define the open-circuit voltage
float V_oc = 0;

// Define the output voltage
float V_out = 0;

// Define the load
float load = 0;

// Define the objective function
float J = 0;

// Define the gradient
float dJ = 0;

// Define a flag for calibration
bool calibrate = true;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize the PWM output pin
  pinMode(pwmPin, OUTPUT);
}

void loop() {
  // Read the output voltage from the analog input pin
  V_out = analogRead(analogPin) * (5.0 / 1023.0);

  // If calibration is needed, remove the load and measure the open-circuit voltage
  if (calibrate) {
    analogWrite(pwmPin, 0); // Set the load to zero
    delay(1000); // Wait for one second
    V_oc = analogRead(analogPin) * (5.0 / 1023.0); // Read the open-circuit voltage
    calibrate = false; // Set the calibration flag to false
  }

  // Calculate the objective function
  J = pow(V_out - ratio * V_oc, 2);

  // Calculate the gradient
  dJ = 2 * (V_out - ratio * V_oc) * (V_out / V_oc);

  // Update the load using gradient descent
  load = load - alpha * dJ;

  // Constrain the load to be between 0 and 255
  load = constrain(load, 0, 255);

  // Write the load to the PWM output pin
  analogWrite(pwmPin, load);

  // Print the output voltage, the open-circuit voltage, the load, and the objective function
  Serial.print("V_out = ");
  Serial.print(V_out);
  Serial.print(" V, V_oc = ");
  Serial.print(V_oc);
  Serial.print(" V, load = ");
  Serial.print(load);
  Serial.print(" , J = ");
  Serial.println(J);

  // Check the stopping criterion
  if (J < epsilon) {
    // Stop the gradient descent algorithm
    Serial.println("Gradient descent converged.");
    calibrate = true; // Set the calibration flag to true
    delay(1000); // Wait for one second
  }
}
