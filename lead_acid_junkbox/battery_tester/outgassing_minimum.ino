// Kalman Filter Variables
float voltage_est = 0.0;        // Estimated voltage
float slope_est = 0.0;          // Estimated voltage slope
float voltage_meas = 0.0;       // Measured voltage
float current_meas = 0.0;       // Measured current
float internalResistance = 0.05; // Estimated internal resistance of the battery (Ohms)
float deltaT = 0.1;             // Time step in seconds (adjust based on your loop timing)

// Kalman Filter Matrices
// State transition matrix A: Models how voltage and slope evolve over time
float A[2][2] = {
    {1, deltaT},   // Voltage is updated based on previous slope
    {0, 1}        // Slope remains constant over short intervals
};

// Control input matrix B: Models how the input current affects the voltage
float B[2] = {internalResistance * deltaT, 0};

// Measurement matrix H: We only measure the voltage
float H[2] = {1, 0};

// Process noise covariance Q: Represents uncertainty in the system model
float Q[2][2] = {
    {0.001, 0},   // Low uncertainty in voltage prediction
    {0, 0.001}    // Low uncertainty in slope prediction
};

// Measurement noise covariance R: Represents sensor noise in voltage measurement
float R = 0.01;   // Adjust based on the precision of your voltage sensor

// Kalman Filter covariance matrix (initialized as identity matrix)
float P[2][2] = {
    {1, 0},
    {0, 1}
};

// Function to perform Kalman Filter update
void kalmanUpdate(float current_input, float measured_voltage) {
  // Predict step: x_k+1 = A * x_k + B * u_k
  float x_pred[2];  // Predicted state
  x_pred[0] = A[0][0] * voltage_est + A[0][1] * slope_est + B[0] * current_input;
  x_pred[1] = A[1][0] * voltage_est + A[1][1] * slope_est + B[1] * current_input;

  // Predicted covariance: P_k+1 = A * P * A^T + Q
  float P_pred[2][2];
  P_pred[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
  P_pred[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];
  P_pred[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
  P_pred[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];

  // Update covariance with process noise
  P_pred[0][0] += Q[0][0];
  P_pred[0][1] += Q[0][1];
  P_pred[1][0] += Q[1][0];
  P_pred[1][1] += Q[1][1];

  // Update step: y_k = z_k - H * x_pred
  float y = measured_voltage - (H[0] * x_pred[0] + H[1] * x_pred[1]);

  // Kalman gain: K = P_pred * H^T / (H * P_pred * H^T + R)
  float S = H[0] * P_pred[0][0] + H[1] * P_pred[1][0] + R;
  float K[2];  // Kalman gain
  K[0] = (P_pred[0][0] * H[0] + P_pred[0][1] * H[1]) / S;
  K[1] = (P_pred[1][0] * H[0] + P_pred[1][1] * H[1]) / S;

  // Update state: x_k = x_pred + K * y
  voltage_est = x_pred[0] + K[0] * y;
  slope_est = x_pred[1] + K[1] * y;

  // Update covariance: P_k = (I - K * H) * P_pred
  P[0][0] = (1 - K[0] * H[0]) * P_pred[0][0];
  P[0][1] = (1 - K[0] * H[0]) * P_pred[0][1];
  P[1][0] = (1 - K[1] * H[1]) * P_pred[1][0];
  P[1][1] = (1 - K[1] * H[1]) * P_pred[1][1];
}

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);

  // Initialize the estimated state
  voltage_est = readBatteryVoltage();
  slope_est = 0.0;  // Initialize the voltage slope estimate
}

void loop() {
  unsigned long currentMillis = millis();

  // Non-blocking: Run control loop every 1 second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Check if the test has started
    if (digitalRead(buttonPin) == LOW && !testInProgress) {
      testInProgress = true;
      state = 1;  // Start the testing process
      Serial.println("Test started...");
    }

    // Handle state machine for testing
    switch (state) {
      case 0:  // Idle state
        break;

      case 1:  // Testing state: Current control and voltage monitoring
        voltage_meas = readBatteryVoltage();  // Measure battery voltage
        current_meas = readCurrent();         // Measure current

        // Update the Kalman filter with the current measurements
        kalmanUpdate(current_meas, voltage_meas);

        // Use the Kalman-filtered voltage and slope to control the current
        myPID.Compute();  // Run the PID loop for closed-loop current control
        analogWrite(pwmPin, pwmOutput);  // Adjust current using the PID output

        // Check for outgassing plateau based on Kalman-filtered slope
        if (checkOutgassing(slope_est)) {
          Serial.print("Outgassing plateau detected at voltage: ");
          Serial.println(voltage_est);

          // Transition to the waiting state (stop current)
          analogWrite(pwmPin, 0);  // Turn off current
          state = 2;  // Go to the waiting state
          previousMillis = currentMillis;  // Reset timer for 20-minute wait
        }

        // Print debug information
        Serial.print("Estimated Voltage: ");
        Serial.print(voltage_est);
        Serial.print(" V, Measured Current: ");
        Serial.print(current_meas);
        Serial.println(" mA");
        break;

      case 2:  // Waiting state after outgassing detection
        if (currentMillis - previousMillis >= 20 * 60 * 1000) {
          // After 20 minutes, reduce the current setpoint and resume testing
          setCurrent -= 50;  // Decrease current by 50mA (adjust as needed)
          if (setCurrent <= 0) setCurrent = 0;  // Ensure current doesn't go below 0
          state = 1;  // Resume testing
          Serial.println("Resuming test with lower current...");
        }
        break;
    }
  }
}

// Function to check for outgassing based on the Kalman-filtered slope
bool checkOutgassing(float voltageSlope) {
  // Detect the plateau in voltage change, indicating outgassing
  // When the slope of the voltage change is near zero for an extended period, outgassing is occurring
  return abs(voltageSlope) < slopeThreshold;  // Adjust slopeThreshold for sensitivity
}

// Placeholder functions for voltage and current readings
float readBatteryVoltage() {
  // Implement actual battery voltage reading here
  return analogRead(A0) * (5.0 / 1023.0);  // Example for Arduino Uno
}

float readCurrent() {
  // Implement actual current measurement here
  return analogRead(A1) * (500.0 / 1023.0);  // Example, assuming 500mA full-scale
}
