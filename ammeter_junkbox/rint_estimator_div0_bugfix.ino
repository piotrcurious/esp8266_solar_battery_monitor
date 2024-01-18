// Define the extra function to estimate r_in
float estimate_r_in(float v_in, float v_out) {
  // Use some formula or algorithm to estimate r_in
  // For example, you can use Ohm's law: r_in = (v_in - v_out) / i_out
  // But you will need to measure or calculate i_out somehow
  // You can also use some other method, such as curve fitting or machine learning
  // This is up to you to decide and implement
  float r_in = 0; // Placeholder value, change this
  // Return the estimated r_in
  return r_in;
}

// Modify the loop function to include the check and the call
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
    // Check if the internal resistance is zero
    if (r_in == 0) {
      // Call the extra function to estimate r_in
      r_in = estimate_r_in(v_in, v_out);
      // Print the estimation result
      Serial.print("Estimated: r_in = ");
      Serial.println(r_in);
    }
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
