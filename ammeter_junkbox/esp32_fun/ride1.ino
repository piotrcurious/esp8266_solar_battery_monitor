#include <ArduinoEigen.h>
#include <cmath>

// --- Global Variables ---
float V_OC = 20.0;          // Open Circuit Voltage
float I_SC = 5.0;          // Short Circuit Current
float capacitance = 1000e-6; // Capacitance (Farads)
float panel_R = 1.0;         // Panel Series Resistance (Ohms)
float panel_R_SH = 100.0;      // Panel Shunt Resistance (Ohms)
float diode_n = 1.5;         // Diode ideality factor
float diode_Is = 1e-6;       // Diode saturation current
float thermal_voltage_Vt = 0.026; // Thermal voltage at room temp
float panel_C = 1.0;         // Panel Capacitance (Farads)
float targetVoltageRatio = 0.8;
float targetVoltage;
float load_R_base = 10.0;    // Base Load Resistance
float pwm_output_resistance = 0.1; // Resistance of PWM output MOSFET and wiring (example) - TUNING PARAMETER


const int voltagePin = A0;    // Analog pin for voltage sensing
const int pwmPin = 2;       // PWM output pin
const int pwmFrequency = 1000; // PWM frequency (Hz)
const int pwmResolution = 8;  // PWM resolution (bits)

float currentVoltageRaw = 0.0;
float currentVoltageFiltered = 0.0;
float previousVoltageFiltered = 0.0;
float pwmDutyCycle = 0.0;
float previousPWM = 0.0;
float integralError = 0.0;
float previousTime = 0.0;
float timeStep = 0.0;
float accumulatedOvershoot = 0.0;
float accumulatedUndershoot = 0.0;

float filterCoefficient = 0.05;
float filterTimeConstant;
float samplingInterval = 0.01;
float systemTimeConstant;
float gradientDelayFactor = 1.0;
unsigned long gradientDelayMs;

float w_error = 1.0;
float w_deriv = 0.1;
float w_integral = 0.01;
float w_pwm_change = 0.001;
float w_overshoot = 0.005;
float w_undershoot = 0.005;
float learningRate = 0.01;
float delta_PWM = 0.01;
float momentumFactor = 0.9;
float previousMomentum = 0.0;

// --- Define Matrix and Vector Types using Eigen ---
typedef Eigen::Matrix<float, 1, 1> Matrix1f;
typedef Eigen::Matrix<float, 1, 1> Vector1f;
typedef Eigen::Matrix<float, 2, 1> Vector2f; // For parameter derivatives (example)

// --- Weighting Matrices for Loss Function (Eigen Matrices) ---
Matrix1f Q_matrix;
Matrix1f Ru_matrix;
Matrix1f Qo_matrix;
Matrix1f Qu_matrix;

// --- State and Control Vectors (Eigen Vectors) ---
Vector1f x_state;     // State vector (voltage)
Vector1f u_control;   // Control input vector (PWM)
Vector1f y_output;    // Output vector (measured voltage)
Vector1f y_target_vec; // Target output vector

// --- Parameter Derivatives Accumulators (Example for panel_R and capacitance) ---
Vector2f accumulatedParameterDerivatives;
float parameterDerivativeLearningRate = 0.0001; // Small learning rate for parameter adjustment


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(pwmPin, OUTPUT);
  ledcSetup(0, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, 0);
  targetVoltage = V_OC * targetVoltageRatio;
  previousTime = millis();
  currentVoltageFiltered = readSolarVoltageRaw();
  previousVoltageFiltered = currentVoltageFiltered;

  systemTimeConstant = panel_R * capacitance;
  filterTimeConstant = samplingInterval / filterCoefficient - samplingInterval;
  filterCoefficient = samplingInterval / (samplingInterval + filterTimeConstant);
  gradientDelayMs = static_cast<unsigned long>(gradientDelayFactor * systemTimeConstant * 1000);

  Serial.print("System Time Constant: "); Serial.print(systemTimeConstant); Serial.println(" s");
  Serial.print("Filter Time Constant: "); Serial.print(filterTimeConstant); Serial.println(" s");
  Serial.print("Gradient Delay (Complex + Param Derivatives): "); Serial.print(gradientDelayMs); Serial.println(" ms");

  // --- Initialize Weighting Matrices ---
  Q_matrix(0, 0) = w_error;
  Ru_matrix(0, 0) = w_pwm_change;
  Qo_matrix(0, 0) = w_overshoot;
  Qu_matrix(0, 0) = w_undershoot;

  // --- Initialize State and Control Vectors ---
  y_target_vec(0, 0) = targetVoltage;
  u_control(0, 0) = pwmDutyCycle;
  x_state(0, 0) = currentVoltageFiltered;

  // --- Initialize Parameter Derivative Accumulators ---
  accumulatedParameterDerivatives.setZero(); // Initialize to zero
}

void loop() {
  timeStep = (millis() - previousTime) / 1000.0;
  previousTime = millis();

  currentVoltageRaw = readSolarVoltageRaw();
  currentVoltageFiltered = applyFilter(currentVoltageRaw);

  u_control(0, 0) = pwmDutyCycle;
  y_output(0, 0) = currentVoltageFiltered;
  x_state(0, 0) = currentVoltageFiltered;

  float loss = calculateLoss(currentVoltageFiltered, targetVoltage);
  Vector2f parameterGradients = calculateParameterDerivatives(loss, currentVoltageFiltered, pwmDutyCycle); // Calculate derivatives wrt parameters
  updateParameters(parameterGradients); // Update panel_R and capacitance based on derivatives

  float gradient = calculateGradient(loss, pwmDutyCycle);
  updatePWM(gradient);


  previousVoltageFiltered = currentVoltageFiltered;
  previousPWM = pwmDutyCycle;

  Serial.print("Voltage (Raw): "); Serial.print(currentVoltageRaw);
  Serial.print(", Voltage (Filtered): "); Serial.print(currentVoltageFiltered);
  Serial.print(", PWM: "); Serial.println(pwmDutyCycle);
  Serial.print(", panel_R: "); Serial.print(panel_R);
  Serial.print(", capacitance: "); Serial.println(capacitance);


  delay(static_cast<int>(samplingInterval * 1000));
}

float readSolarVoltageRaw() {
  int rawValue = analogRead(voltagePin);
  float voltage = rawValue * (3.3 / 4095.0) * voltageScaleFactor;
  return voltage;
}

float applyFilter(float rawVoltage) {
  float filteredVoltage = filterCoefficient * rawVoltage + (1 - filterCoefficient) * previousVoltageFiltered;
  return filteredVoltage;
}

void setPWMDutyCycle(float dutyCycle) {
  pwmDutyCycle = constrain(dutyCycle, 0.0, 1.0);
  int dutyCycleValue = dutyCycle * ((1 << pwmResolution) - 1);
  ledcWrite(0, dutyCycleValue);
}

float calculateLoss(float currentVoltage, float targetVoltage) {
  Vector1f error_vec;
  error_vec(0, 0) = currentVoltage - targetVoltage;

  integralError += error_vec(0,0) * timeStep; // Accumulate integral error

  // Overshoot and Undershoot accumulation - for penalty terms
  if (currentVoltage > targetVoltage) {
    accumulatedOvershoot += (currentVoltage - targetVoltage) * timeStep;
    accumulatedUndershoot = 0;
  } else {
    accumulatedUndershoot += (targetVoltage - currentVoltage) * timeStep;
    accumulatedUndershoot += (targetVoltage - currentVoltage) * timeStep;
  }
  Vector1f overshoot_vec;
  overshoot_vec(0, 0) = accumulatedOvershoot;

  Vector1f undershoot_vec;
  undershoot_vec(0, 0) = accumulatedUndershoot;


  Vector1f pwm_change_vec;
  pwm_change_vec(0, 0) = pwmDutyCycle - previousPWM;

  Matrix1f error_term = error_vec.transpose() * Q_matrix * error_vec;
  Matrix1f pwm_change_term = pwm_change_vec.transpose() * Ru_matrix * pwm_change_vec;
  Matrix1f overshoot_term = overshoot_vec.transpose() * Qo_matrix * overshoot_vec;
  Matrix1f undershoot_term = undershoot_vec.transpose() * Qu_matrix * undershoot_vec;


  float loss = error_term(0, 0) + pwm_change_term(0, 0) + overshoot_term(0,0) + undershoot_term(0,0);
  return loss;
}

// --- More Complex Model-Based Gradient Calculation ---
float calculateGradient(float currentLoss, float currentPWM) {
  float gradient;
  float deltaPWM_for_model = delta_PWM;

  // --- 1. Simulate Forward Perturbation (u + delta_u) ---
  float predictedVoltageUp = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM + deltaPWM_for_model, panel_R, capacitance);
  float lossUp = calculateLoss(predictedVoltageUp, targetVoltage);

  // --- 2. Simulate Backward Perturbation (u - delta_u) ---
  float predictedVoltageDown = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM - deltaPWM_for_model, panel_R, capacitance);
  float lossDown = calculateLoss(predictedVoltageDown, targetVoltage);

  // --- 3. Approximate Gradient ---
  gradient = (lossUp - lossDown) / (2 * deltaPWM_for_model);
  return gradient;
}

// --- Function to predict voltage one time step ahead using Enhanced Solar Cell Model ---
float predictVoltageStep(float currentVoltage, float currentPWM, float nextPWM, float current_panel_R, float current_capacitance) {
  float nextVoltage;
  float R_load_current = load_R_base / (currentPWM + 0.0001);
  float R_load_next = load_R_base / (nextPWM + 0.0001);

  // --- Enhanced Solar Cell Model ---
  float I_diode_current = diode_Is * (exp((currentVoltage + I_SC * current_panel_R) / (diode_n * thermal_voltage_Vt)) - 1.0);
  float I_pv_current = I_SC - I_diode_current - (currentVoltage + I_SC * current_panel_R) / panel_R_SH;

  // Discrete-time approximation
  nextVoltage = currentVoltage + (timeStep / current_capacitance) * (I_pv_current - currentVoltage / (current_panel_R + R_load_next) );

  return nextVoltage;
}

// --- Calculate Parameter Derivatives (wrt panel_R and capacitance) ---
Vector2f calculateParameterDerivatives(float currentLoss, float currentVoltage, float currentPWM) {
  Vector2f parameterGradients;
  float delta_parameter = 0.1 * panel_R; // Example delta for panel_R - ADJUST BASED ON PARAMETER SCALE!
  float delta_capacitance = 0.1 * capacitance; // Example delta for capacitance - ADJUST BASED ON PARAMETER SCALE!

  // --- 1. Perturb panel_R upwards ---
  float perturbed_panel_R_up = panel_R + delta_parameter;
  float predictedVoltage_R_up = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, perturbed_panel_R_up, capacitance);
  float loss_R_up = calculateLoss(predictedVoltage_R_up, targetVoltage);

  // --- 2. Perturb panel_R downwards ---
  float perturbed_panel_R_down = panel_R - delta_parameter;
  float predictedVoltage_R_down = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, perturbed_panel_R_down, capacitance);
  float loss_R_down = calculateLoss(predictedVoltage_R_down, targetVoltage);

  // --- 3. Perturb capacitance upwards ---
  float perturbed_capacitance_up = capacitance + delta_capacitance;
  float predictedVoltage_C_up = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, panel_R, perturbed_capacitance_up);
  float loss_C_up = calculateLoss(predictedVoltage_C_up, targetVoltage);

  // --- 4. Perturb capacitance downwards ---
  float perturbed_capacitance_down = capacitance - delta_capacitance;
  float predictedVoltage_C_down = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, panel_R, perturbed_capacitance_down);
  float loss_C_down = calculateLoss(predictedVoltage_C_down, targetVoltage);

  // --- 5. Approximate Parameter Derivatives (Numerical Central Difference) ---
  parameterGradients(0, 0) = (loss_R_up - loss_R_down) / (2 * delta_parameter); // Derivative wrt panel_R
  parameterGradients(1, 0) = (loss_C_up - loss_C_down) / (2 * delta_capacitance); // Derivative wrt capacitance

  return parameterGradients;
}

void updateParameters(Vector2f parameterGradients) {
    accumulatedParameterDerivatives += parameterGradients * timeStep; // Accumulate parameter derivatives over time (Integral action)

    panel_R -= parameterDerivativeLearningRate * accumulatedParameterDerivatives(0, 0); // Adjust panel_R
    capacitance -= parameterDerivativeLearningRate * accumulatedParameterDerivatives(1, 0); // Adjust capacitance

    // --- Add Constraints/Limits to Parameters if Necessary ---
    panel_R = max(panel_R, 0.1f);       // Example: Keep panel_R above a minimum value
    capacitance = max(capacitance, 1e-9f); // Example: Keep capacitance above a minimum value
}


void updatePWM(float gradient) {
  previousMomentum = momentumFactor * previousMomentum + (1 - momentumFactor) * gradient;
  pwmDutyCycle -= learningRate * momentum;
  setPWMDutyCycle(pwmDutyCycle);
}
