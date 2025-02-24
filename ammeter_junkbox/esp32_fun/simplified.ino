#include <ArduinoEigen.h>
#include <cmath>

// --- Global Variables ---
float V_OC = 20.0;          // Open Circuit Voltage
float I_SC = 5.0;          // Short Circuit Current (Example - adjust for your panel)
float capacitance = 1000e-6; // Capacitance (Farads) -  IMPORTANT: Set your actual value!
float panel_R = 1.0;         // Panel Series Resistance (Ohms) - IMPORTANT: Set your actual value!
float panel_R_SH = 100.0;      // Panel Shunt Resistance (Ohms) - TUNING PARAMETER
float diode_n = 1.5;         // Diode ideality factor - TUNING PARAMETER (typical 1-2)
float diode_Is = 1e-6;       // Diode saturation current - TUNING PARAMETER (Example - adjust based on panel/diode)
float thermal_voltage_Vt = 0.026; // Thermal voltage at room temp (kT/q) - Constant for room temp
float panel_C = 1.0;         // Panel Capacitance (Farads) - if known, mostly for advanced model and tuning guidance
float targetVoltageRatio = 0.8;
float targetVoltage;
float load_R_base = 10.0;    // Base Load Resistance (for model) - TUNING PARAMETER: Crucial for model accuracy!
float pwm_output_resistance = 0.1; // Resistance of PWM output MOSFET and wiring (example) - TUNING PARAMETER

const int voltagePin = A0;    // Analog pin for voltage sensing
const int pwmPin = 2;       // PWM output pin
const int pwmFrequency = 1000; // PWM frequency (Hz) - ADJUST if needed for application
const int pwmResolution = 8;  // PWM resolution (bits, 8-16 for ESP32)

float currentVoltageRaw = 0.0;
float currentVoltageFiltered = 0.0;
float previousVoltageFiltered = 0.0;
float pwmDutyCycle = 0.0;
float previousPWM = 0.0;
float integralError = 0.0;
float previousTime = 0.0;
float timeStep = 0.0;
float accumulatedOvershoot = 0.0; // Accumulate overshoot for penalty
float accumulatedUndershoot = 0.0; // Accumulate undershoot for penalty

float filterCoefficient = 0.05; // Filter coefficient - TUNING PARAMETER: Adjust for noise vs. responsiveness
float filterTimeConstant;
float samplingInterval = 0.01; // Sampling interval (seconds) - ADJUST for control loop bandwidth (1-10Hz target)
float systemTimeConstant;
float gradientDelayFactor = 1.0; // Delay factor for gradient calculation - TUNING PARAMETER: Adjust for system settling

float w_error = 1.0;         // Weight for voltage tracking error - TUNING PARAMETER: Primary tuning knob
float w_deriv = 0.1;         // Weight for voltage derivative (damping) - TUNING PARAMETER: For stability/oscillations
float w_integral = 0.01;        // Weight for integral error (steady-state error removal) - TUNING PARAMETER: For steady-state accuracy
float w_pwm_change = 0.001;    // Weight for PWM change smoothness - TUNING PARAMETER: For reducing PWM jitter
float w_overshoot = 0.005;     // Weight for overshoot penalty - TUNING PARAMETER: To reduce voltage overshoot
float w_undershoot = 0.005;    // Weight for undershoot penalty - TUNING PARAMETER: To reduce voltage undershoot
float learningRate = 0.01;     // Learning Rate for PWM gradient descent - TUNING PARAMETER: Overall responsiveness
float delta_PWM = 0.01;        // PWM perturbation step for gradient approx - TUNING PARAMETER: Numerical gradient sensitivity
float momentumFactor = 0.9;    // Momentum for gradient descent - TUNING PARAMETER: For faster convergence/damping
float previousMomentum = 0.0;


// --- Define Matrix and Vector Types using Eigen ---
typedef Eigen::Matrix<float, 1, 1> Matrix1f;
typedef Eigen::Matrix<float, 1, 1> Vector1f;

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
  Serial.print("Gradient Delay (Improved Code): "); Serial.print(gradientDelayMs); Serial.println(" ms");

  // --- Initialize Weighting Matrices ---
  Q_matrix(0, 0) = w_error;
  Ru_matrix(0, 0) = w_pwm_change;
  Qo_matrix(0, 0) = w_overshoot;
  Qu_matrix(0, 0) = w_undershoot;

  // --- Initialize State and Control Vectors ---
  y_target_vec(0, 0) = targetVoltage;
  u_control(0, 0) = pwmDutyCycle;
  x_state(0, 0) = currentVoltageFiltered;
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
  float gradient = calculateGradient(loss, pwmDutyCycle);
  updatePWM(gradient);

  previousVoltageFiltered = currentVoltageFiltered;
  previousPWM = pwmDutyCycle;

  Serial.print("Voltage (Raw): "); Serial.print(currentVoltageRaw);
  Serial.print(", Voltage (Filtered): "); Serial.print(currentVoltageFiltered);
  Serial.print(", PWM: "); Serial.println(pwmDutyCycle);

  delay(static_cast<int>(samplingInterval * 1000));
}

float readSolarVoltageRaw() {
  int rawValue = analogRead(voltagePin);
  float voltage = rawValue * (3.3 / 4095.0) * voltageScaleFactor; // Adjust voltageScaleFactor based on your voltage divider/scaling
  return voltage;
}

float applyFilter(float rawVoltage) {
  // --- First-Order IIR Filter ---
  // filterCoefficient is related to filter time constant and sampling interval
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

  // Accumulate overshoot/undershoot for penalty terms (hysteresis-like)
  if (currentVoltage > targetVoltage) {
    accumulatedOvershoot += max(0.0f, currentVoltage - targetVoltage) * timeStep; // Only accumulate positive overshoot
    accumulatedUndershoot = 0; // Reset undershoot accumulation if over target
  } else {
    accumulatedUndershoot += max(0.0f, targetVoltage - currentVoltage) * timeStep; // Only accumulate positive undershoot
    accumulatedOvershoot = 0; // Reset overshoot accumulation if under target
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
  Matrix1f undershoot_term = undershoot_vec.transpose() * Qu_matrix * Qu_vec;


  float loss = error_term(0, 0) + pwm_change_term(0, 0) + overshoot_term(0,0) + undershoot_term(0,0);
  return loss;
}

// --- More Complex Model-Based Gradient Calculation ---
float calculateGradient(float currentLoss, float currentPWM) {
  float gradient;
  float deltaPWM_for_model = delta_PWM;

  // --- 1. Simulate Forward Perturbation (u + delta_u) ---
  float predictedVoltageUp = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM + deltaPWM_for_model);
  float lossUp = calculateLoss(predictedVoltageUp, targetVoltage);

  // --- 2. Simulate Backward Perturbation (u - delta_u) ---
  float predictedVoltageDown = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM - deltaPWM_for_model);
  float lossDown = calculateLoss(predictedVoltageDown, targetVoltage);

  // --- 3. Approximate Gradient ---
  gradient = (lossUp - lossDown) / (2 * deltaPWM_for_model);
  return gradient;
}

// --- Function to predict voltage one time step ahead using Enhanced Solar Cell Model ---
float predictVoltageStep(float currentVoltage, float currentPWM, float nextPWM) {
  float nextVoltage;
  float R_load_current = load_R_base / (currentPWM + 0.0001); // Avoid division by zero
  float R_load_next = load_R_base / (nextPWM + 0.0001);

  // --- Enhanced Solar Cell Model (Single Diode Model with Series and Shunt Resistance) ---
  float I_diode_current = diode_Is * (exp((currentVoltage + I_SC * panel_R) / (diode_n * thermal_voltage_Vt)) - 1.0); // Diode current
  float I_pv_current = I_SC - I_diode_current - (currentVoltage + I_SC * panel_R) / panel_R_SH; // Photovoltaic current

  // Discrete-time approximation with enhanced solar cell model and panel capacitor
  nextVoltage = currentVoltage + (timeStep / capacitance) * (I_pv_current - currentVoltage / (panel_R + pwm_output_resistance +  R_load_next) ); // Includes pwm output resistance in load path


  return nextVoltage;
}


void updatePWM(float gradient) {
  previousMomentum = momentumFactor * previousMomentum + (1 - momentumFactor) * gradient;
  pwmDutyCycle -= learningRate * momentum;
  setPWMDutyCycle(pwmDutyCycle);
}
