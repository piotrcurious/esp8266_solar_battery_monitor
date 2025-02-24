#include <ArduinoEigen.h>
#include <cmath>

// --- Global Variables (Same as before, but comments slightly adjusted for focus) ---
float V_OC = 20.0;
float I_SC = 5.0;
float capacitance = 1000e-6;
float panel_R = 1.0;
float panel_R_SH = 100.0;
float diode_n = 1.5;
float diode_Is = 1e-6;
float thermal_voltage_Vt = 0.026;
float panel_C = 1.0;
float targetVoltageRatio = 0.8;
float targetVoltage;
float load_R_base = 10.0;
float pwm_output_resistance = 0.1;

const int voltagePin = A0;
const int pwmPin = 2;
const int pwmFrequency = 1000;
const int pwmResolution = 8;

float currentVoltageRaw = 0.0;
float currentVoltageFiltered = 0.0;
float previousVoltageFiltered = 0.0;
float pwmDutyCycle = 0.0;
float previousPWM = 0.0;
float integralError = 0.0;
float previousError = 0.0; // For derivative of error calculation - NEW
float previousTime = 0.0;
float timeStep = 0.0;
float accumulatedOvershoot = 0.0;
float accumulatedUndershoot = 0.0;

float filterCoefficient = 0.05;
float filterTimeConstant;
float samplingInterval = 0.01;
float systemTimeConstant;
float gradientDelayFactor = 5.0;
unsigned long gradientDelayMs;

float w_error_base = 1.0;       // Base weight for voltage error - BASE WEIGHT
float w_deriv = 0.1;
float w_integral = 0.01;
float w_pwm_change = 0.001;
float w_overshoot_base = 0.005;  // Base weight for overshoot penalty - BASE WEIGHT
float w_undershoot_base = 0.005; // Base weight for undershoot penalty - BASE WEIGHT
float learningRate = 0.01;
float delta_PWM = 0.01;
float momentumFactor = 0.9;
float previousMomentum = 0.0;

float parameterDerivativeLearningRate_base = 0.00001; // Base learning rate for parameter adaptation - BASE LEARNING RATE
float delta_parameter_R = 0.1 * panel_R;
float delta_parameter_C = 0.1 * capacitance;
float parameter_momentum_factor = 0.8; // Momentum for parameter adaptation - NEW
Vector2f previousParameterMomentum;    // Momentum term for parameter adaptation - NEW
Vector2f adaptive_parameterDerivativeLearningRate; // Adaptive learning rate for parameters - NEW

// --- Define Matrix and Vector Types using Eigen ---
typedef Eigen::Matrix<float, 1, 1> Matrix1f;
typedef Eigen::Matrix<float, 1, 1> Vector1f;
typedef Eigen::Matrix<float, 2, 1> Vector2f;

// --- Weighting Matrices for Loss Function ---
Matrix1f Q_matrix;
Matrix1f Ru_matrix;
Matrix1f Qo_matrix;
Matrix1f Qu_matrix;
Matrix1f Qd_matrix; // Weight for derivative of error - NEW

// --- State and Control Vectors ---
Vector1f x_state;
Vector1f u_control;
Vector1f y_output;
Vector1f y_target_vec;

// --- Accumulated Parameter Derivatives Vector ---
Vector2f accumulatedParameterDerivatives;


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
  previousError = currentVoltageFiltered - targetVoltage; // Initialize previous error - NEW

  systemTimeConstant = panel_R * capacitance;
  filterTimeConstant = samplingInterval / filterCoefficient - samplingInterval;
  filterCoefficient = samplingInterval / (samplingInterval + filterTimeConstant);
  gradientDelayMs = static_cast<unsigned long>(gradientDelayFactor * systemTimeConstant * 1000);

  Serial.print("System Time Constant: "); Serial.print(systemTimeConstant); Serial.println(" s");
  Serial.print("Filter Time Constant: "); Serial.print(filterTimeConstant); Serial.println(" s");
  Serial.print("Gradient Delay (Integrated, Adaptive Param, Adv Loss): "); Serial.print(gradientDelayMs); Serial.println(" ms");

  // --- Initialize Weighting Matrices ---
  Q_matrix(0, 0) = w_error_base;
  Ru_matrix(0, 0) = w_pwm_change;
  Qo_matrix(0, 0) = w_overshoot_base;
  Qu_matrix(0, 0) = w_undershoot_base;
  Qd_matrix(0,0) = w_deriv; // Derivative of error weight - NEW

  // --- Initialize State and Control Vectors ---
  y_target_vec(0, 0) = targetVoltage;
  u_control(0, 0) = pwmDutyCycle;
  x_state(0, 0) = currentVoltageFiltered;

  // --- Initialize Parameter Derivative Accumulators and Momentum ---
  accumulatedParameterDerivatives.setZero();
  previousParameterMomentum.setZero();
  adaptive_parameterDerivativeLearningRate.setConstant(parameterDerivativeLearningRate_base); // Initialize adaptive learning rate - NEW
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
  Vector2f parameterGradients = calculateParameterDerivatives(loss, currentVoltageFiltered, pwmDutyCycle);
  updateParameters(parameterGradients);

  float gradient = calculateGradient(loss, pwmDutyCycle);
  updatePWM(gradient);

  previousVoltageFiltered = currentVoltageFiltered;
  previousPWM = pwmDutyCycle;
  previousError = currentVoltageFiltered - targetVoltage; // Update previous error - NEW

  // --- Serial Output for Monitoring - Now with Loss Components and Parameter Derivatives ---
  float error = currentVoltageFiltered - targetVoltage;
  Vector1f error_vec; error_vec(0,0) = error;
  Vector1f pwm_change_vec; pwm_change_vec(0,0) = pwmDutyCycle - previousPWM;
  Vector1f overshoot_vec; overshoot_vec(0,0) = accumulatedOvershoot;
  Vector1f undershoot_vec; undershoot_vec(0,0) = accumulatedUndershoot;
  Vector1f deriv_error_vec; deriv_error_vec(0,0) = (error - previousError)/timeStep; // Derivative of error - NEW

  Matrix1f error_term = error_vec.transpose() * Q_matrix * error_vec;
  Matrix1f pwm_change_term = pwm_change_vec.transpose() * Ru_matrix * pwm_change_vec;
  Matrix1f overshoot_term = overshoot_vec.transpose() * Qo_matrix * overshoot_vec;
  Matrix1f undershoot_term = undershoot_vec.transpose() * Qu_matrix * Qu_vec;
  Matrix1f deriv_error_term = deriv_error_vec.transpose() * Qd_matrix * deriv_error_vec; // Derivative error term - NEW


  Serial.print("Voltage (Filt): "); Serial.print(currentVoltageFiltered);
  Serial.print(", PWM: "); Serial.print(pwmDutyCycle);
  Serial.print(", Loss: "); Serial.print(loss);
  Serial.print(", Error Term: "); Serial.print(error_term(0,0));
  Serial.print(", PWM Change Term: "); Serial.print(pwm_change_term(0,0));
  Serial.print(", Overshoot Term: "); Serial.print(overshoot_term(0,0));
  Serial.print(", Undershoot Term: "); Serial.print(undershoot_term(0,0));
  Serial.print(", Deriv Error Term: "); Serial.print(deriv_error_term(0,0)); // Print derivative error term - NEW
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
  float error = error_vec(0,0); // Scalar error for dynamic weight calculation

  integralError += error * timeStep;

  // --- Dynamic Weighting of Overshoot and Undershoot Penalties based on Error Magnitude ---
  float overshootWeight = w_overshoot_base * (1.0 + abs(error)); // Overshoot weight increases with error magnitude - DYNAMIC WEIGHT
  float undershootWeight = w_undershoot_base * (1.0 + abs(error)); // Undershoot weight increases with error magnitude - DYNAMIC WEIGHT
  Qo_matrix(0, 0) = overshootWeight; // Update overshoot weight matrix - DYNAMIC WEIGHT
  Qu_matrix(0, 0) = undershootWeight; // Update undershoot weight matrix - DYNAMIC WEIGHT


  if (currentVoltage > targetVoltage) {
    accumulatedOvershoot += max(0.0f, currentVoltage - targetVoltage) * timeStep;
    accumulatedUndershoot = 0;
  } else {
    accumulatedUndershoot += max(0.0f, targetVoltage - currentVoltage) * timeStep;
    accumulatedOvershoot = 0;
  }
  Vector1f overshoot_vec;
  overshoot_vec(0, 0) = accumulatedOvershoot;
  Vector1f undershoot_vec;
  undershoot_vec(0, 0) = accumulatedUndershoot;

  Vector1f pwm_change_vec;
  pwm_change_vec(0, 0) = pwmDutyCycle - previousPWM;

  Vector1f deriv_error_vec; // Derivative of error term - NEW
  deriv_error_vec(0,0) = (error - previousError)/timeStep;


  Matrix1f error_term = error_vec.transpose() * Q_matrix * error_vec;
  Matrix1f pwm_change_term = pwm_change_vec.transpose() * Ru_matrix * pwm_change_vec;
  Matrix1f overshoot_term = overshoot_vec.transpose() * Qo_matrix * overshoot_vec;
  Matrix1f undershoot_term = undershoot_vec.transpose() * Qu_matrix * Qu_vec;
  Matrix1f deriv_error_term = deriv_error_vec.transpose() * Qd_matrix * deriv_error_vec; // Derivative error term - NEW


  float loss = error_term(0, 0) + pwm_change_term(0, 0) + overshoot_term(0,0) + undershoot_term(0,0) + deriv_error_term(0,0); // Loss now includes derivative error term - NEW
  return loss;
}

float calculateGradient(float currentLoss, float currentPWM) {
  float gradient;
  float deltaPWM_for_model = delta_PWM;

  float predictedVoltageUp = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM + deltaPWM_for_model, panel_R, capacitance);
  float lossUp = calculateLoss(predictedVoltageUp, targetVoltage);

  float predictedVoltageDown = predictVoltageStep(currentVoltageFiltered, currentPWM, currentPWM - deltaPWM_for_model, panel_R, capacitance);
  float lossDown = calculateLoss(predictedVoltageDown, targetVoltage);

  gradient = (lossUp - lossDown) / (2 * deltaPWM_for_model);
  return gradient;
}

float predictVoltageStep(float currentVoltage, float currentPWM, float nextPWM, float current_panel_R, float current_capacitance) {
  float nextVoltage;
  float R_load_current = load_R_base / (currentPWM + 0.0001);
  float R_load_next = load_R_base / (nextPWM + 0.0001);

  float I_diode_current = diode_Is * (exp((currentVoltage + I_SC * current_panel_R) / (diode_n * thermal_voltage_Vt)) - 1.0);
  float I_pv_current = I_SC - I_diode_current - (currentVoltage + I_SC * current_panel_R) / panel_R_SH;

  nextVoltage = currentVoltage + (timeStep / current_capacitance) * (I_pv_current - currentVoltage / (current_panel_R + pwm_output_resistance +  R_load_next) );

  return nextVoltage;
}

Vector2f calculateParameterDerivatives(float currentLoss, float currentVoltage, float currentPWM) {
  Vector2f parameterGradients;
  float delta_R = delta_parameter_R;
  float delta_C = delta_parameter_C;

  float perturbed_panel_R_up = panel_R + delta_R;
  float predictedVoltage_R_up = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, perturbed_panel_R_up, capacitance);
  float loss_R_up = calculateLoss(predictedVoltage_R_up, targetVoltage);

  float perturbed_panel_R_down = panel_R - delta_R;
  float predictedVoltage_R_down = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, perturbed_panel_R_down, capacitance);
  float loss_R_down = calculateLoss(predictedVoltage_R_down, targetVoltage);

  float perturbed_capacitance_up = capacitance + delta_C;
  float predictedVoltage_C_up = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, panel_R, perturbed_capacitance_up);
  float loss_C_up = calculateLoss(predictedVoltage_C_up, targetVoltage);

  float perturbed_capacitance_down = capacitance - delta_C;
  float predictedVoltage_C_down = predictVoltageStep(currentVoltage, currentPWM, pwmDutyCycle, panel_R, perturbed_capacitance_down);
  float loss_C_down = calculateLoss(predictedVoltage_C_down, targetVoltage);

  parameterGradients(0, 0) = (loss_R_up - loss_R_down) / (2 * delta_R);
  parameterGradients(1, 0) = (loss_C_up - loss_C_down) / (2 * delta_C);

  return parameterGradients;
}

void updateParameters(Vector2f parameterGradients) {
  accumulatedParameterDerivatives += parameterGradients * timeStep;

  // --- Parameter Adaptation with Momentum and Adaptive Learning Rate ---
  previousParameterMomentum = parameter_momentum_factor * previousParameterMomentum + (1 - parameter_momentum_factor) * parameterGradients; // Momentum for parameter updates
  adaptive_parameterDerivativeLearningRate = adaptive_parameterDerivativeLearningRate.array() * (1.0 + 0.01 * parameterGradients.array().abs()); // Example adaptive LR - increases LR if gradient is consistently large

  panel_R -= adaptive_parameterDerivativeLearningRate(0, 0) * previousParameterMomentum(0, 0); // Apply momentum and adaptive LR to panel_R
  capacitance -= adaptive_parameterDerivativeLearningRate(1, 0) * previousParameterMomentum(1, 0); // Apply momentum and adaptive LR to capacitance


  panel_R = max(panel_R, 0.01f);
  capacitance = max(capacitance, 1e-7f);
}


void updatePWM(float gradient) {
  previousMomentum = momentumFactor * previousMomentum + (1 - momentumFactor) * gradient;
  pwmDutyCycle -= learningRate * momentum;
  setPWMDutyCycle(pwmDutyCycle);
}
