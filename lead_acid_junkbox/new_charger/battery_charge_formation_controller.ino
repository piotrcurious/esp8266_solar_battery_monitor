/*
 * Battery Formation / Charge Controller — reviewed & extended
 * ------------------------------------------------------------
 * Original code: single-direction current-sink formation test with an
 * (incorrectly implemented) Kalman filter for detecting an outgassing
 * voltage plateau.
 *
 * Fixes vs. original:
 *   1) Kalman covariance PREDICT was P_pred = A*P + Q. Missing the
 *      A^T term entirely -> should be A*P*A^T + Q. Fixed below.
 *   2) Kalman covariance UPDATE did a row-wise scalar (1 - K[i]*H[i])
 *      instead of the full outer-product matrix (I - K*H). Since
 *      H = [1,0], this silently left P[1][*] essentially uncorrected
 *      by measurements. Fixed with full 2x2 matrix update.
 *   3) Control loop period was 1000 ms, too slow for current-mode PID
 *      or an MPPT perturb & observe loop. Reduced to 100 ms.
 *
 * New capabilities added:
 *   - CC/CV charge state machine (separate from the discharge/
 *     formation test path) with configurable target voltage,
 *     termination current, timeout, and safety cutoffs.
 *   - Transient-source support (e.g. solar panel): input voltage
 *     under-voltage lockout (UVLO) with hysteresis so charging backs
 *     off instead of collapsing the source, plus a simple Perturb &
 *     Observe MPPT loop that hunts for the source's max-power duty
 *     cycle rather than assuming a stiff supply.
 *   - Basic fault handling (overvoltage / overcurrent / input loss).
 *
 * HARDWARE ASSUMPTIONS (adjust to your actual power stage):
 *   - PWM_DISCHARGE_PIN drives a MOSFET current sink used for the
 *     formation/outgassing test (this is your original circuit).
 *   - PWM_CHARGE_PIN drives a separate buck (or buck-boost) converter
 *     that delivers current FROM the source (e.g. solar panel) INTO
 *     the battery. If your hardware only has one power stage, you'll
 *     need to either add the second one or adapt this to a
 *     bidirectional converter — the control/state-machine logic below
 *     is the same either way.
 *   - Voltage dividers and current-sense scale factors under
 *     "Calibration" are placeholders. Measure your actual dividers
 *     and shunt/sense-amp gain and update them.
 *   - No temperature sensing is included. For real battery charging,
 *     strongly consider adding an NTC thermistor and a temperature
 *     cutoff/derate — charging without thermal protection is a real
 *     safety gap, not just a nice-to-have.
 */

#include <PID_v1.h>

// ================= Pin Assignments =================
const int PWM_DISCHARGE_PIN = 9;   // formation/outgassing test sink MOSFET
const int PWM_CHARGE_PIN    = 10;  // charge-path buck converter MOSFET
const int BUTTON_PIN        = 2;
const int BATT_V_PIN        = A0;  // battery voltage divider
const int BATT_I_PIN        = A1;  // battery current shunt/sense amp
const int VIN_PIN           = A2;  // source (e.g. solar panel) voltage divider
const int IIN_PIN           = A3;  // source current sense (for MPPT power calc)

// ================= Calibration (placeholders — measure your hardware) =================
const float VREF = 5.0;
const float ADC_MAX = 1023.0;
const float BATT_V_DIVIDER_RATIO = 2.0;    // Vbatt = Vadc * ratio
const float VIN_DIVIDER_RATIO    = 6.0;    // for a higher-voltage panel input
const float BATT_I_SCALE_MA      = 500.0;  // mA full-scale on BATT_I_PIN
const float IIN_I_SCALE_MA       = 2000.0; // mA full-scale on IIN_PIN

inline float readBatteryVoltage() {
  return (analogRead(BATT_V_PIN) * (VREF / ADC_MAX)) * BATT_V_DIVIDER_RATIO;
}
inline float readBatteryCurrentMa() {
  // Signed: implement your sense-amp's zero-current offset here if needed.
  return analogRead(BATT_I_PIN) * (BATT_I_SCALE_MA / ADC_MAX);
}
inline float readInputVoltage() {
  return (analogRead(VIN_PIN) * (VREF / ADC_MAX)) * VIN_DIVIDER_RATIO;
}
inline float readInputCurrentMa() {
  return analogRead(IIN_PIN) * (IIN_I_SCALE_MA / ADC_MAX);
}

// ================= Safety / charge parameters (tune to your chemistry) =================
const float CHARGE_V_TARGET       = 4.20f;   // e.g. Li-ion single cell CV target
const float CHARGE_V_OVERVOLTAGE  = 4.30f;   // hard cutoff
const float CHARGE_CC_TARGET_MA   = 500.0f;  // bulk/CC charge current
const float CHARGE_TERMINATION_MA = 25.0f;   // taper current -> "done" (~C/20)
const unsigned long CHARGE_TIMEOUT_MS = (unsigned long)6 * 60UL * 60UL * 1000UL; // 6h safety timeout

const float VIN_UVLO_FALLING = 4.5f;   // below this, back off / stop charging
const float VIN_UVLO_RISING  = 5.0f;   // must recover above this to resume (hysteresis)

const float OVERCURRENT_LIMIT_MA = 1500.0f; // hard fault threshold, either direction

const float slopeThreshold = 0.0005f;  // V per control step, tune for your outgassing detection

// ================= Legacy formation/discharge test params =================
float setCurrent = 500.0f; // mA, discharge/formation target (mutable, tapers on plateau)

// ================= Operating Modes =================
enum Mode {
  MODE_IDLE,
  MODE_DISCHARGE_FORMATION,
  MODE_WAIT_AFTER_OUTGASSING,
  MODE_CHARGE_CC,
  MODE_CHARGE_CV,
  MODE_CHARGE_DONE,
  MODE_FAULT
};
volatile Mode mode = MODE_IDLE;
bool inputOk = true; // tracks UVLO state with hysteresis

// ================= Timing =================
unsigned long previousMillis = 0;
const unsigned long CONTROL_INTERVAL_MS = 100; // 10 Hz control loop
unsigned long stateEnteredAt = 0;
unsigned long chargeStartedAt = 0;

// ================= Kalman filter state (tracks battery voltage & its slope) =================
// current_input convention: positive = net current INTO the battery (charging),
// negative = net current OUT of the battery (discharge/formation test).
float voltage_est = 0.0f;
float slope_est   = 0.0f;
float internalResistance = 0.05f; // ohms, rough series-resistance estimate

float P[2][2] = { {1, 0}, {0, 1} };
const float Qn[2][2] = { {0.0005f, 0}, {0, 0.0005f} };
const float R_meas = 0.02f; // voltage measurement noise variance

float A[2][2];
float Bc[2];
const float H[2] = {1, 0};

void kalmanSetTimestep(float dt) {
  A[0][0] = 1; A[0][1] = dt;
  A[1][0] = 0; A[1][1] = 1;
  Bc[0] = internalResistance * dt;
  Bc[1] = 0;
}

void kalmanUpdate(float current_input_ma, float measured_voltage) {
  float current_input = current_input_ma / 1000.0f; // amps, keeps IR term sane

  // ---- Predict: x_pred = A*x + B*u ----
  float x_pred[2];
  x_pred[0] = A[0][0] * voltage_est + A[0][1] * slope_est + Bc[0] * current_input;
  x_pred[1] = A[1][0] * voltage_est + A[1][1] * slope_est + Bc[1] * current_input;

  // AP = A * P
  float AP[2][2];
  AP[0][0] = A[0][0]*P[0][0] + A[0][1]*P[1][0];
  AP[0][1] = A[0][0]*P[0][1] + A[0][1]*P[1][1];
  AP[1][0] = A[1][0]*P[0][0] + A[1][1]*P[1][0];
  AP[1][1] = A[1][0]*P[0][1] + A[1][1]*P[1][1];

  // P_pred = AP * A^T + Q   <-- the A^T multiply was missing in the original
  float P_pred[2][2];
  P_pred[0][0] = AP[0][0]*A[0][0] + AP[0][1]*A[0][1] + Qn[0][0];
  P_pred[0][1] = AP[0][0]*A[1][0] + AP[0][1]*A[1][1] + Qn[0][1];
  P_pred[1][0] = AP[1][0]*A[0][0] + AP[1][1]*A[0][1] + Qn[1][0];
  P_pred[1][1] = AP[1][0]*A[1][0] + AP[1][1]*A[1][1] + Qn[1][1];

  // ---- Update ----
  float y = measured_voltage - (H[0]*x_pred[0] + H[1]*x_pred[1]);
  float S = H[0]*P_pred[0][0] + H[1]*P_pred[1][0] + R_meas;
  float K[2];
  K[0] = (P_pred[0][0]*H[0] + P_pred[0][1]*H[1]) / S;
  K[1] = (P_pred[1][0]*H[0] + P_pred[1][1]*H[1]) / S;

  voltage_est = x_pred[0] + K[0]*y;
  slope_est   = x_pred[1] + K[1]*y;

  // P = (I - K*H) * P_pred, full outer-product form (K is 2x1, H is 1x2)
  float KH00 = K[0]*H[0], KH01 = K[0]*H[1];
  float KH10 = K[1]*H[0], KH11 = K[1]*H[1];
  float newP00 = (1-KH00)*P_pred[0][0] - KH01*P_pred[1][0];
  float newP01 = (1-KH00)*P_pred[0][1] - KH01*P_pred[1][1];
  float newP10 = -KH10*P_pred[0][0] + (1-KH11)*P_pred[1][0];
  float newP11 = -KH10*P_pred[0][1] + (1-KH11)*P_pred[1][1];
  P[0][0]=newP00; P[0][1]=newP01; P[1][0]=newP10; P[1][1]=newP11;
}

// ================= Discharge/formation-test PID (unchanged concept) =================
double pidIn_discharge, pidOut_discharge, pidSet_discharge;
PID pidDischarge(&pidIn_discharge, &pidOut_discharge, &pidSet_discharge, 2.0, 5.0, 0.5, DIRECT);

// ================= Charge CC-phase PID =================
double pidIn_charge, pidOut_charge, pidSet_charge;
PID pidCharge(&pidIn_charge, &pidOut_charge, &pidSet_charge, 2.0, 5.0, 0.5, DIRECT);

// ================= MPPT (Perturb & Observe) state =================
float mpptDuty = 40.0f;         // 0-255 duty applied to PWM_CHARGE_PIN, seed value
float mpptLastPower = 0.0f;
float mpptStep = 3.0f;          // duty step size
int   mpptDirection = 1;
unsigned long mpptLastUpdate = 0;
const unsigned long MPPT_INTERVAL_MS = 1000; // P&O needs to be slower than the inner current loop

void mpptUpdate(float vin, float iin_ma) {
  if (millis() - mpptLastUpdate < MPPT_INTERVAL_MS) return;
  mpptLastUpdate = millis();

  float power = vin * (iin_ma / 1000.0f);
  if (power < mpptLastPower) {
    mpptDirection = -mpptDirection; // wrong way, reverse
  }
  mpptDuty += mpptDirection * mpptStep;
  mpptDuty = constrain(mpptDuty, 0.0f, 255.0f);
  mpptLastPower = power;
}

// ================= UVLO with hysteresis =================
bool checkInputOk(float vin) {
  if (inputOk && vin < VIN_UVLO_FALLING) inputOk = false;
  if (!inputOk && vin > VIN_UVLO_RISING) inputOk = true;
  return inputOk;
}

// ================= Mode transitions =================
void enterMode(Mode m) {
  mode = m;
  stateEnteredAt = millis();
  Serial.print("-> Mode: ");
  Serial.println(m);
}

void startDischargeFormation() {
  setCurrent = 500.0f;
  pidSet_discharge = setCurrent;
  pidDischarge.SetMode(AUTOMATIC);
  analogWrite(PWM_CHARGE_PIN, 0);
  enterMode(MODE_DISCHARGE_FORMATION);
}

void startCharge() {
  pidSet_charge = CHARGE_CC_TARGET_MA;
  pidCharge.SetMode(AUTOMATIC);
  analogWrite(PWM_DISCHARGE_PIN, 0);
  mpptDuty = 40.0f;
  mpptLastPower = 0.0f;
  chargeStartedAt = millis();
  enterMode(MODE_CHARGE_CC);
}

void tripFault(const char* reason) {
  analogWrite(PWM_DISCHARGE_PIN, 0);
  analogWrite(PWM_CHARGE_PIN, 0);
  Serial.print("FAULT: ");
  Serial.println(reason);
  enterMode(MODE_FAULT);
}

// ================= Setup =================
void setup() {
  pinMode(PWM_DISCHARGE_PIN, OUTPUT);
  pinMode(PWM_CHARGE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  kalmanSetTimestep(CONTROL_INTERVAL_MS / 1000.0f);
  voltage_est = readBatteryVoltage();
  slope_est = 0.0f;

  pidDischarge.SetOutputLimits(0, 255);
  pidCharge.SetOutputLimits(0, 255);
  pidDischarge.SetSampleTime(CONTROL_INTERVAL_MS);
  pidCharge.SetSampleTime(CONTROL_INTERVAL_MS);

  enterMode(MODE_IDLE);
  Serial.println("Ready. Serial commands: 'D' = start discharge/formation test, 'C' = start charge.");
}

// ================= Main loop =================
void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    char c = Serial.read();
    if ((c == 'D' || c == 'd') && mode == MODE_IDLE) startDischargeFormation();
    if ((c == 'C' || c == 'c') && mode == MODE_IDLE) startCharge();
  }
  if (digitalRead(BUTTON_PIN) == LOW && mode == MODE_IDLE) {
    startDischargeFormation();
  }

  if (currentMillis - previousMillis < CONTROL_INTERVAL_MS) return;
  float dt = (currentMillis - previousMillis) / 1000.0f;
  previousMillis = currentMillis;
  kalmanSetTimestep(dt);

  float vBatt = readBatteryVoltage();
  float vIn   = readInputVoltage();

  // Global protections, checked every cycle regardless of mode
  if (vBatt > CHARGE_V_OVERVOLTAGE) { tripFault("battery overvoltage"); return; }

  switch (mode) {

    case MODE_IDLE:
    case MODE_FAULT:
      // sit idle; MODE_FAULT requires a reset or a future "clear fault" command
      break;

    case MODE_DISCHARGE_FORMATION: {
      float iBatt = readBatteryCurrentMa();
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("discharge overcurrent"); break; }

      kalmanUpdate(-iBatt, vBatt); // discharge current is negative by convention

      pidIn_discharge = iBatt;
      pidSet_discharge = setCurrent;
      pidDischarge.Compute();
      analogWrite(PWM_DISCHARGE_PIN, (int)pidOut_discharge);

      if (fabs(slope_est) < slopeThreshold && (currentMillis - stateEnteredAt) > 5000) {
        Serial.print("Outgassing plateau detected at voltage: ");
        Serial.println(voltage_est, 4);
        analogWrite(PWM_DISCHARGE_PIN, 0);
        enterMode(MODE_WAIT_AFTER_OUTGASSING);
      }

      Serial.print("V="); Serial.print(voltage_est, 3);
      Serial.print(" slope="); Serial.print(slope_est, 5);
      Serial.print(" I="); Serial.print(iBatt);
      Serial.println(" mA");
      break;
    }

    case MODE_WAIT_AFTER_OUTGASSING:
      if (currentMillis - stateEnteredAt >= 20UL * 60UL * 1000UL) {
        setCurrent -= 50.0f;
        if (setCurrent <= 0) {
          Serial.println("Formation test complete.");
          enterMode(MODE_IDLE);
        } else {
          Serial.println("Resuming formation test with lower current...");
          enterMode(MODE_DISCHARGE_FORMATION);
        }
      }
      break;

    case MODE_CHARGE_CC:
    case MODE_CHARGE_CV: {
      if (!checkInputOk(vIn)) {
        // Source sagging (e.g. cloud passing over the solar panel) — back off
        // instead of collapsing it, but stay in the charge mode and resume
        // automatically once the source recovers (hysteresis in checkInputOk).
        analogWrite(PWM_CHARGE_PIN, 0);
        Serial.println("UVLO: input voltage low, charge paused.");
        break;
      }
      if (currentMillis - chargeStartedAt > CHARGE_TIMEOUT_MS) {
        tripFault("charge timeout");
        break;
      }

      float iBatt = readBatteryCurrentMa();
      float iIn   = readInputCurrentMa();
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("charge overcurrent"); break; }

      kalmanUpdate(iBatt, vBatt); // charging current is positive by convention

      // MPPT hunts the duty cycle that maximizes source power delivery.
      mpptUpdate(vIn, iIn);

      if (mode == MODE_CHARGE_CC) {
        pidIn_charge = iBatt;
        pidSet_charge = CHARGE_CC_TARGET_MA;
        pidCharge.Compute();
        // Current-mode PID output is capped by whatever MPPT says the source
        // can actually deliver, so we don't demand more than is available.
        float duty = min((float)pidOut_charge, mpptDuty);
        analogWrite(PWM_CHARGE_PIN, (int)duty);

        if (voltage_est >= CHARGE_V_TARGET) {
          enterMode(MODE_CHARGE_CV);
        }
      } else { // MODE_CHARGE_CV: hold voltage, let current taper
        float vError = CHARGE_V_TARGET - voltage_est;
        float duty = pidOut_charge + vError * 40.0f; // simple proportional taper
        duty = constrain(duty, 0.0f, min(255.0f, mpptDuty));
        analogWrite(PWM_CHARGE_PIN, (int)duty);
        pidOut_charge = duty;

        if (iBatt < CHARGE_TERMINATION_MA) {
          analogWrite(PWM_CHARGE_PIN, 0);
          Serial.println("Charge complete (termination current reached).");
          enterMode(MODE_CHARGE_DONE);
        }
      }

      Serial.print("Vbatt="); Serial.print(voltage_est, 3);
      Serial.print(" Ibatt="); Serial.print(iBatt);
      Serial.print(" Vin="); Serial.print(vIn, 2);
      Serial.print(" mpptDuty="); Serial.println(mpptDuty, 1);
      break;
    }

    case MODE_CHARGE_DONE:
      // trickle/float logic could go here if your chemistry wants it
      break;
  }
}
