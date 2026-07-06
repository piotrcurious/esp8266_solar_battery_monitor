/*
 * ESP32 Solar Battery Charge / Formation Controller
 * ---------------------------------------------------
 * Charge algorithm (12V lead-acid style, adjust constants for your chemistry):
 *
 *   1) BULK: charge at whatever current the fractional-Voc MPPT loop
 *      says the panel can deliver (duty = mpptDutyCeiling, applied
 *      directly, open-loop w.r.t. current) until battery voltage rises
 *      past BULK_TARGET_V (13.8V default) -- a simple, unfiltered
 *      threshold meaning "roughly full," not the outgassing point.
 *
 *   2) PARASITIC BASELINE: the charge FET is not simply switched off --
 *      with it fully open no current flows and there's nothing for the
 *      INA219 shunt to measure. A small closed voltage-hold loop
 *      targets a setpoint slightly ABOVE the battery's idle voltage;
 *      the steady-state current needed to hold that is the current
 *      needed to counteract self-discharge plus the system's own
 *      parasitic draw. This also sets the lower bound for the search
 *      below: no current at or below it can produce any net voltage
 *      rise, let alone reach outgassing.
 *
 *   3) OUTGASSING CHARACTERIZATION -- rethought as a bisection search
 *      bracketed by high-current and low-current pulses, not a linear
 *      ramp:
 *
 *      a) FIRST pulse uses the maximum current the solar system can
 *         actually deliver (duty = mpptDutyCeiling, open-loop, same as
 *         bulk) -- "limited only by solar system efficiency." A strong
 *         pulse gets through the pre-gassing capacitive charging phase
 *         fast, and if the battery is already near its gassing
 *         potential, it will cross into that regime within the SAME
 *         pulse. The early portion of this pulse (before any knee) is
 *         used to establish the reference "healthy" capacitance,
 *         C_baseline, from dV/dt = I/C. If this pulse shows a knee
 *         (see (c)), it becomes the upper bound (known-gassing current)
 *         for a bisection search. If it shows no knee at all, the
 *         battery simply hasn't reached its gassing voltage yet -- so
 *         another max-current pulse is run (equivalent to continued
 *         charging), not treated as a search failure, since voltage
 *         (not current) is the real limiting factor here.
 *
 *      b) Once a known-gassing upper bound exists, BISECTION pulses run
 *         at the midpoint between the parasitic-based floor and the
 *         current upper bound (closed-loop, PID-held at that specific
 *         target current). Each pulse's own trace answers "did THIS
 *         current show a knee" -- no repeated pulses per level needed,
 *         since a single well-analyzed high-SNR pulse is enough. The
 *         bracket halves each iteration until it narrows below
 *         BISECTION_RESOLUTION_MA; that converged value is, by
 *         construction, the MINIMUM current that reaches outgassing.
 *
 *      c) KNEE DETECTION, within a single pulse: an EMA-smoothed dV/dt
 *         is tracked continuously through the pulse. Its ratio to the
 *         pulse's own early ("pre-knee") slope should stay near 1 while
 *         charging remains purely capacitive, regardless of current --
 *         that's what makes this scale-free rather than a hardcoded
 *         volt or slope number. A sustained drop below
 *         OUTGAS_EFFICIENCY_THRESHOLD marks the knee: the voltage and
 *         current at that instant are the tentative outgassing point.
 *
 *      d) ENERGY PARTITION, logged for every pulse: at each control
 *         tick, I_cap = C_baseline * (measured dV/dt), clamped to the
 *         applied current; the remainder, I_gas = I_applied - I_cap, is
 *         attributed to the competing Faradaic (gas-evolution) reaction.
 *         Integrating V*I_cap and V*I_gas over the pulse gives the
 *         energy split between "went into raising voltage" and "went
 *         into gas instead" directly from the pulse log -- the energy
 *         lost to outgassing is exactly E_gas.
 *
 *      e) DECAY-SIDE VERIFICATION differs depending on whether a knee
 *         was seen: a pulse with NO knee gets the original single-
 *         exponential 3-point decay fit (tau_decay vs the charge-side
 *         tau, as before). A pulse WHERE a knee was confirmed gets a
 *         5-point bi-exponential "peeling" fit instead: hydrogen
 *         evolution is partially reversible, so the relaxation after a
 *         gassing pulse has a fast component (the same ohmic/double-
 *         layer relaxation as always) and a slower second component
 *         (recombination/diffusion-related). The fast time constant
 *         from THIS fit should roughly match the charge-side tau; if it
 *         instead reads very different, or if the "apparent capacitance"
 *         implied by the decay disagrees with C_baseline, that
 *         charge/discharge ASYMMETRY is itself corroborating evidence
 *         that a genuine Faradaic (not purely capacitive) process is
 *         underway -- logged, not used as a hard veto, since the
 *         two-exponential peel is the more fragile of the two fits.
 *
 *   4) FLOAT: hold the discovered outgassing voltage, taper current,
 *      stop once it falls below a termination threshold.
 *
 * CAVEATS worth being explicit about:
 *  - The bi-exponential peel assumes tau_slow is meaningfully larger
 *    than tau_fast; if a real cell's two time constants are close
 *    together the separation degrades. It's also inherently noise-
 *    sensitive (difference-of-differences arithmetic).
 *  - This whole RC/Faradaic-split model is a first-order proxy for real
 *    battery electrochemistry (which also has diffusion/Warburg-like
 *    behavior and voltage-dependent capacitance). It's a much better
 *    proxy than a fixed threshold, but still a proxy -- worth checking
 *    the discovered outgassing point against an independent reference
 *    (hydrometer, gas detection) at least once for your specific cell.
 *  - OUTGAS_EFFICIENCY_THRESHOLD (0.5) is the one number in the
 *    detection logic that isn't derived from measured physics; it's a
 *    judgment call and may need bench tuning for your chemistry.
 *
 * Carried over unchanged from the previous revision:
 *  - ESP32 port: LEDC PWM, ESP32 ADC for the one remaining analog
 *    input, Preferences (NVS) state persistence, esp_sleep deep sleep
 *    for panel-undervoltage shutdown.
 *  - No input current sensing; single INA219 on the battery node gives
 *    both voltage and current, shared with the legacy discharge/
 *    formation test path.
 *  - <8V panel voltage -> FETs off, state saved, deep sleep with
 *    periodic wake to re-check.
 *
 * HARDWARE ASSUMPTIONS (verify against your actual board):
 *   - ESP32 powered from the BATTERY side (own small buck/LDO), not
 *     the panel, so it survives a sub-8V panel long enough to save
 *     state, sleep, and later resume.
 *   - INA219 in series with the battery: getBusVoltage_V() = battery
 *     voltage, getCurrent_mA() > 0 when current flows INTO the battery.
 *     Flip CURRENT_SIGN if your wiring gives the opposite sign.
 *   - PWM_CHARGE_PIN drives a buck converter, panel -> battery.
 *   - PWM_DISCHARGE_PIN drives a sink FET for the legacy manual
 *     formation/discharge test only.
 *   - VIN_PIN is a panel voltage divider on an ADC1 input-only pin.
 *   - CHARGE_V_OVERVOLTAGE (15.0V) and BULK_TARGET_V (13.8V) assume a
 *     12V lead-acid-style pack -- VERIFY for your actual chemistry.
 *   - No temperature sensing -- a real gap beyond a bench test.
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <PID_v1.h>
#include <math.h>

// ================= Pin Assignments =================
const int PWM_CHARGE_PIN    = 25;
const int BUTTON_PIN        = 27;
const int VIN_PIN           = 34;

// ================= LEDC (PWM) config =================
const int CHARGE_CH    = 0;
const int PWM_FREQ_HZ  = 20000;
const int PWM_RES_BITS = 8;
const int PWM_MAX      = 255;

// ================= Calibration =================
const float VIN_DIVIDER_RATIO = 6.0f;
const float ADC_VREF          = 3.3f;
const int   ADC_MAX_COUNTS    = 4095;
const int   ADC_OVERSAMPLE_N  = 16;
const float CURRENT_SIGN      = 1.0f;

Adafruit_INA219 ina219;

//float readVinPanel() {
//  long sum = 0;
//  for (int i = 0; i < ADC_OVERSAMPLE_N; i++) sum += analogRead(VIN_PIN);
//  float avgCounts = (float)sum / ADC_OVERSAMPLE_N;
//  return (avgCounts * (ADC_VREF / ADC_MAX_COUNTS)) * VIN_DIVIDER_RATIO;
//}

// ================= Safety / charge parameters (12V lead-acid style -- VERIFY) =================
const float BULK_TARGET_V         = 13.8f;
const float CHARGE_V_OVERVOLTAGE  = 15.0f;
const float OVERVOLTAGE_PULSE_MARGIN = 0.10f; // force-end a pulse this far below the hard ceiling
const float CHARGE_TERMINATION_MA = 25.0f;
const unsigned long CHARGE_TIMEOUT_MS = (unsigned long)6 * 60UL * 60UL * 1000UL;

const float OVERCURRENT_LIMIT_MA = 1500.0f;
const float slopeThreshold = 0.0005f; // used only by the legacy discharge/formation test

// ================= Panel undervoltage shutdown =================
const float VIN_SHUTDOWN_THRESHOLD = 8.0f;
const float VIN_RESUME_THRESHOLD   = 9.0f;
const int   UVLO_DEBOUNCE_SAMPLES  = 5;
const uint64_t SLEEP_CHECK_INTERVAL_US = 5ULL * 60ULL * 1000000ULL;

// ================= Parasitic baseline (voltage-hold, not FET-off) =================
const float PARASITIC_PROBE_OFFSET_V = 0.10f;
const float PARASITIC_HOLD_KP = 40.0f;
const float PARASITIC_SETTLE_S = 20.0f;
const float PARASITIC_SAMPLE_S = 10.0f;

// ================= Outgassing bisection search =================
const float MIN_PULSE_START_MA = 20.0f;         // absolute floor for the search's lower bound
const float PARASITIC_START_MULTIPLIER = 1.5f;  // lower bound = max(floor, parasitic*this)
const int   MAX_BISECTION_ITERS = 8;
const float BISECTION_RESOLUTION_MA = 15.0f;

// ================= Per-pulse timing/physics (all tau-derived, no fixed durations) =================
const unsigned long IR_SETTLE_MS = 200;          // let the instantaneous IR step settle (ADC/noise)
const unsigned long SLOPE_FIT_WINDOW_MS = 2000;  // fixed short window for the initial R,C,tau estimate
const float TAU_MULTIPLE_FOR_PULSE = 3.0f;       // "no knee yet" pulse timeout, in units of tau
const float TAU_MULTIPLE_POST_KNEE = 1.5f;       // extra run-on after a knee, to nail slope2
const unsigned long MIN_PULSE_MS = 2000, MAX_PULSE_MS = 60000;
const unsigned long MIN_REST_MS  = 3000, MAX_REST_MS  = 120000;

const float OUTGAS_EFFICIENCY_THRESHOLD = 0.7f;  // observedSlope/expectedSlope below this = outgassing
const int   KNEE_CONFIRM_TICKS = 15;             // ~1.5s sustained below threshold, at 100ms ticks
const float EMA_ALPHA = 0.1f;                    // slope smoothing, ~1s time constant at 100ms ticks

// ================= Persisted state (survives deep sleep & power loss) =================
enum Mode {
  MODE_BOOT,
  MODE_IDLE,
  MODE_CHAR_PARASITIC,
  MODE_PASSIVE_FORMATION_TEST,
  MODE_WAIT_AFTER_OUTGASSING,
  MODE_CHARGE_BULK,
  MODE_OUTGAS_PULSE_TEST,
  MODE_CHARGE_FLOAT,
  MODE_CHARGE_DONE,
  MODE_FAULT,
  MODE_LOW_INPUT_SLEEP
};

enum PostParasiticAction { POST_PARASITIC_IDLE, POST_PARASITIC_START_PULSE_TEST };

enum PulseSubPhase {
  PULSE_PRE,
  PULSE_IR_WAIT,
  PULSE_PROBE,
  PULSE_MAIN,
  PULSE_REST_EARLY,
  PULSE_REST_LATE
};

struct PersistedState {
  uint32_t magic;
  Mode mode;
  float setCurrent;
  float parasiticCurrent_mA;
  float outgassingCurrent_mA;
  float outgassingVoltage_V;
  bool calibrated;
  Mode modeBeforeSleep;
};
const uint32_t STATE_MAGIC = 0xB19E5014;

Preferences prefs;
PersistedState state;

void saveState() {
  state.magic = STATE_MAGIC;
  prefs.begin("solarchg", false);
  prefs.putBytes("state", &state, sizeof(state));
  prefs.end();
}

bool loadState() {
  prefs.begin("solarchg", true);
  bool ok = false;
  if (prefs.isKey("state")) {
    PersistedState tmp;
    size_t n = prefs.getBytes("state", &tmp, sizeof(tmp));
    if (n == sizeof(tmp) && tmp.magic == STATE_MAGIC) {
      state = tmp;
      ok = true;
    }
  }
  prefs.end();
  return ok;
}

// ================= Runtime mode =================
Mode mode = MODE_BOOT;
PostParasiticAction postParasiticAction = POST_PARASITIC_IDLE;
unsigned long previousMillis = 0;
const unsigned long CONTROL_INTERVAL_MS = 100;
unsigned long stateEnteredAt = 0;
unsigned long chargeStartedAt = 0;
int uvloLowCount = 0;

// ---- Parasitic-baseline hold-loop state ----
float parasiticTargetVoltage = 0;
float parasiticHoldDuty = 0;
float parasiticAccum = 0;
int parasiticSamples = 0;

// ---- Bisection search state ----
float bisectLow = 0, bisectHigh = 0;
bool haveUpperBound = false;
int bisectionIterations = 0;
bool doingFinalConfirm = false;

// ---- Per-pulse state ----
bool pulseOpenLoopMaxDuty = false;
float pulseTestCurrent = 0;
float C_baseline = -1;
PulseSubPhase pulseSubPhase = PULSE_PRE;
unsigned long pulsePhaseStartedAt = 0;
unsigned long pulsePostKneeStartedAt = 0;
float V_beforePulse = 0, V_afterIR = 0, V_afterProbe = 0;
float R_est = 0, C_est = 0, tau_est = 0;
float slope1 = 0, slope2 = 0;
float emaSlope = 0, V_prevTick = 0;
bool kneeDetected = false;
int kneeConfirmTicks = 0;
float kneeVoltage = 0, kneeCurrentMa = 0;
bool pulseFoundKnee = false;
double E_cap_J = 0, E_gas_J = 0;

// ---- Decay-fit state ----
float decaySamples[5];
unsigned long decaySampleTimesMs[5];
int decaySampleIdx = 0;
int decaySampleCount = 0;
float tau_decay = -1;         // single-exponential case (no knee)
float tau_fast_decay = -1, tau_slow_decay = -1; // bi-exponential case (knee)
unsigned long restRemainingMs = 0;

float cvTargetVoltage = 0;

// ================= Kalman filter (legacy discharge/formation test only) =================
float voltage_est = 0.0f;
float slope_est   = 0.0f;
float internalResistance = 0.05f;

float P[2][2] = { {1, 0}, {0, 1} };
const float Qn[2][2] = { {0.0005f, 0}, {0, 0.0005f} };
const float R_meas = 0.02f;

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
  float current_input = current_input_ma / 1000.0f;

  float x_pred[2];
  x_pred[0] = A[0][0] * voltage_est + A[0][1] * slope_est + Bc[0] * current_input;
  x_pred[1] = A[1][0] * voltage_est + A[1][1] * slope_est + Bc[1] * current_input;

  float AP[2][2];
  AP[0][0] = A[0][0]*P[0][0] + A[0][1]*P[1][0];
  AP[0][1] = A[0][0]*P[0][1] + A[0][1]*P[1][1];
  AP[1][0] = A[1][0]*P[0][0] + A[1][1]*P[1][0];
  AP[1][1] = A[1][0]*P[0][1] + A[1][1]*P[1][1];

  float P_pred[2][2];
  P_pred[0][0] = AP[0][0]*A[0][0] + AP[0][1]*A[0][1] + Qn[0][0];
  P_pred[0][1] = AP[0][0]*A[1][0] + AP[0][1]*A[1][1] + Qn[0][1];
  P_pred[1][0] = AP[1][0]*A[0][0] + AP[1][1]*A[0][1] + Qn[1][0];
  P_pred[1][1] = AP[1][0]*A[1][0] + AP[1][1]*A[1][1] + Qn[1][1];

  float y = measured_voltage - (H[0]*x_pred[0] + H[1]*x_pred[1]);
  float S = H[0]*P_pred[0][0] + H[1]*P_pred[1][0] + R_meas;
  float K[2];
  K[0] = (P_pred[0][0]*H[0] + P_pred[0][1]*H[1]) / S;
  K[1] = (P_pred[1][0]*H[0] + P_pred[1][1]*H[1]) / S;

  voltage_est = x_pred[0] + K[0]*y;
  slope_est   = x_pred[1] + K[1]*y;

  float KH00 = K[0]*H[0], KH01 = K[0]*H[1];
  float KH10 = K[1]*H[0], KH11 = K[1]*H[1];
  float newP00 = (1-KH00)*P_pred[0][0] - KH01*P_pred[1][0];
  float newP01 = (1-KH00)*P_pred[0][1] - KH01*P_pred[1][1];
  float newP10 = -KH10*P_pred[0][0] + (1-KH11)*P_pred[1][0];
  float newP11 = -KH10*P_pred[0][1] + (1-KH11)*P_pred[1][1];
  P[0][0]=newP00; P[0][1]=newP01; P[1][0]=newP10; P[1][1]=newP11;
}

// ================= PID controllers =================
double pidIn_charge, pidOut_charge, pidSet_charge;
PID pidCharge(&pidIn_charge, &pidOut_charge, &pidSet_charge, 2.0, 5.0, 0.5, DIRECT);

// ================= Fractional-Voc MPPT (no Iin needed) =================
const float VOC_FRACTION = 0.80f;
const unsigned long VOC_RESAMPLE_INTERVAL_MS = 10000;
const unsigned long VOC_SETTLE_MS = 40;
const float MPPT_KP = 6.0f;

float vinTarget = 0;
float mpptDutyCeiling = 0;
unsigned long lastVocSample = 0;
uint32_t currentChargeDuty = 0;

void sampleVocAndUpdateTarget() {
  uint32_t dutyBeforeSample = currentChargeDuty;
  ledcWrite(CHARGE_CH, 0);
  delay(VOC_SETTLE_MS);
  float voc = readVinPanel();
  vinTarget = VOC_FRACTION * voc;
  ledcWrite(CHARGE_CH, dutyBeforeSample);
  lastVocSample = millis();
  Serial.print("MPPT: sampled Voc="); Serial.print(voc, 2);
  Serial.print("V, target Vmp="); Serial.println(vinTarget, 2);
}

void mpptVoltageLoop(float vinLoaded) {
  if (millis() - lastVocSample >= VOC_RESAMPLE_INTERVAL_MS) {
    sampleVocAndUpdateTarget();
  }
  float error = vinLoaded - vinTarget;
  mpptDutyCeiling += MPPT_KP * error * (CONTROL_INTERVAL_MS / 1000.0f);
  mpptDutyCeiling = constrain(mpptDutyCeiling, 0.0f, (float)PWM_MAX);
}

// ================= Mode transitions =================
void enterMode(Mode m) {
  mode = m;
  state.mode = m;
  stateEnteredAt = millis();
  Serial.print("-> Mode: "); Serial.println((int)m);
}

void allOff() {
  ledcWrite(CHARGE_CH, 0);
  currentChargeDuty = 0;
}

void tripFault(const char* reason) {
  allOff();
  Serial.print("FAULT: "); Serial.println(reason);
  enterMode(MODE_FAULT);
  saveState();
}

void startParasiticCharacterization() {
  ledcWrite(CHARGE_CH, 0);
  float idleV = ina219.getBusVoltage_V();
  parasiticTargetVoltage = idleV + PARASITIC_PROBE_OFFSET_V;
  parasiticHoldDuty = 0;
  parasiticAccum = 0;
  parasiticSamples = 0;
  enterMode(MODE_CHAR_PARASITIC);
  Serial.print("Parasitic baseline: idle V="); Serial.print(idleV,3);
  Serial.print(", holding at "); Serial.println(parasiticTargetVoltage,3);
}

void startPassiveFormationTest() {
  ledcWrite(CHARGE_CH, 0);
  enterMode(MODE_PASSIVE_FORMATION_TEST);
  Serial.println("Starting passive formation test (observing discharge via parasitic load).");
}

void startCharge() {
  mpptDutyCeiling = 40;
  lastVocSample = 0;
  chargeStartedAt = millis();
  enterMode(MODE_CHARGE_BULK);
}

// Applies the pulse's charge current, using whichever mode this pulse is in.
void applyPulseDuty(float iBatt) {
  if (pulseOpenLoopMaxDuty) {
    ledcWrite(CHARGE_CH, (int)mpptDutyCeiling);
  } else {
    pidIn_charge = iBatt;
    pidCharge.Compute();
    float duty = min((float)pidOut_charge, mpptDutyCeiling);
    ledcWrite(CHARGE_CH, (int)duty);
  }
}

void beginPulse() {
  pulseSubPhase = PULSE_PRE;
  pulsePhaseStartedAt = millis();
  kneeDetected = false;
  kneeConfirmTicks = 0;
  slope1 = 0;
  E_cap_J = 0;
  E_gas_J = 0;
  pulseFoundKnee = false;
}

void beginBisectionPulse() {
  pulseOpenLoopMaxDuty = false;
  pidSet_charge = pulseTestCurrent;
  pidCharge.SetMode(AUTOMATIC);
  beginPulse();
}

void startOutgasPulseTest() {
  bisectLow = max(MIN_PULSE_START_MA, state.parasiticCurrent_mA * PARASITIC_START_MULTIPLIER);
  bisectHigh = 0;
  haveUpperBound = false;
  bisectionIterations = 0;
  doingFinalConfirm = false;
  C_baseline = -1;
  pulseOpenLoopMaxDuty = true;
  beginPulse();
  enterMode(MODE_OUTGAS_PULSE_TEST);
  Serial.println("Starting outgassing characterization: maximum-available-current pulse.");
}

void startFloatHold() {
  cvTargetVoltage = state.outgassingVoltage_V;
  pidOut_charge = 0;
  enterMode(MODE_CHARGE_FLOAT);
}

void goToLowInputSleep() {
  Serial.println("Panel voltage low -- safe shutdown, saving state, deep sleep.");
  allOff();
  state.modeBeforeSleep = mode;
  state.mode = MODE_LOW_INPUT_SLEEP;
  saveState();
  esp_sleep_enable_timer_wakeup(SLEEP_CHECK_INTERVAL_US);
  esp_deep_sleep_start();
}

// Two-exponential "peeling" decay fit, used only after a confirmed knee.
// samples[0..4] at timesMs[0..4]; [2],[3],[4] equally spaced, used first to
// fit the slow tail; that prediction is then subtracted from the early
// points [0],[1] to isolate and fit the fast component. Requires
// tau_slow >> tau_fast for a clean separation -- see file header caveats.
void fitBiExponentialDecay(float* samples, unsigned long* timesMs, float &tauFastOut, float &tauSlowOut) {
  tauFastOut = -1; tauSlowOut = -1;
  float dtLate = (timesMs[3]-timesMs[2]) / 1000.0f;
  float d1 = samples[2]-samples[3];
  float d2 = samples[3]-samples[4];
  if (!(d1 > 1e-6f && d2 > 1e-6f && d2 < d1)) return;
  float tauSlow = -dtLate / logf(d2/d1);
  tauSlowOut = tauSlow;

  float t2 = timesMs[2]/1000.0f, t3 = timesMs[3]/1000.0f;
  float denom = 1.0f - expf(-(t3-t2)/tauSlow);
  if (denom < 1e-9f) return;
  float AslowExpT2 = (samples[2]-samples[3]) / denom;
  float Vinf = samples[2] - AslowExpT2;

  float t0 = timesMs[0]/1000.0f, t1 = timesMs[1]/1000.0f;
  float slowAt0 = Vinf + AslowExpT2 * expf(-(t0-t2)/tauSlow);
  float slowAt1 = Vinf + AslowExpT2 * expf(-(t1-t2)/tauSlow);
  float resid0 = samples[0]-slowAt0;
  float resid1 = samples[1]-slowAt1;
  float dtEarly = t1-t0;
  if (resid0 > 1e-6f && resid1 > 1e-6f && resid1 < resid0) {
    tauFastOut = -dtEarly / logf(resid1/resid0);
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  ledcSetup(CHARGE_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PWM_CHARGE_PIN, CHARGE_CH);
  allOff();

  Wire.begin();
  if (!ina219.begin()) {
    Serial.println("INA219 not found -- check wiring/address.");
  }
  ina219.setCalibration_32V1A();

  kalmanSetTimestep(CONTROL_INTERVAL_MS / 1000.0f);

  bool haveState = loadState();
  float vinNow = readVinPanel();

  if (haveState && state.mode == MODE_LOW_INPUT_SLEEP) {
    if (vinNow < VIN_RESUME_THRESHOLD) {
      Serial.print("Still low (Vin="); Serial.print(vinNow, 2);
      Serial.println("V) -- back to sleep.");
      esp_sleep_enable_timer_wakeup(SLEEP_CHECK_INTERVAL_US);
      esp_deep_sleep_start();
    }
    Serial.println("Panel recovered.");
    voltage_est = ina219.getBusVoltage_V();
    slope_est = 0;
    if (state.modeBeforeSleep == MODE_CHARGE_BULK || state.modeBeforeSleep == MODE_CHAR_PARASITIC ||
        state.modeBeforeSleep == MODE_OUTGAS_PULSE_TEST || state.modeBeforeSleep == MODE_CHARGE_FLOAT) {
      Serial.println("Restarting charge sequence from bulk.");
      startCharge();
    } else {
      enterMode(state.modeBeforeSleep);
    }
  } else if (haveState && state.calibrated) {
    voltage_est = ina219.getBusVoltage_V();
    enterMode(MODE_IDLE);
    Serial.println("Loaded prior calibration:");
    Serial.print("  parasitic="); Serial.print(state.parasiticCurrent_mA); Serial.println(" mA");
    Serial.print("  outgas I="); Serial.print(state.outgassingCurrent_mA);
    Serial.print(" mA at V="); Serial.println(state.outgassingVoltage_V, 3);
  } else {
    state = PersistedState{};
    voltage_est = ina219.getBusVoltage_V();
    Serial.println("No prior calibration found.");
    postParasiticAction = POST_PARASITIC_IDLE;
    startParasiticCharacterization();
  }

  pidCharge.SetOutputLimits(0, PWM_MAX);
  pidCharge.SetSampleTime(CONTROL_INTERVAL_MS);

  Serial.println("Ready. Serial: 'D'=legacy formation test, "
                  "'C'=start charge (bulk -> parasitic -> pulse test -> float), "
                  "'A'=re-run parasitic characterization only.");
}

// ================= Main loop =================
void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    char c = Serial.read();
    if ((c=='D'||c=='d') && mode==MODE_IDLE) startPassiveFormationTest();
    if ((c=='C'||c=='c') && mode==MODE_IDLE) startCharge();
    if ((c=='A'||c=='a') && mode==MODE_IDLE) { postParasiticAction = POST_PARASITIC_IDLE; startParasiticCharacterization(); }
  }
  if (digitalRead(BUTTON_PIN) == LOW && mode == MODE_IDLE) {
    startPassiveFormationTest();
  }

  if (currentMillis - previousMillis < CONTROL_INTERVAL_MS) return;
  float dt = (currentMillis - previousMillis) / 1000.0f;
  previousMillis = currentMillis;
  kalmanSetTimestep(dt);

  float vBatt = ina219.getBusVoltage_V();
  float vIn   = readVinPanel();

  bool activeModeUsesInput = (mode == MODE_CHARGE_BULK || mode == MODE_OUTGAS_PULSE_TEST
                               || mode == MODE_CHARGE_FLOAT || mode == MODE_CHAR_PARASITIC
                               || mode == MODE_PASSIVE_FORMATION_TEST || mode == MODE_WAIT_AFTER_OUTGASSING
                               || mode == MODE_IDLE);
  if (activeModeUsesInput) {
    if (vIn < VIN_SHUTDOWN_THRESHOLD) uvloLowCount++; else uvloLowCount = 0;
    if (uvloLowCount >= UVLO_DEBOUNCE_SAMPLES) {
      goToLowInputSleep();
    }
  }

  if (vBatt > CHARGE_V_OVERVOLTAGE) { tripFault("battery overvoltage"); return; }

  switch (mode) {

    case MODE_IDLE:
    case MODE_FAULT:
      break;

    case MODE_CHAR_PARASITIC: {
      float iBatt = ina219.getCurrent_mA() * CURRENT_SIGN;
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("parasitic-probe overcurrent"); break; }

      float vError = parasiticTargetVoltage - vBatt;
      parasiticHoldDuty += vError * PARASITIC_HOLD_KP * (CONTROL_INTERVAL_MS/1000.0f);
      parasiticHoldDuty = constrain(parasiticHoldDuty, 0.0f, (float)PWM_MAX);
      ledcWrite(CHARGE_CH, (int)parasiticHoldDuty);

      bool settled = (currentMillis - stateEnteredAt) > (unsigned long)(PARASITIC_SETTLE_S * 1000);
      if (settled) {
        parasiticAccum += iBatt;
        parasiticSamples++;
        if ((currentMillis - stateEnteredAt) > (unsigned long)((PARASITIC_SETTLE_S + PARASITIC_SAMPLE_S) * 1000)) {
          state.parasiticCurrent_mA = parasiticAccum / max(1, parasiticSamples);
          ledcWrite(CHARGE_CH, 0);
          saveState();
          Serial.print("Parasitic/self-discharge-compensation current: ");
          Serial.print(state.parasiticCurrent_mA, 2);
          Serial.println(" mA");
          if (postParasiticAction == POST_PARASITIC_START_PULSE_TEST) {
            startOutgasPulseTest();
          } else {
            enterMode(MODE_IDLE);
          }
        }
      }
      break;
    }

    case MODE_PASSIVE_FORMATION_TEST: {
      float iBatt = ina219.getCurrent_mA() * CURRENT_SIGN; // Expected to be negative (discharge)

      kalmanUpdate(iBatt, vBatt);

      if (fabs(slope_est) < slopeThreshold && (currentMillis - stateEnteredAt) > 10000) {
        Serial.print("Passive discharge plateau detected at voltage: "); Serial.println(voltage_est, 4);
        Serial.print("Current draw: "); Serial.print(-iBatt, 2); Serial.println(" mA");
        enterMode(MODE_WAIT_AFTER_OUTGASSING);
      }
      break;
    }

    case MODE_WAIT_AFTER_OUTGASSING:
      if (currentMillis - stateEnteredAt >= 5UL * 60UL * 1000UL) {
        Serial.println("Passive formation test snapshot complete.");
        enterMode(MODE_IDLE);
      }
      break;

    case MODE_CHARGE_BULK: {
      if (currentMillis - chargeStartedAt > CHARGE_TIMEOUT_MS) { tripFault("charge timeout"); break; }

      float iBatt = ina219.getCurrent_mA() * CURRENT_SIGN;
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("charge overcurrent"); break; }

      mpptVoltageLoop(vIn);
      currentChargeDuty = (uint32_t)mpptDutyCeiling;
      ledcWrite(CHARGE_CH, currentChargeDuty);

      if (vBatt >= BULK_TARGET_V) {
        Serial.println("Bulk charge plateau reached -- measuring parasitic baseline, then pulse-testing for outgassing.");
        ledcWrite(CHARGE_CH, 0);
        postParasiticAction = POST_PARASITIC_START_PULSE_TEST;
        startParasiticCharacterization();
        break;
      }

      Serial.print("Bulk Vbatt="); Serial.print(vBatt,3);
      Serial.print(" Ibatt="); Serial.print(iBatt);
      Serial.print(" Vin="); Serial.print(vIn,2);
      Serial.print(" duty="); Serial.println(currentChargeDuty);
      break;
    }

    case MODE_OUTGAS_PULSE_TEST: {
      float iBatt = ina219.getCurrent_mA() * CURRENT_SIGN;
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("pulse test overcurrent"); break; }

      switch (pulseSubPhase) {

        case PULSE_PRE: {
          ledcWrite(CHARGE_CH, 0);
          mpptVoltageLoop(vIn); // refresh MPPT target here, between pulses -- not mid-pulse
          V_beforePulse = vBatt;
          if (!pulseOpenLoopMaxDuty) pidCharge.SetMode(AUTOMATIC);
          pulseSubPhase = PULSE_IR_WAIT;
          pulsePhaseStartedAt = currentMillis;
          break;
        }

        case PULSE_IR_WAIT: {
          applyPulseDuty(iBatt);
          if (currentMillis - pulsePhaseStartedAt >= IR_SETTLE_MS) {
            V_afterIR = vBatt;
            R_est = (iBatt > 1.0f) ? (V_afterIR - V_beforePulse) / (iBatt/1000.0f) : 0;
            pulseSubPhase = PULSE_PROBE;
            pulsePhaseStartedAt = currentMillis;
          }
          break;
        }

        case PULSE_PROBE: {
          applyPulseDuty(iBatt);
          if (currentMillis - pulsePhaseStartedAt >= SLOPE_FIT_WINDOW_MS) {
            V_afterProbe = vBatt;
            float probeSlope = (V_afterProbe - V_afterIR) / (SLOPE_FIT_WINDOW_MS/1000.0f);
            float iAmps = iBatt/1000.0f;
            C_est = (probeSlope > 1e-6f) ? (iAmps/probeSlope) : -1;
            tau_est = (C_est > 0) ? R_est*C_est : -1;
            if (C_baseline < 0 && C_est > 0) {
              C_baseline = C_est;
              Serial.print("Baseline capacitance established: "); Serial.print(C_baseline,2); Serial.println(" F");
            }
            slope1 = probeSlope;
            emaSlope = probeSlope;
            V_prevTick = V_afterProbe;
            Serial.print("Pulse probe: I="); Serial.print(iBatt,1);
            Serial.print("mA R="); Serial.print(R_est,4);
            Serial.print(" ohm tau="); Serial.print(tau_est,2); Serial.println(" s");
            pulseSubPhase = PULSE_MAIN;
            pulsePhaseStartedAt = currentMillis;
          }
          break;
        }

        case PULSE_MAIN: {
          applyPulseDuty(iBatt);

          float instSlope = (vBatt - V_prevTick) / (CONTROL_INTERVAL_MS/1000.0f);
          emaSlope = EMA_ALPHA*instSlope + (1-EMA_ALPHA)*emaSlope;
          V_prevTick = vBatt;

          float iAmps = iBatt/1000.0f;
          float I_cap = (C_baseline > 0) ? constrain(C_baseline*instSlope, 0.0f, iAmps) : iAmps;
          float I_gas = max(0.0f, iAmps - I_cap);
          E_cap_J += vBatt * I_cap * (CONTROL_INTERVAL_MS/1000.0);
          E_gas_J += vBatt * I_gas * (CONTROL_INTERVAL_MS/1000.0);

          unsigned long elapsed = currentMillis - pulsePhaseStartedAt;

          if (!kneeDetected) {
            float ratio = (slope1 > 1e-9f) ? emaSlope/slope1 : 1.0f;
            if (ratio < OUTGAS_EFFICIENCY_THRESHOLD || vBatt >= CHARGE_V_OVERVOLTAGE - OVERVOLTAGE_PULSE_MARGIN) {
              kneeConfirmTicks++;
              if (kneeConfirmTicks >= KNEE_CONFIRM_TICKS) {
                kneeDetected = true;
                kneeVoltage = vBatt;
                kneeCurrentMa = iBatt;
                pulsePostKneeStartedAt = currentMillis;
                Serial.print("Knee detected: V="); Serial.print(kneeVoltage,4);
                Serial.print(" I="); Serial.print(kneeCurrentMa,1);
                Serial.print(" efficiency="); Serial.println(ratio,3);
              }
            } else {
              kneeConfirmTicks = 0;
            }
            unsigned long noKneeTimeout = constrain(
              (tau_est>0) ? (unsigned long)(TAU_MULTIPLE_FOR_PULSE*tau_est*1000.0f) : MIN_PULSE_MS,
              MIN_PULSE_MS, MAX_PULSE_MS);
            if (!kneeDetected && elapsed >= noKneeTimeout) {
              slope2 = emaSlope;
              pulseFoundKnee = false;
              goto pulseMainDone;
            }
          } else {
            unsigned long postKneeTarget = constrain(
              (tau_est>0) ? (unsigned long)(TAU_MULTIPLE_POST_KNEE*tau_est*1000.0f) : MIN_PULSE_MS,
              MIN_PULSE_MS/2, MAX_PULSE_MS/2);
            if (currentMillis - pulsePostKneeStartedAt >= postKneeTarget) {
              slope2 = emaSlope;
              pulseFoundKnee = true;
              goto pulseMainDone;
            }
          }
          break;

          pulseMainDone:
          Serial.print("Pulse energy: E_cap="); Serial.print(E_cap_J,4);
          Serial.print("J E_gas="); Serial.print(E_gas_J,4);
          Serial.print("J (gas fraction=");
          Serial.print((E_cap_J+E_gas_J>1e-9)? (E_gas_J/(E_cap_J+E_gas_J))*100.0 : 0.0, 1);
          Serial.println("%)");

          ledcWrite(CHARGE_CH, 0);
          decaySampleIdx = 0;
          float tauRef = (tau_est>0) ? tau_est : 2.0f;
          if (pulseFoundKnee) {
            float dtE = max(0.3f, tauRef*0.5f);
            float dtL = max(1.0f, tauRef*2.0f);
            decaySampleTimesMs[0] = 0;
            decaySampleTimesMs[1] = (unsigned long)(dtE*1000);
            decaySampleTimesMs[2] = (unsigned long)(4.0f*tauRef*1000);
            decaySampleTimesMs[3] = decaySampleTimesMs[2] + (unsigned long)(dtL*1000);
            decaySampleTimesMs[4] = decaySampleTimesMs[3] + (unsigned long)(dtL*1000);
            decaySampleCount = 5;
          } else {
            unsigned long dtS = constrain((unsigned long)(tauRef*500.0f), 500UL, 20000UL);
            decaySampleTimesMs[0] = 0; decaySampleTimesMs[1] = dtS; decaySampleTimesMs[2] = 2*dtS;
            decaySampleCount = 3;
          }
          pulseSubPhase = PULSE_REST_EARLY;
          pulsePhaseStartedAt = currentMillis;
          break;
        }

        case PULSE_REST_EARLY: {
          ledcWrite(CHARGE_CH, 0);
          unsigned long elapsed = currentMillis - pulsePhaseStartedAt;
          if (decaySampleIdx < decaySampleCount && elapsed >= decaySampleTimesMs[decaySampleIdx]) {
            decaySamples[decaySampleIdx] = vBatt;
            decaySampleIdx++;
          }
          if (decaySampleIdx >= decaySampleCount) {
            float tauForRest = -1;
            if (decaySampleCount == 3) {
              float d1 = decaySamples[0]-decaySamples[1];
              float d2 = decaySamples[1]-decaySamples[2];
              float dtSec = decaySampleTimesMs[1]/1000.0f;
              tau_decay = (d1>1e-5f && d2>1e-5f && d2<d1) ? -dtSec/logf(d2/d1) : -1;
              if (tau_decay > 0 && tau_est > 0) {
                Serial.print("tau_decay="); Serial.print(tau_decay,2);
                Serial.print("s vs tau_charge="); Serial.print(tau_est,2); Serial.println("s");
              }
              tauForRest = (tau_decay>0) ? tau_decay : tau_est;
            } else {
              fitBiExponentialDecay(decaySamples, decaySampleTimesMs, tau_fast_decay, tau_slow_decay);
              Serial.print("Decay fit (post-knee): tau_fast="); Serial.print(tau_fast_decay,2);
              Serial.print("s tau_slow="); Serial.print(tau_slow_decay,2); Serial.println("s");
              if (tau_fast_decay > 0 && tau_est > 0) {
                float apparentCratio = tau_fast_decay / tau_est; // ~R fixed, so tau ratio ~= C ratio
                Serial.print("Charge/decay capacitance asymmetry ratio: "); Serial.println(apparentCratio,2);
                if (apparentCratio > 2.0f || apparentCratio < 0.5f) {
                  Serial.println("  note: charge-side and decay-side time constants disagree substantially -- "
                                  "consistent with a genuine Faradaic (gassing) process, not just noise.");
                }
              }
              tauForRest = (tau_slow_decay>0) ? tau_slow_decay : ((tau_fast_decay>0)?tau_fast_decay:tau_est);
            }
            unsigned long fullRestMs = constrain(
              (tauForRest>0) ? (unsigned long)(3.0f*tauForRest*1000.0f) : MIN_REST_MS,
              MIN_REST_MS, MAX_REST_MS);
            unsigned long already = decaySampleTimesMs[decaySampleCount-1];
            restRemainingMs = (fullRestMs > already) ? (fullRestMs - already) : 0;
            pulseSubPhase = PULSE_REST_LATE;
            pulsePhaseStartedAt = currentMillis;
          }
          break;
        }

        case PULSE_REST_LATE: {
          ledcWrite(CHARGE_CH, 0);
          if (currentMillis - pulsePhaseStartedAt >= restRemainingMs) {

            if (pulseOpenLoopMaxDuty) {
              if (pulseFoundKnee) {
                bisectHigh = kneeCurrentMa;
                haveUpperBound = true;
                state.outgassingCurrent_mA = kneeCurrentMa;
                state.outgassingVoltage_V = kneeVoltage;
                Serial.println("Initial high-current pulse found outgassing -- starting bisection search for the minimum current.");
                pulseTestCurrent = (bisectLow+bisectHigh)/2.0f;
                beginBisectionPulse();
              } else {
                if (vBatt >= CHARGE_V_OVERVOLTAGE - OVERVOLTAGE_PULSE_MARGIN) {
                  tripFault("reached overvoltage margin without ever finding an outgassing plateau -- check cell/wiring");
                  break;
                }
                Serial.println("No knee yet at max available current -- battery not yet at gassing voltage; continuing.");
                beginPulse(); // stays open-loop max-duty
              }
            } else if (doingFinalConfirm) {
              if (!pulseFoundKnee) {
                Serial.println("Final confirmation pulse did not reproduce the knee -- borderline/noisy result, "
                                "keeping the last confirmed value but treat it with some caution.");
              }
              state.calibrated = true;
              saveState();
              Serial.print("Outgassing result: minimum current~"); Serial.print(state.outgassingCurrent_mA);
              Serial.print(" mA at V="); Serial.println(state.outgassingVoltage_V,4);
              startFloatHold();
            } else {
              bisectionIterations++;
              if (pulseFoundKnee) {
                bisectHigh = pulseTestCurrent;
                state.outgassingCurrent_mA = pulseTestCurrent;
                state.outgassingVoltage_V = kneeVoltage;
              } else {
                bisectLow = pulseTestCurrent;
              }
              if ((bisectHigh-bisectLow) <= BISECTION_RESOLUTION_MA || bisectionIterations >= MAX_BISECTION_ITERS) {
                doingFinalConfirm = true;
                pulseTestCurrent = bisectHigh;
                Serial.println("Bisection converged -- running one confirmation pulse before finalizing.");
                beginBisectionPulse();
              } else {
                pulseTestCurrent = (bisectLow+bisectHigh)/2.0f;
                beginBisectionPulse();
              }
            }
          }
          break;
        }
      }
      break;
    }

    case MODE_CHARGE_FLOAT: {
      if (currentMillis - chargeStartedAt > CHARGE_TIMEOUT_MS) { tripFault("charge timeout"); break; }

      float iBatt = ina219.getCurrent_mA() * CURRENT_SIGN;
      if (iBatt > OVERCURRENT_LIMIT_MA) { tripFault("charge overcurrent"); break; }

      mpptVoltageLoop(vIn);

      float vError = cvTargetVoltage - vBatt;
      float duty = pidOut_charge + vError * 40.0f;
      duty = constrain(duty, 0.0f, mpptDutyCeiling);
      pidOut_charge = duty;
      currentChargeDuty = (uint32_t)duty;
      ledcWrite(CHARGE_CH, currentChargeDuty);

      if (iBatt < CHARGE_TERMINATION_MA) {
        ledcWrite(CHARGE_CH, 0);
        Serial.println("Charge complete (float current tapered to termination threshold).");
        enterMode(MODE_CHARGE_DONE);
      }

      Serial.print("Float Vbatt="); Serial.print(vBatt,3);
      Serial.print(" Ibatt="); Serial.print(iBatt);
      Serial.print(" target="); Serial.println(cvTargetVoltage,3);
      break;
    }

    case MODE_CHARGE_DONE:
      break;

    case MODE_BOOT:
    case MODE_LOW_INPUT_SLEEP:
      break;
  }
}
