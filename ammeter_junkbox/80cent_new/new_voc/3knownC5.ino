#include "AVR_PWM.h"
#include <Arduino.h> // Required for millis(), analogRead(), Serial, etc.
#include <math.h>    // Required for exp(), pow(), fabs()
#include <float.h>   // Required for DBL_MAX for error initialization

    // Pin and voltage definitions
    static const uint8_t PWM_PIN = 9;
    static const uint8_t LOAD_PIN = A1;
    static const uint8_t SOURCE_PIN = A0;
    static const uint8_t TEMP_PIN = A2;  // Added temperature sensing

    // Voltage ranges and references
    static constexpr float LOAD_VOLTAGE_RANGE = 30.0;
    static constexpr float SOURCE_VOLTAGE_RANGE = 28.0;
    static constexpr float TEMP_REFERENCE = 25.0;  // Reference temperature in °C

    // Buck converter parameters
    static constexpr float L = 100e-6;     // Inductor value in H
    static constexpr float C_IN = 0.01;    // Input capacitor (10000uF) - IMPORTANT: Used for rint calculation
    static constexpr float C_OUT = 470e-6; // Output capacitor
    static constexpr float R_L = 0.1;      // Inductor DCR at reference temp
    static constexpr float R_DS_ON = 0.1;  // MOSFET on-resistance at reference temp
    static constexpr float TC_R_L = 0.004; // Temperature coefficient for inductor DCR
    static constexpr float TC_R_DS = 0.006;// Temperature coefficient for MOSFET

    // Switching parameters
    static constexpr float F_SW = 14000.0;  // Switching frequency in Hz
    static constexpr float T_SW = 1.0/F_SW; // Switching period
    static constexpr float T_DEAD = 1.0e-6; // Dead time

    // Protection thresholds
    static constexpr float I_MAX = 10.0;    // Maximum current limit
    static constexpr float V_MAX = 35.0;    // Maximum voltage limit
    static constexpr float TEMP_MAX = 85.0; // Maximum temperature

    // Control parameters
    static constexpr float LEARNING_RATE = 0.0001;
    static constexpr float VOLTAGE_FACTOR = 0.80;
    static constexpr float MIN_DUTY = 0.00;
    static constexpr float MAX_DUTY = 0.95;

    // Timing constants
    static const unsigned long STARTUP_DELAY = 1000;
    static const unsigned long CALIBRATION_INTERVAL_BASE = 20000; // Base calibration interval
    static unsigned long CALIBRATION_INTERVAL = CALIBRATION_INTERVAL_BASE; // Adaptive Calibration Interval - start with base
    static const unsigned long PROTECTION_CHECK_INTERVAL = 100;
    static const unsigned long DEBUG_SERIAL_OUTPUT_INTERVAL = 100;
    static const unsigned long SHORT_CHECK_INTERVAL = 2000; // Interval for short OC voltage check
    static const uint8_t SHORT_CHECK_SAMPLES = 3; // Samples for short OC check
    static const float OC_VOLTAGE_DIVERGENCE_THRESHOLD = 0.05; // Threshold for triggering full calibration (5% divergence)


// RC Fitting Configuration
static const uint8_t OC_VOLTAGE_SAMPLES = 15; // Increased samples for better fitting
static const unsigned long OC_VOLTAGE_SAMPLE_INTERVAL = 20; // Increased interval for clearer RC curve
static const uint8_t MAX_ITERATIONS = 20; // Maximum iterations for Gauss-Newton
static const float CONVERGENCE_THRESHOLD = 0.001; // Convergence threshold for parameter changes


class OptimizedBuckController {
private:
    AVR_PWM* pwm_instance;
    uint16_t pwm_period;

    // State variables
    float temperature =  TEMP_REFERENCE;
    float duty_cycle = 0;
    float source_voltage = 0;
    float OC_voltage = 0 ;
    float last_OC_voltage_full_calibration = 0; // Store OC voltage from full calibration
    float last_OC_voltage_short_check = 0;     // Store OC voltage from short check
    float target_voltage = 0;
    float load_voltage = 0;
    float inductor_current = 0;
    float input_current = 0;
    float error = 0;
    float rint = 0; // Estimated internal resistance

    // Impedance model
    struct ImpedanceModel {
        float R_source;    // Source internal resistance
        float R_inductor;  // Temperature-compensated inductor resistance
        float R_fet;       // Temperature-compensated FET resistance
        float X_L;         // Inductive reactance
        float X_C;         // Capacitive reactance
        float Z_total;     // Total impedance magnitude
    } impedance;

    // Protection state
    bool fault_condition = false;
    unsigned long last_protection_check = 0;
    // calibration state
    unsigned long last_calibration = 0;
    unsigned long last_oc_voltage_short_check_time = 0; // Time of last short OC check
    // debug out
    unsigned long last_debug_output = 0;


    void updateTemperatureCompensation() {
        float temp_diff = temperature -  TEMP_REFERENCE;
        impedance.R_inductor =  R_L * (1 +  TC_R_L * temp_diff);
        impedance.R_fet =  R_DS_ON * (1 +  TC_R_DS * temp_diff);
    }

    float calculateSwitchingLosses() {
        float P_sw = 0;
        // Switching losses
        P_sw += 0.5 * source_voltage * inductor_current *  F_SW *
                ( T_DEAD +  T_SW * (duty_cycle + (1 - duty_cycle)));
        // Conduction losses
        P_sw += impedance.R_fet * sq(inductor_current) * duty_cycle;
        return P_sw;
    }

    void updateImpedanceModel() {
        float omega = 2 * PI *  F_SW;

        // Update reactances
        impedance.X_L = omega *  L;
        impedance.X_C = 1 / (omega *  C_IN);

        // Calculate total impedance including switching effects
        float Z_dc = impedance.R_source + impedance.R_inductor + impedance.R_fet * duty_cycle;
        float Z_ac = sqrt(sq(impedance.X_L - impedance.X_C) + sq(Z_dc));

        impedance.Z_total = Z_dc * (1 - duty_cycle) + Z_ac * duty_cycle;
    }

    bool checkProtection() {
        if (source_voltage >  V_MAX ||
            load_voltage >  V_MAX ||
            inductor_current >  I_MAX ||
            temperature >  TEMP_MAX) {
            fault_condition = true;
            setPWM(0); // Shutdown PWM
            return true;
        }
        return false;
    }

    void updateControl() {
        if (fault_condition) return;

        // Calculate target voltage considering losses
        float switching_losses = calculateSwitchingLosses();
        target_voltage =  VOLTAGE_FACTOR * OC_voltage *
                             (1 - switching_losses / (source_voltage * input_current+0.0001));

        // Update error with impedance compensation
        float voltage_drop = impedance.Z_total * input_current;
        //error = (error + (target_voltage - (OC_voltage + voltage_drop))) / 2;
        error = (error+ (target_voltage-source_voltage))/2;
        // Update duty cycle with adaptive learning rate
        //float adaptive_rate =  LEARNING_RATE * (1 - duty_cycle * duty_cycle);
        //duty_cycle = fabs(duty_cycle + adaptive_rate * error);
        duty_cycle = constrain(duty_cycle -  LEARNING_RATE * error, 0, 1);
        //Serial.println(error);
        duty_cycle = constrain(duty_cycle,  MIN_DUTY,  MAX_DUTY);

        setPWM(duty_cycle);
    }

    void setPWM(float value) {
        uint16_t pwm_value = value * pwm_period;
        pwm_instance->setPWM( PWM_PIN,  F_SW, pwm_value);
    }

    float readVoltage(uint8_t pin, float range) {
        return analogRead(pin) * (range / 1023.0);
    }

    float readTemperature() {
        // Simple temperature reading - adjust based on your sensor
        return 25;
//        return (analogRead( TEMP_PIN) * (5.0 / 1023.0) - 0.5) * 100;
    }

    // Function to solve 2x2 linear system using Gaussian Elimination
    bool solveLinearSystem(float A[2][2], float b[2], float x[2]) {
        float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (fabs(det) < 1e-6) { // Check if determinant is close to zero (singular matrix)
            Serial.println(F("Error: Singular matrix in solver."));
            return false; // Indicate failure
        }

        // Gaussian Elimination (or Cramer's Rule - which is effectively what this is for 2x2)
        x[0] = (b[0] * A[1][1] - b[1] * A[0][1]) / det;
        x[1] = (b[1] * A[0][0] - b[0] * A[1][0]) / det;
        return true; // Indicate success
    }


    float fitRCProfile() {
        float voltage_samples[OC_VOLTAGE_SAMPLES];
        unsigned long time_samples[OC_VOLTAGE_SAMPLES];

        Serial.println(F("Sampling OC Voltage for RC fit..."));
        for (int i = 0; i < OC_VOLTAGE_SAMPLES; ++i) {
            time_samples[i] = millis();
            voltage_samples[i] = readVoltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
            Serial.print(F("Sample ")); Serial.print(i); Serial.print(F(": ")); Serial.print(voltage_samples[i]); Serial.print(F("V at ")); Serial.print(time_samples[i]); Serial.println(F("ms"));
            delay(OC_VOLTAGE_SAMPLE_INTERVAL);
        }

        // Gauss-Newton Non-Linear Least Squares Fitting

        float Voc_estimate = 20.0; // Initial guess for Voc (adjust as needed)
        float tau_estimate = 0.05; // Initial guess for tau (time constant, in seconds) - 50ms (adjust based on capacitor and expected rint)
        float last_Voc = DBL_MAX;
        float last_tau = DBL_MAX;

        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            if (fabs(Voc_estimate - last_Voc) < CONVERGENCE_THRESHOLD && fabs(tau_estimate - last_tau) < CONVERGENCE_THRESHOLD) {
                Serial.print(F("Converged after ")); Serial.print(iteration); Serial.println(F(" iterations."));
                break; // Convergence achieved
            }
            last_Voc = Voc_estimate;
            last_tau = tau_estimate;

            float sum_residuals_Voc = 0;
            float sum_residuals_tau = 0;
            float sum_jacobian_Voc_sq = 0;
            float sum_jacobian_tau_sq = 0;
            float sum_jacobian_Voc_tau = 0;


            for (int i = 0; i < OC_VOLTAGE_SAMPLES; ++i) {
                float t = (time_samples[i] - time_samples[0]) / 1000.0; // Time in seconds
                float V_measured = voltage_samples[i];
                float exp_term = exp(-t / tau_estimate);
                float V_model = Voc_estimate * (1.0 - exp_term);
                float residual = V_measured - V_model;

                //Jacobians (partial derivatives)
                float jacobian_Voc = (1.0 - exp_term);
                float jacobian_tau = Voc_estimate * (t / pow(tau_estimate, 2)) * exp_term;

                sum_residuals_Voc += residual * jacobian_Voc;
                sum_residuals_tau += residual * jacobian_tau;
                sum_jacobian_Voc_sq += pow(jacobian_Voc, 2);
                sum_jacobian_tau_sq += pow(jacobian_tau, 2);
                sum_jacobian_Voc_tau += jacobian_Voc * jacobian_tau;
            }

            // Construct the 2x2 matrix and vector for the linear system
            float Jacobian_transpose_Jacobian[2][2] = {
                {sum_jacobian_Voc_sq, sum_jacobian_Voc_tau},
                {sum_jacobian_Voc_tau, sum_jacobian_tau_sq}
            };
            float Jacobian_transpose_residuals[2] = {sum_residuals_Voc, sum_residuals_tau};
            float deltas[2];

            // Solve the linear system using Gaussian Elimination
            if (!solveLinearSystem(Jacobian_transpose_Jacobian, Jacobian_transpose_residuals, deltas)) {
                Serial.println(F("Linear system solve failed, stopping iteration."));
                break; // Stop iterating if solver fails
            }

            Voc_estimate += deltas[0];
            tau_estimate += deltas[1];


            if (tau_estimate < 0 || tau_estimate > 1) { // Basic bounds for tau, adjust if needed. Prevents non-physical values and potential divergence
                tau_estimate = max(0.001f, min(0.5f, last_tau)); // Clamp tau to a reasonable range (1ms to 500ms)
                Serial.println(F("Tau out of bounds, clamping."));
                break; // Stop iteration if tau becomes invalid
            }
             if (Voc_estimate < 0 || Voc_estimate > SOURCE_VOLTAGE_RANGE * 1.2) { // Basic bounds for Voc, adjust if needed.
                Voc_estimate = max(0.0f, min(SOURCE_VOLTAGE_RANGE * 1.1f, last_Voc));
                Serial.println(F("Voc out of bounds, clamping."));
                break; // Stop iteration if Voc becomes invalid.
            }
            Serial.print(F("Iteration ")); Serial.print(iteration); Serial.print(F(": Voc est = ")); Serial.print(Voc_estimate); Serial.print(F(", Tau est = ")); Serial.println(tau_estimate);
        }


        rint = tau_estimate / C_IN; // Calculate rint
        OC_voltage = Voc_estimate;

        Serial.print(F("Fitted OC Voltage (Voc): ")); Serial.println(OC_voltage);
        Serial.print(F("Fitted Time Constant (tau): ")); Serial.println(tau_estimate);
        Serial.print(F("Estimated Internal Resistance (rint): ")); Serial.println(rint);

        return OC_voltage;
    }

    float shortCheckOCVoltage() {
        float voltage_samples[SHORT_CHECK_SAMPLES];
        float voltage_sum = 0;
        Serial.println(F("Performing Short OC Voltage Check..."));
        for (int i = 0; i < SHORT_CHECK_SAMPLES; ++i) {
            voltage_samples[i] = readVoltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
            voltage_sum += voltage_samples[i];
            delay(OC_VOLTAGE_SAMPLE_INTERVAL); // Use same interval as full calibration for consistent settling
        }
        float avg_voltage = voltage_sum / SHORT_CHECK_SAMPLES;
        Serial.print(F("Short Check Avg OC Voltage: ")); Serial.println(avg_voltage);
        return avg_voltage;
    }


public:
    OptimizedBuckController() : pwm_instance(nullptr), pwm_period(0) {}

    void begin() {
        Serial.begin(115200);
        pinMode( PWM_PIN, OUTPUT);

        // Soft start initialization
        delay( STARTUP_DELAY);

        pwm_instance = new AVR_PWM( PWM_PIN,  F_SW, 0);
        if (pwm_instance) {
            pwm_instance->setPWM();
            pwm_period = pwm_instance->getPWMPeriod();
        }

        // Initial calibration
        calibrate();
        last_OC_voltage_full_calibration = OC_voltage; // Store initial calibration OC voltage
    }

    void calibrate() {
        // Perform initial measurements - Full Calibration with RC Fitting
        setPWM(0);
        delay(100); // Short delay after PWM off to let voltage settle slightly, before sampling

        // Measure open circuit characteristics using RC fitting (Gauss-Newton)
        OC_voltage = fitRCProfile(); // Now uses Gauss-Newton RC fitting
        last_OC_voltage_full_calibration = OC_voltage; // Update last full calibration OC voltage


        temperature = readTemperature();

        // Update temperature compensation
        updateTemperatureCompensation();

        // Start with safe duty cycle
        duty_cycle = 0.1;
        fault_condition = false;
        CALIBRATION_INTERVAL = CALIBRATION_INTERVAL_BASE; // Reset calibration interval after full calibration
    }

    void update() {
        unsigned long now = millis();

        // Regular measurements
        source_voltage = readVoltage( SOURCE_PIN,  SOURCE_VOLTAGE_RANGE);
        load_voltage = readVoltage( LOAD_PIN,  LOAD_VOLTAGE_RANGE);
        temperature = readTemperature();

        // Update models
        updateTemperatureCompensation();
        updateImpedanceModel();

        // Protection check
        if (now - last_protection_check >=  PROTECTION_CHECK_INTERVAL) {
            checkProtection();
            last_protection_check = now;
        }

        // Short OC Voltage Check for Drift Detection and Adaptive Calibration
        if (now - last_oc_voltage_short_check_time >= SHORT_CHECK_INTERVAL) {
            last_OC_voltage_short_check = shortCheckOCVoltage();
            last_oc_voltage_short_check_time = now;

            float voltage_divergence = fabs(last_OC_voltage_short_check - last_OC_voltage_full_calibration) / last_OC_voltage_full_calibration;

            if (voltage_divergence > OC_VOLTAGE_DIVERGENCE_THRESHOLD) {
                Serial.println(F("OC Voltage Divergence detected, initiating full recalibration."));
                calibrate(); // Trigger full recalibration
                 CALIBRATION_INTERVAL = CALIBRATION_INTERVAL_BASE; // Keep base interval after recalibration
            } else {
                // Gradually increase calibration interval if voltage is stable
                CALIBRATION_INTERVAL = min(CALIBRATION_INTERVAL + 1000, CALIBRATION_INTERVAL_BASE * 5); // Increase by 1s, up to 5x base interval
                Serial.print(F("OC Voltage Stable, increasing calibration interval to: ")); Serial.println(CALIBRATION_INTERVAL/1000.0); Serial.println(F("s"));
            }
        }


              // calibration - Full Calibration only if triggered by divergence or interval
        if (now - last_calibration >=  CALIBRATION_INTERVAL) {
            calibrate();
            last_calibration = now;
        }


        // Normal operation
        if (!fault_condition) {
            updateControl();
        }

        // Debug output
        if (millis() - last_debug_output >  DEBUG_SERIAL_OUTPUT_INTERVAL) {
            printDebugInfo();
            last_debug_output = millis();
        }
    }

    void printDebugInfo() {
        Serial.print(F("V_OC:")); Serial.print(OC_voltage);
        Serial.print(F(",V_target:")); Serial.print(target_voltage);
        Serial.print(F(",V_src:")); Serial.print(source_voltage);
        Serial.print(F(",V_load:")); Serial.print(load_voltage);
        Serial.print(F(",I_L:")); Serial.print(inductor_current);
        Serial.print(F(",Duty:")); Serial.print(duty_cycle);
        Serial.print(F(",Temp:")); Serial.print(temperature);
        Serial.print(F(",Fault:")); Serial.print(fault_condition);
        Serial.print(F(",rint:")); Serial.print(rint);
        Serial.print(F(",C_Int:")); Serial.print(CALIBRATION_INTERVAL/1000.0); Serial.println(F("s"));
    }
};

OptimizedBuckController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.update();
}
