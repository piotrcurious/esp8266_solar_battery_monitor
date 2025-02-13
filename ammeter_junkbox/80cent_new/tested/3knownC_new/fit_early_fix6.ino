// Global variable to store the suggested sampling interval in milliseconds
int suggestedSamplingIntervalMs;

bool fit_rc_profile_early() {
    if (NUM_SAMPLES < 2) return false;

    // Constants for parameter constraints (same as before)
    constexpr float FALLBACK_TAU        = 0.01f;
    constexpr float MIN_TAU             = 1e-6f;
    constexpr float MAX_TAU             = 1.0f;
    constexpr float MIN_LR              = 1e-5f;
    constexpr float MAX_LR              = 0.1f;
    constexpr float MOMENTUM            = 0.9f;
    constexpr float MIN_IMPROVEMENT     = 1e-6f;

    // Constants for suggested sampling interval calculation (same as before)
    constexpr float SUGGESTED_INTERVAL_TAU_FACTOR = 0.25f; // Aim for interval to be a fraction of tau
    constexpr int   MIN_INTERVAL_MS_SUGGESTED   = 5;    // Minimum suggested interval in milliseconds
    constexpr int   MAX_INTERVAL_MS_SUGGESTED   = 50;   // Maximum suggested interval in milliseconds
    constexpr int   DEFAULT_INTERVAL_MS         = 20;   // Default interval if suggestion fails or fit is not valid

    // Constants for derivative check (same as before)
    constexpr float MIN_DERIVATIVE_START_VOLTAGE_RATE = 0.01f; // Minimum initial dV/dt in V/s to consider valid charging
    constexpr float DERIVATIVE_DECREASE_THRESHOLD_RATIO = 0.7f; // Ratio of points where derivative decreases to total points for valid exponential decay

    // Constants for derivative-based interval adjustment (NEW - for interval increase as well)
    constexpr float DERIVATIVE_INTERVAL_DECREASE_FACTOR = 0.1f; // Tune this factor - higher value means more aggressive interval reduction (same as before)
    constexpr float LOW_DERIVATIVE_THRESHOLD          = 0.05f; // Threshold for considering derivative as "low" - adjust as needed
    constexpr float INTERVAL_INCREASE_FACTOR          = 0.5f;  // Percentage increase of interval when derivative is low (0.5f = 50% increase)


    // Initial parameter estimates (same as before)
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];
    fitted_b    = fitted_voc - voltage_samples[0];

    float t0 = time_samples[0] / 1000.0f;  // Convert to seconds

    // Estimate initial tau (same as before)
    float sum_ln = 0.0f, sum_t = 0.0f;
    int valid_points = 0;

    for (int i = 1; i < NUM_SAMPLES / 2; i++) {
        float t_i = (time_samples[i] / 1000.0f) - t0;
        float v_diff = fitted_voc - voltage_samples[i];
        float v_init = fitted_voc - voltage_samples[0];

        if (v_diff > 0.001f && v_init > 0.001f) {
            float ln_ratio = logf(v_diff / v_init);
            if (fabsf(ln_ratio) > 1e-6f) {
                sum_ln += ln_ratio;
                sum_t  += t_i;
                valid_points++;
            }
        }
    }

    fitted_tau = (valid_points > 0 && fabsf(sum_ln) > 1e-6f) ? -sum_t / sum_ln : FALLBACK_TAU;
    fitted_tau = constrain(fitted_tau, MIN_TAU, MAX_TAU);

    // --- Derivative based feature detection (same as before) ---
    float derivative[NUM_SAMPLES - 1];
    bool derivative_check_passed = false;
    int decreasing_derivative_points = 0;
    float max_abs_derivative = 0.0f; // Track max derivative magnitude

    for (int i = 1; i < NUM_SAMPLES; i++) {
        float dt = (time_samples[i] - time_samples[i-1]) / 1000.0f; // Time difference in seconds
        if (dt > 0) { // Avoid division by zero if time samples are identical (should not happen ideally)
            derivative[i-1] = (voltage_samples[i] - voltage_samples[i-1]) / dt;
        } else {
            derivative[i-1] = 0; // Or handle this case appropriately, maybe skip the point
        }
        if (i > 1 && derivative[i-1] < derivative[i-2]) {
            decreasing_derivative_points++;
        }
        max_abs_derivative = max(max_abs_derivative, abs(derivative[i-1])); // Update max derivative
    }

    if (NUM_SAMPLES > 2 && derivative[0] >= MIN_DERIVATIVE_START_VOLTAGE_RATE && (float)decreasing_derivative_points / (NUM_SAMPLES - 2) >= DERIVATIVE_DECREASE_THRESHOLD_RATIO) {
        derivative_check_passed = true;
    }

#ifdef FITTING_DEBUG
    Serial.print(F("Derivative Check: "));
    Serial.println(derivative_check_passed ? F("Passed") : F("Failed"));
    if (!derivative_check_passed) {
        Serial.print(F("  Initial dV/dt: ")); Serial.println(derivative[0], 4);
        Serial.print(F("  Decreasing Derivative Ratio: ")); Serial.println((float)decreasing_derivative_points / (NUM_SAMPLES - 2), 4);
    }
#endif

    if (!derivative_check_passed) {
        suggestedSamplingIntervalMs = DEFAULT_INTERVAL_MS;
        return false; // Derivative check failed, so return fit as failed - data might not be good
    }


    // Gradient descent fitting (same as before)
    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float prev_grad_voc = 0.0f, prev_grad_b = 0.0f, prev_grad_tau = 0.0f;
    float prev_error = INFINITY;
    int stagnant_iters = 0;

    for (int iter = 0; iter < RC_FIT_ITERATIONS; iter++) {
        float grad_voc = 0.0f, grad_b = 0.0f, grad_tau = 0.0f, error_sum = 0.0f;
        float inv_tau = 1.0f / max(fitted_tau, MIN_TAU);  // Precompute division for efficiency

        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] / 1000.0f) - t0;
            float exp_term = expf(-t * inv_tau);
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = voltage_samples[i] - expected_v;
            error_sum += error * error;

            // Compute gradients
            grad_voc += -2.0f * error;
            grad_b    +=  2.0f * error * exp_term;
            grad_tau += (fitted_b * t * exp_term * grad_b * inv_tau) * 2.0f;
        }

        float current_error = error_sum / NUM_SAMPLES;

        // Early stopping (same as before)
        if (fabsf(prev_error - current_error) < MIN_IMPROVEMENT) {
            if (++stagnant_iters > 5) break;
        } else {
            stagnant_iters = 0;
        }

        // Apply momentum (same as before)
        grad_voc = MOMENTUM * prev_grad_voc + (1 - MOMENTUM) * grad_voc;
        grad_b    = MOMENTUM * prev_grad_b    + (1 - MOMENTUM) * grad_b;
        grad_tau = MOMENTUM * prev_grad_tau + (1 - MOMENTUM) * grad_tau;

        prev_grad_voc = grad_voc;
        prev_grad_b    = grad_b;
        prev_grad_tau = grad_tau;

        // Update parameters (same as before)
        float inv_samples = 1.0f / NUM_SAMPLES;  // Precompute division
        fitted_voc -= adaptive_lr * grad_voc * inv_samples;
        fitted_b    -= adaptive_lr * grad_b * inv_samples;
        fitted_tau -= adaptive_lr * grad_tau * inv_samples;

        // Constrain values (same as before)
        fitted_tau = constrain(fitted_tau, MIN_TAU, MAX_TAU);
        fitted_b    = max(fitted_b, 0.0f);
        fitted_voc = constrain(fitted_voc,
                                        voltage_samples[NUM_SAMPLES - 1] * 0.8f,
                                        voltage_samples[NUM_SAMPLES - 1] * 1.2f);

        // Adaptive learning rate (same as before)
        adaptive_lr = constrain(adaptive_lr * (current_error > prev_error ? 0.5f : 1.1f), MIN_LR, MAX_LR);
        prev_error = current_error;

#ifdef FITTING_DEBUG
        if (iter % (RC_FIT_ITERATIONS / 5) == 0) {
            Serial.print(F("RC Fit: Iter "));
            Serial.print(iter);
            Serial.print(F(", Voc: "));
            Serial.print(fitted_voc, 4);
            Serial.print(F(", B: "));
            Serial.print(fitted_b, 4);
            Serial.print(F(", Tau: "));
            Serial.print(fitted_tau, 4);
            Serial.print(F(", Error: "));
            Serial.println(current_error, 6);
        }
#endif
    }

    // Final parameter validation (same as before)
    bool valid_params = (fitted_voc > 0.0f) &&
                        (fitted_tau > MIN_TAU) &&
                        (fitted_b >= 0.0f) &&
                        (fitted_voc < voltage_samples[0] * 1.5f);

    if (valid_params) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;

        // Calculate suggested sampling interval based on fitted tau (as before)
        float suggestedIntervalSec_tau_based = fitted_tau * SUGGESTED_INTERVAL_TAU_FACTOR;
        int suggestedIntervalMs_tau_based = round(suggestedIntervalSec_tau_based * 1000);

        // --- Derivative based interval adjustment (NEW - with increase logic) ---
        float interval_adjustment_factor = 1.0f; // Initialize to 1 (no change by default)

        if (max_abs_derivative > LOW_DERIVATIVE_THRESHOLD) {
            // Decrease interval if derivative is high (strong irradiation)
            interval_adjustment_factor = max(0.1f, min(1.0f, 1.0f / (1.0f + DERIVATIVE_INTERVAL_DECREASE_FACTOR * max_abs_derivative)));
        } else {
            // Increase interval if derivative is low (weak irradiation)
            float increase_percentage = (LOW_DERIVATIVE_THRESHOLD - max_abs_derivative) / LOW_DERIVATIVE_THRESHOLD * INTERVAL_INCREASE_FACTOR;
            interval_adjustment_factor = 1.0f + max(0.0f, increase_percentage); // Ensure no decrease from increase factor
        }
        suggestedSamplingIntervalMs = round(suggestedIntervalMs_tau_based * interval_adjustment_factor);
        suggestedSamplingIntervalMs = constrain(suggestedSamplingIntervalMs, MIN_INTERVAL_MS_SUGGESTED, MAX_INTERVAL_MS_SUGGESTED);


#ifdef FITTING_DEBUG
        Serial.print(F("Tau-based Interval (ms): ")); Serial.println(suggestedIntervalMs_tau_based);
        Serial.print(F("Max Abs Derivative (V/s): ")); Serial.println(max_abs_derivative, 4);
        Serial.print(F("Adjustment Factor: ")); Serial.println(interval_adjustment_factor, 4);
        Serial.print(F("Derivative Adjusted Interval (ms): ")); Serial.println(suggestedSamplingIntervalMs);
#endif


    } else {
        suggestedSamplingIntervalMs = DEFAULT_INTERVAL_MS; // Use default interval if fit is not valid or params are bad.
    }

    return valid_params && derivative_check_passed; // Return false if either fit or derivative check fails
}

void setup() {
  Serial.begin(9600);
  // Example usage - same as before, adjust dummy data to test both high and low irradiation scenarios
  #define NUM_SAMPLES 50
  float voltage_samples[NUM_SAMPLES];
  unsigned long time_samples[NUM_SAMPLES];
  #define RC_FIT_LEARNING_RATE 0.01f
  #define RC_FIT_ITERATIONS 100
  #define KNOWN_CAPACITANCE 0.0001f // Example capacitance
  float fitted_voc, fitted_b, fitted_tau, resistance_tau_est;


  // --- Example 1: Simulate Strong Irradiation (Fast Charging) ---
  Serial.println("--- Strong Irradiation Example ---");
  for(int i=0; i<NUM_SAMPLES; ++i) {
    time_samples[i] = i * 20; // 20ms interval example
    voltage_samples[i] = 0.0f + (5.0f - 0.0f) * (1.0f - exp(- (float)time_samples[i]/1000.0f / 0.02f)); // Faster charging (tau=0.02)
  }
  if (fit_rc_profile_early()) {
    Serial.print("Fit successful. Suggested Sampling Interval (ms): ");
    Serial.println(suggestedSamplingIntervalMs);
    Serial.print("Fitted Tau: ");
    Serial.println(fitted_tau, 4);
  } else {
    Serial.println("Fit failed OR Derivative Check Failed.");
    Serial.println("Default Sampling Interval (ms) used as suggestion: ");
    Serial.println(suggestedSamplingIntervalMs);
  }
  Serial.println("----------------------------------");

  delay(1000); // Small delay between examples

  // --- Example 2: Simulate Weak Irradiation (Slow Charging) ---
  Serial.println("--- Weak Irradiation Example ---");
  for(int i=0; i<NUM_SAMPLES; ++i) {
    time_samples[i] = i * 20; // 20ms interval example
    voltage_samples[i] = 0.0f + (5.0f - 0.0f) * (1.0f - exp(- (float)time_samples[i]/1000.0f / 0.5f)); // Slower charging (tau=0.5)
  }
  if (fit_rc_profile_early()) {
    Serial.print("Fit successful. Suggested Sampling Interval (ms): ");
    Serial.println(suggestedSamplingIntervalMs);
    Serial.print("Fitted Tau: ");
    Serial.println(fitted_tau, 4);
  } else {
    Serial.println("Fit failed OR Derivative Check Failed.");
    Serial.println("Default Sampling Interval (ms) used as suggestion: ");
    Serial.println(suggestedSamplingIntervalMs);
  }
   Serial.println("----------------------------------");


}

void loop() {
  // Your main loop code here
}
