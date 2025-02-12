bool fit_rc_profile_early() {
    if (NUM_SAMPLES < 2) return false;

    // Constants for parameter bounds and fallback values
    const float fallback_tau      = 0.01f;
    const float min_tau           = 1e-6f;
    const float max_tau           = 1.0f;
    const float min_learning_rate = 1e-5f;
    const float max_learning_rate = 0.1f;
    const float momentum_factor   = 0.9f;
    const float min_improvement   = 1e-6f;

    // Initialize initial parameter guesses based on voltage samples.
    // Use the final voltage sample as Voc and the difference from the first sample as B.
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];
    fitted_b   = fitted_voc - voltage_samples[0];

    // Precompute the initial time (converted to seconds) for relative time calculations.
    float t0 = time_samples[0] / 1000.0f;

    // Estimate tau using multiple data points for a more robust initial guess.
    float sum_ln = 0.0f;
    float sum_t  = 0.0f;
    int valid_points = 0;

    for (int i = 1; i < NUM_SAMPLES / 2; i++) {
        float t_i   = (time_samples[i] / 1000.0f) - t0;
        float v_diff = fitted_voc - voltage_samples[i];
        float v_init = fitted_voc - voltage_samples[0];
        if (v_diff > 0.001f && v_init > 0.001f) {  // Avoid log of zero or negative values
            float ratio = v_diff / v_init;
            float ln_ratio = log(ratio);
            if (fabs(ln_ratio) > 1e-6f) {
                sum_ln += ln_ratio;
                sum_t  += t_i;
                valid_points++;
            }
        }
    }
    
    // If we gathered valid data, compute tau; otherwise, use a fallback value.
    if (valid_points > 0 && fabs(sum_ln) > 1e-6f) {
        fitted_tau = -sum_t / sum_ln;
    } else {
        fitted_tau = fallback_tau;
    }
    fitted_tau = constrain(fitted_tau, min_tau, max_tau);

    // Gradient descent parameters
    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float prev_grad_voc = 0.0f;
    float prev_grad_b   = 0.0f;
    float prev_grad_tau = 0.0f;
    float prev_error    = INFINITY;
    int stagnant_iterations = 0;

    // Main optimization loop
    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
        float grad_voc = 0.0f;
        float grad_b   = 0.0f;
        float grad_tau = 0.0f;
        float error_sum = 0.0f;

        // Compute error and gradients for each sample.
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // Convert time to seconds relative to the first sample.
            float t = (time_samples[i] / 1000.0f) - t0;
            float measured_v = voltage_samples[i];
            float exp_term = exp(-t / max(fitted_tau, min_tau));
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = measured_v - expected_v;
            error_sum += error * error;

            // Gradient with respect to fitted_voc (direct effect on expected voltage)
            grad_voc += -2.0f * error;
            // Gradient with respect to fitted_b (scaled by the exponential decay)
            grad_b   += 2.0f * error * exp_term;
            // Gradient with respect to fitted_tau (taking care to avoid division by near-zero)
            if (fitted_tau > min_tau) {
                grad_tau += 2.0f * error * fitted_b * t * exp_term / (fitted_tau * fitted_tau);
            }
        }

        float current_error = error_sum / NUM_SAMPLES;

        // Check for early stopping if improvements are minimal.
        if (fabs(prev_error - current_error) < min_improvement) {
            stagnant_iterations++;
            if (stagnant_iterations > 5) break;
        } else {
            stagnant_iterations = 0;
        }

        // Apply momentum to the gradients.
        grad_voc = momentum_factor * prev_grad_voc + (1 - momentum_factor) * grad_voc;
        grad_b   = momentum_factor * prev_grad_b   + (1 - momentum_factor) * grad_b;
        grad_tau = momentum_factor * prev_grad_tau + (1 - momentum_factor) * grad_tau;

        // Save current gradients for use in the next iteration.
        prev_grad_voc = grad_voc;
        prev_grad_b   = grad_b;
        prev_grad_tau = grad_tau;

        // Update the parameters with normalized gradients.
        fitted_voc -= adaptive_lr * (grad_voc / NUM_SAMPLES);
        fitted_b   -= adaptive_lr * (grad_b   / NUM_SAMPLES);
        fitted_tau -= adaptive_lr * (grad_tau / NUM_SAMPLES);

        // Constrain parameters to physically reasonable ranges.
        fitted_tau = constrain(fitted_tau, min_tau, max_tau);
        fitted_b   = max(fitted_b, 0.0f);
        // For Voc, keep it within 80%-120% of the final voltage sample.
        fitted_voc = constrain(fitted_voc, voltage_samples[NUM_SAMPLES - 1] * 0.8f,
                                voltage_samples[NUM_SAMPLES - 1] * 1.2f);

        // Adjust the learning rate adaptively.
        if (current_error > prev_error) {
            adaptive_lr *= 0.5f;
        } else if (stagnant_iterations == 0) {
            adaptive_lr *= 1.1f;
        }
        adaptive_lr = constrain(adaptive_lr, min_learning_rate, max_learning_rate);
        prev_error = current_error;

#ifdef FITTING_DEBUG
        if (iteration % (RC_FIT_ITERATIONS / 5) == 0) {
            Serial.print(F("Early RC Fit: "));
            Serial.print(iteration);
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

    // Final validation of the fitted parameters.
    bool valid_parameters = (fitted_voc > 0.0f) &&
                            (fitted_tau > min_tau) &&
                            (fitted_b >= 0.0f) &&
                            (fitted_voc < voltage_samples[0] * 1.5f); // Conservative upper bound

    if (valid_parameters) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        return true;
    }

    return false;
}
