bool fit_rc_profile_early() {
    if (NUM_SAMPLES < 2) return false;

    // Initialize parameters with better initial guesses
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];  // Final voltage as Voc
    fitted_b = fitted_voc - voltage_samples[0];     // Initial voltage drop

    // Improved tau estimation using multiple points
    float sum_ln = 0.0;
    float sum_t = 0.0;
    int valid_points = 0;
    
    for (int i = 1; i < NUM_SAMPLES/2; i++) {
        float v_diff = fitted_voc - voltage_samples[i];
        float v_init = fitted_voc - voltage_samples[0];
        if (v_diff > 0.001 && v_init > 0.001) {  // Avoid log of zero or negative
            float t = (time_samples[i] - time_samples[0]) / 1000.0;
            sum_ln += log(v_diff / v_init);
            sum_t += t;
            valid_points++;
        }
    }
    
    fitted_tau = valid_points > 0 ? -sum_t / sum_ln : 0.01;  // Fallback to 0.01 if estimation fails
    fitted_tau = constrain(fitted_tau, 0.001, 1.0);  // Reasonable bounds for tau

    // Optimization parameters
    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float prev_gradients_voc = 0.0;
    float prev_gradients_b = 0.0;
    float prev_gradients_tau = 0.0;
    const float momentum_factor = 0.9;
    const float min_improvement = 1e-6;

    float prev_error = INFINITY;
    int stagnant_iterations = 0;

    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {
        float gradient_voc = 0.0;
        float gradient_b = 0.0;
        float gradient_tau = 0.0;
        float error_sum_squares = 0.0;

        // Calculate gradients and error
        for (int i = 0; i < NUM_SAMPLES; i++) {
            float t = (time_samples[i] - time_samples[0]) / 1000.0;
            float measured_v = voltage_samples[i];
            float exp_term = exp(-t / max(fitted_tau, 1e-6));
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = measured_v - expected_v;
            error_sum_squares += error * error;

            // Gradient calculation with better numerical stability
            gradient_voc += -2.0 * error;
            gradient_b += 2.0 * error * exp_term;
            if (fitted_tau > 1e-6) {  // Protect against tiny tau values
                gradient_tau += 2.0 * error * fitted_b * t * exp_term / (fitted_tau * fitted_tau);
            }
        }

        float current_error = error_sum_squares / NUM_SAMPLES;

        // Early stopping check
        if (fabs(prev_error - current_error) < min_improvement) {
            stagnant_iterations++;
            if (stagnant_iterations > 5) break;
        } else {
            stagnant_iterations = 0;
        }

        // Apply momentum to gradients
        gradient_voc = momentum_factor * prev_gradients_voc + (1 - momentum_factor) * gradient_voc;
        gradient_b = momentum_factor * prev_gradients_b + (1 - momentum_factor) * gradient_b;
        gradient_tau = momentum_factor * prev_gradients_tau + (1 - momentum_factor) * gradient_tau;

        // Store gradients for next iteration
        prev_gradients_voc = gradient_voc;
        prev_gradients_b = gradient_b;
        prev_gradients_tau = gradient_tau;

        // Update parameters with normalized gradients
        fitted_voc -= adaptive_lr * gradient_voc / NUM_SAMPLES;
        fitted_b -= adaptive_lr * gradient_b / NUM_SAMPLES;
        fitted_tau -= adaptive_lr * gradient_tau / NUM_SAMPLES;

        // Parameter constraints
        fitted_tau = constrain(fitted_tau, 1e-6, 1.0);
        fitted_b = max(fitted_b, 0.0);
        fitted_voc = constrain(fitted_voc, 
                             voltage_samples[NUM_SAMPLES - 1] * 0.8,  // Lower bound
                             voltage_samples[NUM_SAMPLES - 1] * 1.2); // Upper bound

        // Adaptive learning rate adjustment
        if (current_error > prev_error) {
            adaptive_lr *= 0.5;
        } else if (stagnant_iterations == 0) {
            adaptive_lr *= 1.1;
        }
        adaptive_lr = constrain(adaptive_lr, 1e-5, 0.1);

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

    // Validation checks
    bool parameters_valid = (fitted_voc > 0) &&
                          (fitted_tau > 1e-6) &&
                          (fitted_b >= 0) &&
                          (fitted_voc < voltage_samples[0] * 1.5);  // More conservative bound

    if (parameters_valid) {
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        return true;
    }

    return false;
}
