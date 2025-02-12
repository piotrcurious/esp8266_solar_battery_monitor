bool fit_rc_profile() {
    if (NUM_SAMPLES < 2) return false;

    // Initialize parameters with better initial guesses
    fitted_voc = voltage_samples[NUM_SAMPLES - 1];  // Final voltage as Voc
    fitted_b = fitted_voc - voltage_samples[0];     // Initial voltage drop
    
    // Estimate initial tau using two points method
    float v1 = voltage_samples[0];
    float v2 = voltage_samples[NUM_SAMPLES/2];
    float t1 = (time_samples[0] - time_samples[0]) / 1000.0;
    float t2 = (time_samples[NUM_SAMPLES/2] - time_samples[0]) / 1000.0;
    fitted_tau = (t2 - t1) / log((fitted_voc - v1) / (fitted_voc - v2));
    
    // Adaptive learning rate parameters
    float adaptive_lr = RC_FIT_LEARNING_RATE;
    float momentum_voc = 0.0;
    float momentum_b = 0.0;
    float momentum_tau = 0.0;
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
            float exp_term = exp(-t / fitted_tau);
            float expected_v = fitted_voc - fitted_b * exp_term;
            float error = measured_v - expected_v;
            error_sum_squares += error * error;
            
            // More numerically stable gradient calculations
            gradient_voc += -2.0 * error;
            gradient_b += 2.0 * error * exp_term;
            gradient_tau += 2.0 * error * fitted_b * (t / (fitted_tau * fitted_tau)) * exp_term;
        }
        
        float current_error = error_sum_squares / NUM_SAMPLES;
        
        // Check for convergence
        if (fabs(prev_error - current_error) < min_improvement) {
            stagnant_iterations++;
            if (stagnant_iterations > 5) {
                Serial.println(F("RC Fit: Early convergence reached"));
                break;
            }
        } else {
            stagnant_iterations = 0;
        }
        
        // Apply momentum and adaptive learning rate
        momentum_voc = momentum_factor * momentum_voc - adaptive_lr * gradient_voc / NUM_SAMPLES;
        momentum_b = momentum_factor * momentum_b - adaptive_lr * gradient_b / NUM_SAMPLES;
        momentum_tau = momentum_factor * momentum_tau - adaptive_lr * gradient_tau / NUM_SAMPLES;
        
        fitted_voc += momentum_voc;
        fitted_b += momentum_b;
        fitted_tau += momentum_tau;
        
        // Parameter constraints with smooth clamping
        fitted_tau = max(fitted_tau, 1e-6);
        fitted_b = max(fitted_b, 0.0);
        fitted_voc = max(fitted_voc, voltage_samples[NUM_SAMPLES - 1] * 0.5);
        
        // Adaptive learning rate adjustment
        if (current_error > prev_error) {
            adaptive_lr *= 0.5;
        } else if (stagnant_iterations == 0) {
            adaptive_lr *= 1.1;
        }
        adaptive_lr = constrain(adaptive_lr, 0.0001, 0.1);
        
        prev_error = current_error;
        
        // Debug output with reduced frequency
        if (iteration % (RC_FIT_ITERATIONS / 5) == 0) {
            Serial.print(F("RC Fit: "));
            Serial.print(iteration);
            Serial.print(F(", Error: "));
            Serial.print(current_error, 6);
            Serial.print(F(", LR: "));
            Serial.println(adaptive_lr, 6);
        }
    }
    
    // Validation checks
    bool parameters_valid = (fitted_voc > 0) && 
                          (fitted_tau > 1e-6) && 
                          (fitted_b >= 0) &&
                          (fitted_voc < voltage_samples[0] * 2.0);  // Sanity check
                          
    if (parameters_valid) {
        // Calculate final R estimate using known capacitance
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;
        
        // Additional validation of resistance estimate
        if (resistance_tau_est > 0 && resistance_tau_est < 1000.0) {  // Adjust limits as needed
            return true;
        }
    }
    
    return false;
}
