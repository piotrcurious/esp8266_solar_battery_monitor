bool fit_rc_profile() {  
    if (NUM_SAMPLES < 2) return false;  

    fitted_voc = voltage_samples[NUM_SAMPLES - 1];  
    fitted_b = fitted_voc - voltage_samples[0];  
    fitted_tau = 0.01;  

    float prev_gradient_voc = 0.0;
    float prev_gradient_b = 0.0;
    float prev_gradient_tau = 0.0;
    float momentum_factor = 0.9; // Momentum term (adjust as needed)
    float adaptive_lr = RC_FIT_LEARNING_RATE;

    for (int iteration = 0; iteration < RC_FIT_ITERATIONS; iteration++) {  
        float gradient_voc = 0.0;  
        float gradient_b = 0.0;  
        float gradient_tau = 0.0;  
        float error_sum_squares = 0.0;  

        for (int i = 0; i < NUM_SAMPLES; i++) {  
            float t = (time_samples[i] - time_samples[0]) / 1000.0;  
            float measured_v = voltage_samples[i];  

            // Ensure exp(-t / fitted_tau) remains stable  
            float exp_term = exp(-t / max(fitted_tau, 1e-6));  
            float expected_v = fitted_voc - fitted_b * exp_term;  
            float error = measured_v - expected_v;  
            error_sum_squares += error * error;  

            gradient_voc += -2.0 * error;  
            gradient_b   +=  2.0 * error * exp_term;  
            gradient_tau +=  2.0 * error * fitted_b * (t / (fitted_tau * fitted_tau)) * exp_term;  
        }  

        // Apply momentum  
        gradient_voc = momentum_factor * prev_gradient_voc + (1 - momentum_factor) * gradient_voc;  
        gradient_b = momentum_factor * prev_gradient_b + (1 - momentum_factor) * gradient_b;  
        gradient_tau = momentum_factor * prev_gradient_tau + (1 - momentum_factor) * gradient_tau;  

        // Adaptive learning rate adjustment  
        float gradient_magnitude = sqrt(gradient_voc * gradient_voc + gradient_b * gradient_b + gradient_tau * gradient_tau);
        if (gradient_magnitude > 1.0) {  
            adaptive_lr = RC_FIT_LEARNING_RATE / gradient_magnitude;  
        }

        // Update parameters  
        fitted_voc -= adaptive_lr * gradient_voc / NUM_SAMPLES;  
        fitted_b   -= adaptive_lr * gradient_b   / NUM_SAMPLES;  
        fitted_tau -= adaptive_lr * gradient_tau / NUM_SAMPLES;  

        // Constrain values  
        fitted_tau = max(fitted_tau, 1e-6);  
        fitted_b   = max(fitted_b,   0.0);  
        fitted_voc = max(fitted_voc, 0.0);  

        // Gradient clipping to prevent divergence  
        if (abs(gradient_voc) > 10.0) gradient_voc = (gradient_voc / abs(gradient_voc)) * 10.0;
        if (abs(gradient_b) > 10.0) gradient_b = (gradient_b / abs(gradient_b)) * 10.0;
        if (abs(gradient_tau) > 10.0) gradient_tau = (gradient_tau / abs(gradient_tau)) * 10.0;

        // Store previous gradients  
        prev_gradient_voc = gradient_voc;  
        prev_gradient_b = gradient_b;  
        prev_gradient_tau = gradient_tau;  

        if (iteration % (RC_FIT_ITERATIONS / 10) == 0) {  
            Serial.print(F("RC Fit Iter: ")); Serial.print(iteration);  
            Serial.print(F(", Voc: ")); Serial.print(fitted_voc);  
            Serial.print(F(", B: ")); Serial.print(fitted_b);  
            Serial.print(F(", Tau: ")); Serial.print(fitted_tau);  
            Serial.print(F(", Error^2: ")); Serial.println(error_sum_squares / NUM_SAMPLES);  
        }  
    }  

    if (fitted_voc > 0 && fitted_tau > 1e-6 && fitted_b >= 0) {  
        resistance_tau_est = fitted_tau / KNOWN_CAPACITANCE;  
        return true;  
    } else {  
        return false;  
    }  
}
