float calculateBlendingFactor() {
    // Analyze coefficient characteristics
    float coeffNorm = 0.0f;
    float coeffVariance = 0.0f;
    float dcComponent = chebCoeffs[0];
    
    // Calculate coefficient norm and variance
    for (int i = 1; i <= CHEB_ORDER; i++) {
        coeffNorm += std::abs(chebCoeffs[i]);
        coeffVariance += chebCoeffs[i] * chebCoeffs[i];
    }
    
    // Normalize variance
    coeffVariance = std::sqrt(coeffVariance / CHEB_ORDER);
    
    // Coefficient complexity metric
    float complexityFactor = coeffNorm / (std::abs(dcComponent) + 1e-6f);
    
    // Prediction error tracking
    float predictionErrorFactor = std::min(1.0f, 
        std::sqrt(std::abs(prediction_error_sum) / samplesPerRead)
    );
    
    // Dynamic adaptive blending strategy
    float baseBlendFactor = 0.3f;  // Default base blending
    
    // Adjust blending based on multiple criteria
    float adaptiveBlend = baseBlendFactor * (1.0f + 
        // Increase blend if coefficients are very complex
        0.5f * std::tanh(complexityFactor - 1.0f) +
        
        // Reduce blend if prediction error is low
        -0.3f * std::exp(-predictionErrorFactor) +
        
        // Additional damping based on coefficient variance
        -0.2f * std::tanh(coeffVariance)
    );
    
    // Constrain blending factor
    return std::max(0.1f, std::min(0.7f, adaptiveBlend));
}

// Additional supporting members in the class
private:
    float prediction_error_sum = 0.0f;
    int samplesPerRead = 10;  // Default value, updated during read

// Modification in the read() method to track prediction error
float read(int pin, int samplesPerRead = 10) {
    // ... existing code ...
    
    // Store samplesPerRead for blending factor calculation
    this->samplesPerRead = samplesPerRead;
    
    // Reset prediction error sum
    prediction_error_sum = 0.0f;
    
    // In the prediction error calculation loop
    for (int i = 0; i < samplesPerRead; i++) {
        float predicted = predictWithChebyshev(i * deltaT, deltaT * samplesPerRead, minSample, maxSample);
        prediction_error_sum += pow(predicted - samples[i], 2);
    }
    
    // ... rest of the existing read() method ...
}
