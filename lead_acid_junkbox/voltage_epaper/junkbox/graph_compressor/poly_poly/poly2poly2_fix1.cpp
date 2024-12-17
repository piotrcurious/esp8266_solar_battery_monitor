void PolynomialLogger::recompressOldestSets() {
    // Combine and recompress the two oldest sets
    uint8_t set1 = (state.currentSet + NUM_POLYNOMIAL_SETS - 2) % NUM_POLYNOMIAL_SETS;
    uint8_t set2 = (state.currentSet + NUM_POLYNOMIAL_SETS - 1) % NUM_POLYNOMIAL_SETS;
    
    // Extract coefficients from existing polynomials
    float coefficientsSet1[COEFFICIENTS_PER_POLYNOMIAL];
    float coefficientsSet2[COEFFICIENTS_PER_POLYNOMIAL];
    
    // Dequantize coefficients for both sets
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        coefficientsSet1[i] = dequantizeCoefficient(
            state.polynomialSets[set1][0].coefficients[i]
        );
        coefficientsSet2[i] = dequantizeCoefficient(
            state.polynomialSets[set2][0].coefficients[i]
        );
    }
    
    // Advanced polynomial combination strategy
    float combinedCoefficients[COEFFICIENTS_PER_POLYNOMIAL] = {0};
    
    // Weighted combination approach
    // Use a more sophisticated merging strategy that considers 
    // polynomial characteristics and temporal relationship
    constexpr float MERGE_WEIGHT_1 = 0.6;  // Weight for first set
    constexpr float MERGE_WEIGHT_2 = 0.4;  // Weight for second set
    
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        // Weighted linear combination with non-linear scaling
        combinedCoefficients[i] = 
            (MERGE_WEIGHT_1 * coefficientsSet1[i]) + 
            (MERGE_WEIGHT_2 * coefficientsSet2[i]);
        
        // Apply non-linear scaling to preserve higher-order characteristics
        combinedCoefficients[i] *= std::pow(
            1.0f + std::abs(combinedCoefficients[i]), 
            0.5f * (i + 1)  // Increasing non-linearity for higher-degree terms
        );
    }
    
    // Implement advanced coefficient normalization
    // Prevent coefficient explosion during merging
    float normalizationFactor = 1.0f;
    float maxCoeff = 0.0f;
    
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        maxCoeff = std::max(maxCoeff, std::abs(combinedCoefficients[i]));
    }
    
    // Adaptive normalization
    if (maxCoeff > 1.0f) {
        normalizationFactor = 1.0f / maxCoeff;
    }
    
    // Apply normalization and quantization
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        // Normalize and then quantize
        float normalizedCoeff = combinedCoefficients[i] * normalizationFactor;
        
        // Store quantized coefficient
        state.polynomialSets[set1][0].coefficients[i] = 
            quantizeCoefficient(normalizedCoeff);
    }
    
    // Optional: Add statistical validity check
    float entropyMeasure = 0.0f;
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        entropyMeasure += std::abs(combinedCoefficients[i]);
    }
    
    // If merged polynomial seems statistically insignificant, 
    // fall back to a simpler merging strategy
    if (entropyMeasure < 0.1f) {
        // Simple average merging as fallback
        for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
            float avgCoeff = (coefficientsSet1[i] + coefficientsSet2[i]) * 0.5f;
            state.polynomialSets[set1][0].coefficients[i] = 
                quantizeCoefficient(avgCoeff);
        }
    }
    
    // Zero out the second set to mark it as merged
    std::memset(&state.polynomialSets[set2], 0, sizeof(QuantizedPolynomial) * MAX_SAMPLES);
    
    // Optional: Log recompression event
    #ifdef DEBUG_LOGGING
    Serial.println("Polynomial Sets Recompressed");
    #endif
}

// Enhanced entropy and validity checking (optional helper method)
float PolynomialLogger::computePolynomialEntropy(const float* coefficients) {
    float entropy = 0.0f;
    float totalMagnitude = 0.0f;
    
    for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
        float coeffMagnitude = std::abs(coefficients[i]);
        entropy += coeffMagnitude * std::log2(1.0f + coeffMagnitude);
        totalMagnitude += coeffMagnitude;
    }
    
    // Normalize entropy
    return (totalMagnitude > 0) ? (entropy / totalMagnitude) : 0.0f;
}
