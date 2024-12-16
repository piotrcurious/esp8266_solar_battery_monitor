#include "PolyLogger.h"

// Global singleton instance
PolynomialLogger polyLogger;

// Constructor and initialization
PolynomialLogger::PolynomialLogger() {
    initialize();
}

void PolynomialLogger::initialize() {
    // Clear all polynomial sets
    std::memset(&state, 0, sizeof(PolynomialLoggerState));
    
    // Initialize random seed
    randomSeed(analogRead(0));
}

void PolynomialLogger::processRandomIntervalSampling() {
    // Generate random sampling interval 
    // Uses exponential backoff to create non-uniform sampling
    static uint32_t lastSampleTime = 0;
    uint32_t currentTime = millis();
    
    // Exponential random interval between 50ms and ~8 seconds
    uint32_t randomInterval = 50 + random(0, 1 << random(0, 8));
    
    if (currentTime - lastSampleTime >= randomInterval) {
        // Sample current sensor or data source
        float currentSample = 0.0f;  // TODO: Replace with actual sensor reading
        
        logSample(currentSample);
        
        lastSampleTime = currentTime;
    }
}

void PolynomialLogger::logSample(float value) {
    // Current polynomial set
    auto& currentSet = state.polynomialSets[state.currentSet];
    
    // Store sample in current set
    if (state.currentSetSampleCount < MAX_SAMPLES) {
        // Capture sample and time delta
        currentSet[state.currentSetSampleCount].timeDelta = 
            (state.currentSetSampleCount == 0) ? 0 : 
            millis() - currentSet[state.currentSetSampleCount - 1].timeDelta;
        
        // Placeholder for sample storage (will be part of fitting later)
        state.currentSetSampleCount++;
    }
    
    // Check if current set is full
    if (state.currentSetSampleCount >= MAX_SAMPLES) {
        // Compress current set
        compressPolynomialSet(state.currentSet);
        
        // Move to next set with rolling buffer logic
        state.currentSet = (state.currentSet + 1) % NUM_POLYNOMIAL_SETS;
        state.currentSetSampleCount = 0;
        
        // Periodically recompress oldest sets
        if (state.currentSet % 2 == 0) {
            recompressOldestSets();
        }
    }
}

void PolynomialLogger::recompressOldestSets() {
    // Combine and recompress the two oldest sets
    uint8_t set1 = (state.currentSet + NUM_POLYNOMIAL_SETS - 2) % NUM_POLYNOMIAL_SETS;
    uint8_t set2 = (state.currentSet + NUM_POLYNOMIAL_SETS - 1) % NUM_POLYNOMIAL_SETS;
    
    // Merge samples from both sets
    float mergedSamples[MAX_SAMPLES * 2];
    uint16_t mergedSampleCount = 0;
    
    // Reconstruct samples from set1 polynomials
    for (uint8_t i = 0; i < MAX_SAMPLES; ++i) {
        double reconstructedSample = 0.0;
        float t = 2.0f * (static_cast<float>(i) / (MAX_SAMPLES - 1)) - 1.0f;
        
        for (uint8_t j = 0; j < COEFFICIENTS_PER_POLYNOMIAL; ++j) {
            double coeff = dequantizeCoefficient(
                state.polynomialSets[set1][i].coefficients[j]
            );
            reconstructedSample += coeff * std::pow(t, j);
        }
        
        mergedSamples[mergedSampleCount++] = static_cast<float>(reconstructedSample);
    }
    
    // Repeat for set2
    for (uint8_t i = 0; i < MAX_SAMPLES; ++i) {
        double reconstructedSample = 0.0;
        float t = 2.0f * (static_cast<float>(i) / (MAX_SAMPLES - 1)) - 1.0f;
        
        for (uint8_t j = 0; j < COEFFICIENTS_PER_POLYNOMIAL; ++j) {
            double coeff = dequantizeCoefficient(
                state.polynomialSets[set2][i].coefficients[j]
            );
            reconstructedSample += coeff * std::pow(t, j);
        }
        
        mergedSamples[mergedSampleCount++] = static_cast<float>(reconstructedSample);
    }
    
    // Fit new combined polynomial
    float newCoefficients[COEFFICIENTS_PER_POLYNOMIAL];
    if (fitPolynomialToSamples(mergedSamples, mergedSampleCount, newCoefficients)) {
        // Store merged polynomial in set1, effectively replacing both sets
        for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
            state.polynomialSets[set1][i].coefficients[i] = 
                quantizeCoefficient(newCoefficients[i]);
        }
        
        // Mark set2 as unused by zeroing
        std::memset(&state.polynomialSets[set2], 0, sizeof(QuantizedPolynomial) * MAX_SAMPLES);
    }
}

// Utility methods for diagnostics and system management
void PolynomialLogger::printCurrentState() {
    Serial.println("Polynomial Logger State:");
    Serial.printf("Current Set: %d\n", state.currentSet);
    Serial.printf("Samples in Current Set: %d\n", state.currentSetSampleCount);
    
    // Optional: Add more detailed state printing
}

uint16_t PolynomialLogger::getAvailableStorageSpace() {
    // Calculate remaining storage in current set
    return MAX_SAMPLES - state.currentSetSampleCount;
}

// Arduino setup and loop integration
void setup() {
    Serial.begin(115200);
    polyLogger.initialize();
}

void loop() {
    // Periodic random interval sampling
    polyLogger.processRandomIntervalSampling();
    
    // Optional: Add periodic state reporting or other management
    static uint32_t lastReportTime = 0;
    if (millis() - lastReportTime > 10000) {  // Report every 10 seconds
        polyLogger.printCurrentState();
        lastReportTime = millis();
    }
}
