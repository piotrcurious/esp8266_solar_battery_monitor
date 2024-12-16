#include "PolyLogger.h"

// Coefficient quantization methods
int8_t PolynomialLogger::quantizeCoefficient(double coefficient) {
    // Linear quantization mapping float to 8-bit signed integer
    // Assumes coefficient range typically between -1.0 and 1.0
    constexpr double QUANT_SCALE = 127.0;
    constexpr double CLAMP_MIN = -1.0;
    constexpr double CLAMP_MAX = 1.0;
    
    // Clamp the coefficient to expected range
    double clampedCoeff = std::max(CLAMP_MIN, std::min(CLAMP_MAX, coefficient));
    
    // Scale and convert to integer
    return static_cast<int8_t>(std::round(clampedCoeff * QUANT_SCALE));
}

double PolynomialLogger::dequantizeCoefficient(int8_t quantizedCoeff) {
    constexpr double DEQUANT_SCALE = 1.0 / 127.0;
    return static_cast<double>(quantizedCoeff) * DEQUANT_SCALE;
}

// Advanced least squares polynomial fitting
bool PolynomialLogger::fitPolynomialToSamples(
    const float* samples, 
    uint16_t sampleCount, 
    float* coefficients
) {
    // Implement Vandermonde matrix-based least squares fitting
    // This is a computationally intensive method, so we'll use a compact approach
    
    if (sampleCount < COEFFICIENTS_PER_POLYNOMIAL) {
        return false;  // Insufficient data for fitting
    }
    
    // Prepare Vandermonde matrix and data vector
    float vandermondeMatrix[COEFFICIENTS_PER_POLYNOMIAL * COEFFICIENTS_PER_POLYNOMIAL] = {0};
    float dataVector[COEFFICIENTS_PER_POLYNOMIAL] = {0};
    
    // Normalized time mapping
    auto normalizeTime = [sampleCount](uint16_t index) {
        return 2.0f * (static_cast<float>(index) / (sampleCount - 1)) - 1.0f;
    };
    
    // Construct Vandermonde matrix
    for (uint16_t i = 0; i < sampleCount; ++i) {
        float t = normalizeTime(i);
        float tPower = 1.0f;
        
        for (uint8_t j = 0; j < COEFFICIENTS_PER_POLYNOMIAL; ++j) {
            vandermondeMatrix[i * COEFFICIENTS_PER_POLYNOMIAL + j] = tPower;
            tPower *= t;
        }
    }
    
    // Pseudo-inverse computation (simplified Gram-Schmidt QR decomposition)
    float qrMatrix[COEFFICIENTS_PER_POLYNOMIAL * COEFFICIENTS_PER_POLYNOMIAL];
    std::memcpy(qrMatrix, vandermondeMatrix, sizeof(qrMatrix));
    
    // QR decomposition with Gram-Schmidt orthogonalization
    for (uint8_t k = 0; k < COEFFICIENTS_PER_POLYNOMIAL; ++k) {
        // Compute column norm
        float norm = 0.0f;
        for (uint16_t i = k; i < sampleCount; ++i) {
            norm += qrMatrix[i * COEFFICIENTS_PER_POLYNOMIAL + k] * 
                    qrMatrix[i * COEFFICIENTS_PER_POLYNOMIAL + k];
        }
        norm = std::sqrt(norm);
        
        // Orthogonalization
        for (uint16_t i = k + 1; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
            float dot = 0.0f;
            for (uint16_t j = k; j < sampleCount; ++j) {
                dot += qrMatrix[j * COEFFICIENTS_PER_POLYNOMIAL + k] * 
                       qrMatrix[j * COEFFICIENTS_PER_POLYNOMIAL + i];
            }
            
            for (uint16_t j = k; j < sampleCount; ++j) {
                qrMatrix[j * COEFFICIENTS_PER_POLYNOMIAL + i] -= 
                    dot * qrMatrix[j * COEFFICIENTS_PER_POLYNOMIAL + k] / norm;
            }
        }
    }
    
    // Solve using back-substitution
    for (int8_t i = COEFFICIENTS_PER_POLYNOMIAL - 1; i >= 0; --i) {
        float sum = 0.0f;
        for (uint8_t j = i + 1; j < COEFFICIENTS_PER_POLYNOMIAL; ++j) {
            sum += qrMatrix[i * COEFFICIENTS_PER_POLYNOMIAL + j] * coefficients[j];
        }
        
        coefficients[i] = (dataVector[i] - sum) / qrMatrix[i * COEFFICIENTS_PER_POLYNOMIAL + i];
    }
    
    return true;
}

// Method to store a fitted polynomial set
void PolynomialLogger::compressPolynomialSet(uint8_t setIndex) {
    // Collect samples from the current set
    float samples[MAX_SAMPLES];
    float coefficients[COEFFICIENTS_PER_POLYNOMIAL];
    
    // Extract sample values
    for (uint16_t i = 0; i < state.currentSetSampleCount; ++i) {
        // Placeholder for actual sample extraction logic
        samples[i] = 0.0f;  // TODO: Implement actual sample retrieval
    }
    
    // Fit polynomial
    if (fitPolynomialToSamples(samples, state.currentSetSampleCount, coefficients)) {
        // Quantize and store coefficients
        for (uint8_t i = 0; i < COEFFICIENTS_PER_POLYNOMIAL; ++i) {
            state.polynomialSets[setIndex][i].coefficients[i] = 
                quantizeCoefficient(coefficients[i]);
        }
    }
}
