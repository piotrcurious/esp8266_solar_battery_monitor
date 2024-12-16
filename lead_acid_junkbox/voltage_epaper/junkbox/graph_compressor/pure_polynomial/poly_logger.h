#ifndef POLY_LOGGER_H
#define __POLYNOMIAL_LOGGER__

#include <Arduino.h>
#include <cmath>
#include <cstdint>

// Configuration Constants
constexpr uint8_t NUM_POLYNOMIAL_SETS = 8;
constexpr uint8_t POLYNOMIAL_DEGREE = 5;
constexpr uint8_t COEFFICIENTS_PER_POLYNOMIAL = POLYNOMIAL_DEGREE + 1;
constexpr uint16_t MAX_SAMPLES = 64;

// Quantized 8-bit polynomial coefficient representation
struct QuantizedPolynomial {
    int8_t coefficients[COEFFICIENTS_PER_POLYNOMIAL];
    uint16_t timeDelta;  // 16-bit time between samples
};

// Polynomial Logging Storage Structure
struct PolynomialLoggerState {
    QuantizedPolynomial polynomialSets[NUM_POLYNOMIAL_SETS][MAX_SAMPLES];
    uint8_t currentSet = 0;
    uint8_t currentSetSampleCount = 0;
};

class PolynomialLogger {
private:
    PolynomialLoggerState state;
    
    // Polynomial quantization and compression methods will be declared here
    int8_t quantizeCoefficient(double coefficient);
    double dequantizeCoefficient(int8_t quantizedCoeff);
    
    // Polynomial fitting and recompression methods
    bool fitPolynomialToSamples(const float* samples, uint16_t sampleCount, float* coefficients);
    void compressPolynomialSet(uint8_t setIndex);
    void recompressOldestSets();

public:
    PolynomialLogger();
    
    void initialize();
    void logSample(float value);
    void processRandomIntervalSampling();
    
    // Utility and diagnostic methods
    void printCurrentState();
    uint16_t getAvailableStorageSpace();
};

extern PolynomialLogger polyLogger;

#endif // POLY_LOGGER_H
