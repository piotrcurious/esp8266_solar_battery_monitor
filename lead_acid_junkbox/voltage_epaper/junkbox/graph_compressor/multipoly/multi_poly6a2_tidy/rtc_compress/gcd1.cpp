#include <Arduino.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>

#define POLY_COUNT 5     // Number of polynomials per segment
#define POLY_DEGREE 5    // 5th degree polynomial (6 coefficients)
#define SEGMENTS 10      // Number of segments to store

// Improved compression strategy structure
struct CompressedPolynomialSegment {
    // Common denominator polynomial
    float denominatorCoeffs[POLY_DEGREE + 1];
    
    // Simplified numerator polynomials
    float simplifiedCoeffs[POLY_COUNT][POLY_DEGREE + 1];
    
    // Compression quality metrics
    float compressionRatio;
    float reconstructionError;
    
    uint32_t timeDeltas;
};

class AdvancedPolynomialCompressor {
private:
    // Numerical stability constants
    static constexpr float EPSILON = 1e-6;
    static constexpr int MAX_FACTORIZATION_ATTEMPTS = 5;

    // Compute polynomial GCD using Euclidean algorithm
    static std::vector<float> polynomialGCD(
        const std::vector<float>& a, 
        const std::vector<float>& b
    ) {
        std::vector<float> poly1 = a, poly2 = b;
        
        while (!poly2.empty() && !std::all_of(poly2.begin(), poly2.end(), 
               [](float x) { return std::abs(x) < EPSILON; })) {
            // Polynomial long division
            std::vector<float> quotient, remainder;
            polynomialDivision(poly1, poly2, quotient, remainder);
            
            poly1 = poly2;
            poly2 = remainder;
        }
        
        // Normalize the GCD
        if (!poly1.empty()) {
            float normalizeFactor = poly1.back();
            std::transform(poly1.begin(), poly1.end(), poly1.begin(), 
                [normalizeFactor](float x) { return x / normalizeFactor; });
        }
        
        return poly1;
    }

    // Polynomial long division
    static void polynomialDivision(
        const std::vector<float>& dividend, 
        const std::vector<float>& divisor,
        std::vector<float>& quotient, 
        std::vector<float>& remainder
    ) {
        quotient.clear();
        remainder = dividend;
        
        int dividendDegree = dividend.size() - 1;
        int divisorDegree = divisor.size() - 1;
        
        if (divisorDegree < 0) {
            // Division by zero polynomial
            return;
        }
        
        while (remainder.size() >= divisor.size()) {
            float leadCoeff = remainder.back() / divisor.back();
            int degreeOffset = remainder.size() - divisor.size();
            
            quotient.insert(quotient.begin(), leadCoeff);
            
            for (size_t i = 0; i < divisor.size(); ++i) {
                remainder[degreeOffset + i] -= leadCoeff * divisor[i];
            }
            
            // Remove leading zeros
            while (!remainder.empty() && std::abs(remainder.back()) < EPSILON) {
                remainder.pop_back();
            }
        }
        
        std::reverse(quotient.begin(), quotient.end());
    }

    // Compute reconstruction error
    static float computeReconstructionError(
        const std::vector<std::vector<float>>& originalPolys,
        const std::vector<float>& denominator,
        const std::vector<std::vector<float>>& simplifiedPolys
    ) {
        float totalError = 0.0f;
        
        for (size_t polyIdx = 0; polyIdx < originalPolys.size(); ++polyIdx) {
            // Reconstruct and compare
            std::vector<float> reconstructed = multiplyPolynomials(denominator, simplifiedPolys[polyIdx]);
            
            float polyError = 0.0f;
            for (size_t i = 0; i < originalPolys[polyIdx].size(); ++i) {
                polyError += std::abs(originalPolys[polyIdx][i] - reconstructed[i]);
            }
            
            totalError += polyError;
        }
        
        return totalError / originalPolys.size();
    }

    // Multiply two polynomials
    static std::vector<float> multiplyPolynomials(
        const std::vector<float>& a, 
        const std::vector<float>& b
    ) {
        std::vector<float> result(a.size() + b.size() - 1, 0.0f);
        
        for (size_t i = 0; i < a.size(); ++i) {
            for (size_t j = 0; j < b.size(); ++j) {
                result[i+j] += a[i] * b[j];
            }
        }
        
        return result;
    }

    // Find optimal factorization
    static CompressedPolynomialSegment findOptimalFactorization(
        const PolynomialSegment& segment
    ) {
        CompressedPolynomialSegment bestCompression;
        float bestCompressionRatio = std::numeric_limits<float>::max();
        
        // Convert to vector for easier manipulation
        std::vector<std::vector<float>> polys;
        for (int p = 0; p < POLY_COUNT; ++p) {
            std::vector<float> poly(
                segment.coefficients[p], 
                segment.coefficients[p] + POLY_DEGREE + 1
            );
            polys.push_back(poly);
        }
        
        // Multiple factorization attempts
        for (int attempt = 0; attempt < MAX_FACTORIZATION_ATTEMPTS; ++attempt) {
            CompressedPolynomialSegment currentCompression;
            
            // Strategy variations
            switch (attempt) {
                case 0: // Standard GCD approach
                    {
                        auto denominator = polys[0];
                        for (size_t i = 1; i < polys.size(); ++i) {
                            denominator = polynomialGCD(denominator, polys[i]);
                        }
                        
                        // Copy denominator coefficients
                        std::copy(
                            denominator.begin(), 
                            denominator.end(), 
                            currentCompression.denominatorCoeffs
                        );
                        
                        // Simplify polynomials
                        for (int p = 0; p < POLY_COUNT; ++p) {
                            std::vector<float> simplified;
                            std::vector<float> quotient;
                            polynomialDivision(polys[p], denominator, quotient, simplified);
                            
                            std::copy(
                                quotient.begin(), 
                                quotient.end(), 
                                currentCompression.simplifiedCoeffs[p]
                            );
                        }
                    }
                    break;
                
                // Additional factorization strategies can be added here
                // ... (e.g., different GCD variants, normalization approaches)
            }
            
            // Compute compression metrics
            float reconstructionError = computeReconstructionError(
                polys, 
                std::vector<float>(
                    currentCompression.denominatorCoeffs, 
                    currentCompression.denominatorCoeffs + POLY_DEGREE + 1
                ),
                std::vector<std::vector<float>>(
                    currentCompression.simplifiedCoeffs, 
                    currentCompression.simplifiedCoeffs + POLY_COUNT
                )
            );
            
            // Evaluate compression quality
            float compressionRatio = reconstructionError;
            
            if (compressionRatio < bestCompressionRatio) {
                bestCompressionRatio = compressionRatio;
                bestCompression = currentCompression;
                bestCompression.compressionRatio = compressionRatio;
                bestCompression.reconstructionError = reconstructionError;
            }
        }
        
        // Copy time deltas
        bestCompression.timeDeltas = segment.timeDeltas[0];
        
        return bestCompression;
    }

public:
    // Compress polynomial segment
    static CompressedPolynomialSegment compressSegment(const PolynomialSegment& segment) {
        return findOptimalFactorization(segment);
    }

    // Decompress segment
    static PolynomialSegment decompressSegment(const CompressedPolynomialSegment& compressed) {
        PolynomialSegment decompressed;
        
        // Reconstruct polynomials
        std::vector<float> denominator(
            compressed.denominatorCoeffs, 
            compressed.denominatorCoeffs + POLY_DEGREE + 1
        );
        
        for (int p = 0; p < POLY_COUNT; ++p) {
            std::vector<float> simplifiedPoly(
                compressed.simplifiedCoeffs[p], 
                compressed.simplifiedCoeffs[p] + POLY_DEGREE + 1
            );
            
            // Multiply simplified poly with denominator
            std::vector<float> reconstructedPoly = multiplyPolynomials(denominator, simplifiedPoly);
            
            // Copy back to original structure
            std::copy(
                reconstructedPoly.begin(), 
                reconstructedPoly.end(), 
                decompressed.coefficients[p]
            );
        }
        
        // Restore time deltas
        for (int i = 0; i < POLY_COUNT; ++i) {
            decompressed.timeDeltas[i] = compressed.timeDeltas;
        }
        
        return decompressed;
    }
};

// RTC memory storage
RTC_DATA_ATTR CompressedPolynomialSegment rtcSegmentBuffer[SEGMENTS];
RTC_DATA_ATTR uint8_t rtcSegmentCount = 0;

void storePolynomialSegment(const PolynomialSegment& segment) {
    if (rtcSegmentCount < SEGMENTS) {
        rtcSegmentBuffer[rtcSegmentCount++] = 
            AdvancedPolynomialCompressor::compressSegment(segment);
    }
}

void setup() {
    Serial.begin(115200);
    
    // Example usage
    PolynomialSegment exampleSegment = {
        .coefficients = {
            {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},   // Polynomial 1
            {2.0, 4.0, 6.0, 8.0, 10.0, 12.0}, // Polynomial 2
            // ... other polynomials
        },
        .timeDeltas = {100, 100, 100, 100, 100}
    };

    storePolynomialSegment(exampleSegment);
}

void loop() {
    // Main application logic
}
