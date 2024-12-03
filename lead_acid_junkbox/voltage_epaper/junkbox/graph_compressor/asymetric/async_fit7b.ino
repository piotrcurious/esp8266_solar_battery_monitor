#include <cmath>
#include <algorithm>
#include <array>
#include <limits>

// Constants
#define FIELD_ORDER 256       // Corresponds to 8-bit finite field
#define MAX_ITER 100
#define TOLERANCE 1e-4
#define MAX_CANDIDATES 10
#define DIMENSION 5           // Polynomial coefficients: a4, a3, a2, a1, a0
#define TIME_QUANTIZATION_SCALE 1000  // Scale factor for timestamp quantization
#define OUT_OF_BOUNDS_PENALTY 10000.0 // Penalty for predictions exceeding bounds

// Modular arithmetic for finite field operations
inline int mod(int value, int modBase) {
    return (value % modBase + modBase) % modBase;
}

// Addition in finite field
inline int fieldAdd(int a, int b) {
    return mod(a + b, FIELD_ORDER);
}

// Multiplication in finite field
inline int fieldMultiply(int a, int b) {
    return mod(a * b, FIELD_ORDER);
}

// Lifted Polynomial Representation
struct PolynomialCandidate {
    int coeffs[DIMENSION];  // Coefficients a4...a0 in quantized field
    float error;            // Residual error in fit
};

// Evaluate polynomial at given timestamp
float evaluatePolynomial(const PolynomialCandidate& poly, float t) {
    float result = 0.0;
    for (int i = 0; i < DIMENSION; i++) {
        result += poly.coeffs[i] * std::pow(t, DIMENSION - i - 1);
    }
    return result;
}

// Quantize timestamps to canonical range
float quantizeTimestamp(float timestamp, float startTime) {
    return (timestamp - startTime) / TIME_QUANTIZATION_SCALE;
}

// Compute kernel-based residual error considering timestamps and bounds
float computeKernelResidual(float data[][2], float startTime, int count, const PolynomialCandidate& candidate) {
    float error = 0.0;

    for (int i = 0; i < count; i++) {
        float quantizedTime = quantizeTimestamp(data[i][0], startTime);
        float predicted = evaluatePolynomial(candidate, quantizedTime);
        float residual = data[i][1] - predicted;

        // Penalize out-of-bounds predictions
        float lowerBound = data[i][1] - 1e-6;  // Include small tolerance
        float upperBound = data[i][1] + 1e-6;
        if (predicted < lowerBound || predicted > upperBound) {
            error += OUT_OF_BOUNDS_PENALTY;  // Add penalty for exceeding bounds
        }

        error += residual * residual;  // Add squared residual error
    }

    return error;
}

// Generate candidate polynomials using kernel mapping
void generateKernelCandidates(float data[][2], float startTime, int count, PolynomialCandidate candidates[MAX_CANDIDATES], int& candidateCount) {
    for (int i = 0; i < MAX_CANDIDATES; i++) {
        PolynomialCandidate candidate;
        for (int j = 0; j < DIMENSION; j++) {
            candidate.coeffs[j] = rand() % FIELD_ORDER;  // Random initialization within finite field
        }
        candidate.error = computeKernelResidual(data, startTime, count, candidate);
        candidates[i] = candidate;
    }
    std::sort(candidates, candidates + MAX_CANDIDATES, [](const PolynomialCandidate& a, const PolynomialCandidate& b) {
        return a.error < b.error;
    });
    candidateCount = MAX_CANDIDATES;
}

// Refine candidates using cohomological structure
void refineCandidates(float data[][2], float startTime, int count, PolynomialCandidate& candidate) {
    for (int iter = 0; iter < MAX_ITER; iter++) {
        PolynomialCandidate refined = candidate;

        // Perturb coefficients to explore cohomological sections
        for (int i = 0; i < DIMENSION; i++) {
            refined.coeffs[i] = fieldAdd(candidate.coeffs[i], rand() % 3 - 1);  // Local refinement in field
        }

        refined.error = computeKernelResidual(data, startTime, count, refined);
        if (refined.error < candidate.error) {
            candidate = refined;
        }

        if (std::abs(candidate.error) < TOLERANCE) break;
    }
}

// Hybrid Fitting Strategy with Bounds Integration
void fitWithBoundsIntegration(float data[][2], int count, int bestCoeffs[DIMENSION]) {
    float startTime = data[0][0];  // Normalize timestamps relative to the start
    PolynomialCandidate candidates[MAX_CANDIDATES];
    int candidateCount = 0;

    // Generate kernel-based candidates
    generateKernelCandidates(data, startTime, count, candidates, candidateCount);

    // Refine candidates
    for (int i = 0; i < candidateCount; i++) {
        refineCandidates(data, startTime, count, candidates[i]);
    }

    // Select best candidate
    PolynomialCandidate bestCandidate = candidates[0];
    for (int i = 1; i < candidateCount; i++) {
        if (candidates[i].error < bestCandidate.error) {
            bestCandidate = candidates[i];
        }
    }

    // Copy best coefficients
    for (int i = 0; i < DIMENSION; i++) {
        bestCoeffs[i] = bestCandidate.coeffs[i];
    }
}
