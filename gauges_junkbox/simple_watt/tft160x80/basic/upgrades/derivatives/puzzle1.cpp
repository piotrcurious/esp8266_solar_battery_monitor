#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <algorithm>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class AdvancedKalmanADC {
private:
    // Expanded Chebyshev fitting parameters
    static const int CHEB_ORDER = 7;  // Increased from 5 to 7 for more complex fitting
    static constexpr float CHEB_SCALE_FACTOR = 1.1;  // Additional scaling factor

    // Improved Chebyshev polynomial calculation
    float chebyshevPolynomial(int n, float x) {
        switch(n) {
            case 0: return 1.0;
            case 1: return x;
            case 2: return 2.0 * x * x - 1.0;
            case 3: return x * (4.0 * x * x - 3.0);
            case 4: return 8.0 * x * x * x * x - 8.0 * x * x + 1.0;
            default: {
                float a = x, b = 1.0, c;
                for (int i = 2; i <= n; i++) {
                    c = 2.0 * x * a - b;
                    b = a;
                    a = c;
                }
                return a;
            }
        }
    }

    // Advanced mapping function to handle non-linear scaling
    float mapToChebDomain(float t, float tMax) {
        // Hyperbolic tangent mapping for better non-linear scaling
        float scaledT = std::tanh(t / (0.5 * tMax)) * 2.0;
        return scaledT * CHEB_SCALE_FACTOR;
    }

    // Improved weighted least squares Chebyshev fitting
    float fitChebyshevLeastSquares(const float* samples, int count, float deltaT) {
        float tMax = deltaT * (count - 1);
        
        // Enhanced robustness with median and adaptive scaling
        float medianSample = [&]() {
            std::vector<float> sortedSamples(samples, samples + count);
            std::nth_element(sortedSamples.begin(), 
                             sortedSamples.begin() + count/2, 
                             sortedSamples.end());
            return sortedSamples[count/2];
        }();

        // Adaptive scaling and weighting
        float sampleSpread = *std::max_element(samples, samples + count) - 
                             *std::min_element(samples, samples + count);
        
        // Eigen matrices for least squares
        MatrixXd A(count, CHEB_ORDER + 1);
        VectorXd b(count);
        VectorXd weights(count);

        // Advanced weighting strategy
        for (int i = 0; i < count; i++) {
            float t = i * deltaT;
            float x = mapToChebDomain(t, tMax);
            
            // Adaptive weight based on distance from median and sample spread
            float weight = 1.0 / (1.0 + std::abs(samples[i] - medianSample) / sampleSpread);
            weights(i) = std::pow(weight, 1.5);  // Non-linear weight enhancement
            
            // Build Chebyshev basis matrix
            for (int j = 0; j <= CHEB_ORDER; j++) {
                A(i, j) = chebyshevPolynomial(j, x) * weights(i);
            }
            b(i) = samples[i] * weights(i);
        }

        // Solve weighted least squares with regularization
        Eigen::MatrixXd regularization = Eigen::MatrixXd::Identity(CHEB_ORDER + 1, CHEB_ORDER + 1);
        regularization(0, 0) = 0.0;  // Don't regularize DC component
        VectorXd coeffs = (A.transpose() * A + 0.01 * regularization).ldlt().solve(A.transpose() * b);

        // Store coefficients for later use
        for (int i = 0; i <= CHEB_ORDER; i++) {
            chebCoeffs[i] = coeffs(i);
        }

        return coeffs(0);  // Return DC component
    }

    // Enhanced prediction with more sophisticated blending
    float predictWithChebyshev(float t, float tMax, float minSample, float maxSample) {
        float x = mapToChebDomain(t, tMax);
        
        // Full Chebyshev polynomial reconstruction
        float value = chebCoeffs[0];
        for (int i = 1; i <= CHEB_ORDER; i++) {
            value += chebCoeffs[i] * chebyshevPolynomial(i, x);
        }

        // More sophisticated blending with theoretical model
        float theoretical = theoreticalRCResponse(t, minSample, maxSample);
        
        // Adaptive blending based on fit quality
        float blendFactor = calculateBlendingFactor();
        
        return (1.0f - blendFactor) * value + blendFactor * theoretical;
    }

    // New method to calculate dynamic blending factor
    float calculateBlendingFactor() {
        // Placeholder for a more sophisticated blending calculation
        // This would involve analyzing the coefficients, 
        // prediction error, and other system characteristics
        return 0.3f;  // Can be dynamically adjusted
    }

public:
    // Rest of the class remains the same...
};
