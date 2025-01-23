#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

class LinearSuperpositionPolynomialFitter {
public:
    /**
     * Polynomial fitting using linear system superposition principle
     * @param x Input x values
     * @param y Input y values
     * @param degree Polynomial degree
     * @return Polynomial coefficients
     */
    static std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        // Decompose the problem into individual basis contributions
        std::vector<std::vector<float>> basisContributions = decomposeBasisContributions(x, y, degree);
        
        // Apply superposition principle to reconstruct coefficients
        return reconstructCoefficients(basisContributions);
    }

private:
    /**
     * Decompose the problem into individual basis function contributions
     */
    static std::vector<std::vector<float>> decomposeBasisContributions(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<std::vector<float>> basisContributions(degree + 1);
        
        for (size_t k = 0; k <= static_cast<size_t>(degree); ++k) {
            // Individual basis function contribution
            basisContributions[k] = computeBasisContribution(x, y, k);
        }
        
        return basisContributions;
    }

    /**
     * Compute contribution of a single basis function
     */
    static std::vector<float> computeBasisContribution(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        size_t power
    ) {
        std::vector<float> contribution(x.size());
        
        // Compute basis function values
        std::vector<float> basisFunction(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            basisFunction[i] = std::pow(x[i], power);
        }
        
        // Compute projection of y onto basis function
        float numerator = 0.0f;
        float denominator = 0.0f;
        
        for (size_t i = 0; i < x.size(); ++i) {
            numerator += y[i] * basisFunction[i];
            denominator += basisFunction[i] * basisFunction[i];
        }
        
        // Coefficient for this basis function
        float coefficient = (denominator != 0.0f) ? numerator / denominator : 0.0f;
        
        // Compute individual point contributions
        for (size_t i = 0; i < x.size(); ++i) {
            contribution[i] = coefficient * basisFunction[i];
        }
        
        return contribution;
    }

    /**
     * Reconstruct coefficients by summing individual basis contributions
     */
    static std::vector<float> reconstructCoefficients(
        const std::vector<std::vector<float>>& basisContributions
    ) {
        std::vector<float> coefficients(basisContributions.size(), 0.0f);
        
        // Apply superposition principle
        for (size_t k = 0; k < basisContributions.size(); ++k) {
            // Sum of individual point contributions
            coefficients[k] = std::accumulate(
                basisContributions[k].begin(), 
                basisContributions[k].end(), 
                0.0f
            ) / basisContributions[k].size();
        }
        
        return coefficients;
    }

    /**
     * Optional: Verify superposition property
     */
    static bool verifySuperpositionProperty(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        const std::vector<float>& coefficients
    ) {
        // Reconstruct the original signal by summing basis functions
        std::vector<float> reconstructedY(y.size(), 0.0f);
        
        for (size_t k = 0; k < coefficients.size(); ++k) {
            for (size_t i = 0; i < x.size(); ++i) {
                reconstructedY[i] += coefficients[k] * std::pow(x[i], k);
            }
        }
        
        // Compare reconstruction with original signal
        float totalError = 0.0f;
        for (size_t i = 0; i < y.size(); ++i) {
            totalError += std::abs(reconstructedY[i] - y[i]);
        }
        
        return totalError / y.size() < 1e-6;
    }
};
