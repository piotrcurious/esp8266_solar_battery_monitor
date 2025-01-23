#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

class AdvancedPolynomialFitter {
public:
    enum class OptimizationMethod {
        LEAST_SQUARES,
        REGULARIZED,
        // Add more methods as needed
    };

    // Improved polynomial fitting using superposition principle
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree, 
                                     OptimizationMethod method = OptimizationMethod::LEAST_SQUARES) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        // Normalize input data to improve numerical stability
        auto [normalizedX, xOffset, xScale] = normalizeData(x);
        auto [normalizedY, yOffset, yScale] = normalizeData(y);

        // Compute basis functions using additive state decomposition
        std::vector<std::vector<double>> basisFunctions = computeBasisFunctions(normalizedX, degree);

        // Use pseudoinverse method for robust coefficient estimation
        std::vector<double> coeffs = computePseudoinverse(basisFunctions, normalizedY, method);

        // Denormalize coefficients
        return denormalizeCoefficients(coeffs, xOffset, xScale, yOffset, yScale);
    }

private:
    // Normalize data to improve numerical conditioning
    template<typename T>
    std::tuple<std::vector<double>, double, double> normalizeData(const std::vector<T>& data) {
        if (data.empty()) return {{}, 0.0, 1.0};

        auto [minIt, maxIt] = std::minmax_element(data.begin(), data.end());
        double minVal = *minIt, maxVal = *maxIt;
        double offset = (minVal + maxVal) / 2.0;
        double scale = (maxVal - minVal) != 0 ? 2.0 / (maxVal - minVal) : 1.0;

        std::vector<double> normalizedData;
        normalizedData.reserve(data.size());
        
        std::transform(data.begin(), data.end(), std::back_inserter(normalizedData), 
            [&](T val) { return (static_cast<double>(val) - offset) * scale; });

        return {normalizedData, offset, scale};
    }

    // Compute orthogonal basis functions using Gram-Schmidt process
    std::vector<std::vector<double>> computeBasisFunctions(const std::vector<double>& x, int degree) {
        std::vector<std::vector<double>> basisFunctions(degree + 1, std::vector<double>(x.size()));
        
        // First basis function is always constant
        std::fill(basisFunctions[0].begin(), basisFunctions[0].end(), 1.0 / std::sqrt(x.size()));

        for (int k = 1; k <= degree; ++k) {
            // Compute basis function using orthogonalization
            std::vector<double> currentBasis(x.size());
            for (size_t i = 0; i < x.size(); ++i) {
                currentBasis[i] = std::pow(x[i], k);
            }

            // Orthogonalize against previous basis functions
            for (int j = 0; j < k; ++j) {
                // Compute projection
                double projectionCoeff = computeInnerProduct(currentBasis, basisFunctions[j]);
                
                // Subtract projection
                for (size_t i = 0; i < x.size(); ++i) {
                    currentBasis[i] -= projectionCoeff * basisFunctions[j][i];
                }
            }

            // Normalize the basis function
            double norm = std::sqrt(computeInnerProduct(currentBasis, currentBasis));
            if (norm > 1e-10) {
                for (size_t i = 0; i < x.size(); ++i) {
                    basisFunctions[k][i] = currentBasis[i] / norm;
                }
            }
        }

        return basisFunctions;
    }

    // Compute inner product between two vectors
    double computeInnerProduct(const std::vector<double>& a, const std::vector<double>& b) {
        return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
    }

    // Compute pseudoinverse using different methods
    std::vector<double> computePseudoinverse(const std::vector<std::vector<double>>& basisFunctions, 
                                             const std::vector<double>& y, 
                                             OptimizationMethod method) {
        size_t m = basisFunctions.size();
        std::vector<double> coeffs(m, 0.0);

        // Compute coefficients using projection method
        for (size_t k = 0; k < m; ++k) {
            coeffs[k] = computeInnerProduct(y, basisFunctions[k]);
        }

        // Apply regularization if needed
        if (method == OptimizationMethod::REGULARIZED) {
            applyTikhanovRegularization(coeffs);
        }

        return coeffs;
    }

    // Simple Tikhonov regularization to prevent overfitting
    void applyTikhanovRegularization(std::vector<double>& coeffs, double lambda = 1e-6) {
        for (size_t k = 1; k < coeffs.size(); ++k) {
            coeffs[k] /= (1.0 + lambda * k * k);
        }
    }

    // Denormalize coefficients back to original scale
    std::vector<float> denormalizeCoefficients(const std::vector<double>& coeffs, 
                                               double xOffset, double xScale, 
                                               double yOffset, double yScale) {
        std::vector<double> denormalized(coeffs.size());
        denormalized[0] = coeffs[0] * yScale + yOffset;

        for (size_t k = 1; k < coeffs.size(); ++k) {
            double scaleFactor = yScale / std::pow(xScale, k);
            denormalized[k] = coeffs[k] * scaleFactor;
        }

        // Convert to float
        return std::vector<float>(denormalized.begin(), denormalized.end());
    }
};
