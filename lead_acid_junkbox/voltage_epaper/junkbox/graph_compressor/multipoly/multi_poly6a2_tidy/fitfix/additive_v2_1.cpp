#include <vector>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <algorithm>

class AdvancedPolynomialFitter {
public:
    // Enum for optimization method (kept for compatibility)
    enum OptimizationMethod {
        LEAST_SQUARES,
        RIDGE_REGRESSION,
        // Add more methods as needed
    };

    /**
     * Fit a polynomial using superposition principle and orthogonal decomposition
     * @param x Input x values
     * @param y Input y values
     * @param degree Polynomial degree
     * @param method Optimization method
     * @return Polynomial coefficients
     */
    static std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        OptimizationMethod method = LEAST_SQUARES
    ) {
        // Input validation
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        // Normalize input to improve numerical stability
        auto [normalizedX, xScale] = normalizeInput(x);
        
        // Use Gram-Schmidt orthogonalization for stable basis decomposition
        auto orthogonalBasis = computeOrthogonalBasis(normalizedX, degree);
        
        // Compute coefficients using inner product projection
        std::vector<double> coeffs = computeCoefficients(orthogonalBasis, y);
        
        // Rescale coefficients to account for input normalization
        rescaleCoefficients(coeffs, xScale, degree);

        // Convert to float and return
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }

private:
    /**
     * Normalize input to improve numerical stability
     * @return Pair of normalized x values and scale factor
     */
    static std::pair<std::vector<double>, double> normalizeInput(const std::vector<float>& x) {
        if (x.empty()) return {{}, 1.0};

        // Find min and max to create a [-1, 1] scaling
        auto [minIt, maxIt] = std::minmax_element(x.begin(), x.end());
        double xMin = *minIt, xMax = *maxIt;
        double xScale = std::max(std::abs(xMin), std::abs(xMax));
        
        std::vector<double> normalizedX;
        normalizedX.reserve(x.size());
        
        // Normalize to [-1, 1] range
        for (float xi : x) {
            normalizedX.push_back(xi / xScale);
        }

        return {normalizedX, xScale};
    }

    /**
     * Compute orthogonal basis using Gram-Schmidt process
     * @param x Normalized input values
     * @param degree Polynomial degree
     * @return Orthogonal basis vectors
     */
    static std::vector<std::vector<double>> computeOrthogonalBasis(
        const std::vector<double>& x, 
        int degree
    ) {
        std::vector<std::vector<double>> basis(degree + 1);
        
        // First basis vector is constant
        basis[0] = std::vector<double>(x.size(), 1.0 / std::sqrt(x.size()));
        
        for (int k = 1; k <= degree; ++k) {
            basis[k] = std::vector<double>(x.size());
            
            // Compute basis vector using Gram-Schmidt
            for (size_t i = 0; i < x.size(); ++i) {
                basis[k][i] = std::pow(x[i], k);
            }
            
            // Orthogonalize against previous basis vectors
            for (int j = 0; j < k; ++j) {
                double proj = computeInnerProduct(basis[k], basis[j]);
                
                for (size_t i = 0; i < x.size(); ++i) {
                    basis[k][i] -= proj * basis[j][i];
                }
            }
            
            // Normalize the vector
            double norm = std::sqrt(computeInnerProduct(basis[k], basis[k]));
            for (size_t i = 0; i < x.size(); ++i) {
                basis[k][i] /= norm;
            }
        }
        
        return basis;
    }

    /**
     * Compute coefficients using inner product projection
     * @param orthogonalBasis Orthogonal basis vectors
     * @param y Input y values
     * @return Coefficient vector
     */
    static std::vector<double> computeCoefficients(
        const std::vector<std::vector<double>>& orthogonalBasis,
        const std::vector<float>& y
    ) {
        std::vector<double> coeffs;
        coeffs.reserve(orthogonalBasis.size());
        
        for (const auto& basisVector : orthogonalBasis) {
            // Project y onto the basis vector
            double coeff = computeInnerProduct(y, basisVector);
            coeffs.push_back(coeff);
        }
        
        return coeffs;
    }

    /**
     * Rescale coefficients to account for input normalization
     * @param coeffs Coefficient vector
     * @param xScale Input scaling factor
     * @param degree Polynomial degree
     */
    static void rescaleCoefficients(
        std::vector<double>& coeffs, 
        double xScale, 
        int degree
    ) {
        for (int k = 1; k < static_cast<int>(coeffs.size()); ++k) {
            coeffs[k] /= std::pow(xScale, k);
        }
    }

    /**
     * Compute inner product of two vectors
     * @param v1 First vector
     * @param v2 Second vector
     * @return Inner product
     */
    template<typename T, typename U>
    static double computeInnerProduct(
        const std::vector<T>& v1, 
        const std::vector<U>& v2
    ) {
        if (v1.size() != v2.size()) {
            throw std::invalid_argument("Vectors must have the same size");
        }
        
        return std::inner_product(
            v1.begin(), v1.end(), 
            v2.begin(), 0.0
        );
    }
};
