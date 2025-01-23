#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>

class AdaptivePolynomialFitter {
public:
    enum RegressionMethod {
        SEQUENTIAL_ORTHOGONAL, 
        RECURSIVE_LEAST_SQUARES,
        KERNEL_REGRESSION
    };

    /**
     * Adaptive polynomial fitting with causal decomposition
     * @param x Input x values (must be monotonically increasing)
     * @param y Input y values
     * @param degree Polynomial degree
     * @param method Regression method
     * @return Polynomial coefficients
     */
    static std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        RegressionMethod method = RECURSIVE_LEAST_SQUARES
    ) {
        // Validate input
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        // Ensure monotonically increasing x values
        if (!isMonotonicallyIncreasing(x)) {
            throw std::invalid_argument("Input x values must be monotonically increasing");
        }

        switch (method) {
            case SEQUENTIAL_ORTHOGONAL:
                return sequentialOrthogonalRegression(x, y, degree);
            case RECURSIVE_LEAST_SQUARES:
                return recursiveLeastSquares(x, y, degree);
            case KERNEL_REGRESSION:
                return kernelBasedRegression(x, y, degree);
            default:
                throw std::invalid_argument("Unsupported regression method");
        }
    }

private:
    /**
     * Check if vector is monotonically increasing
     */
    static bool isMonotonicallyIncreasing(const std::vector<float>& x) {
        return std::is_sorted(x.begin(), x.end());
    }

    /**
     * Sequential Orthogonal Regression with Causal Projection
     */
    static std::vector<float> sequentialOrthogonalRegression(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<double> coeffs(degree + 1, 0.0);
        std::vector<double> residuals(y.begin(), y.end());
        
        for (int k = 0; k <= degree; ++k) {
            // Compute basis functions with causal constraints
            std::vector<double> basisFunction = computeCausalBasisFunction(x, k);
            
            // Compute coefficient with minimal past interference
            double coefficient = computeCausalCoefficient(basisFunction, residuals);
            coeffs[k] = coefficient;
            
            // Update residuals, removing the contribution of current basis
            for (size_t i = 0; i < residuals.size(); ++i) {
                residuals[i] -= coefficient * basisFunction[i];
            }
        }
        
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }

    /**
     * Compute Causal Basis Function with Temporal Locality
     */
    static std::vector<double> computeCausalBasisFunction(
        const std::vector<float>& x, 
        int power
    ) {
        std::vector<double> basis(x.size());
        
        // Normalize x to manage numerical stability
        double xMin = x.front(), xMax = x.back();
        double xScale = xMax - xMin;
        
        for (size_t i = 0; i < x.size(); ++i) {
            // Normalized input with temporal weighting
            double normalizedX = (x[i] - xMin) / xScale;
            
            // Causal basis function with exponential decay
            basis[i] = std::pow(normalizedX, power) * 
                       std::exp(-0.5 * (x.size() - i) / x.size());
        }
        
        return basis;
    }

    /**
     * Compute Causal Coefficient with Minimal Interference
     */
    static double computeCausalCoefficient(
        const std::vector<double>& basisFunction,
        const std::vector<double>& residuals
    ) {
        // Compute weighted inner product
        double numerator = 0.0;
        double denominator = 0.0;
        
        for (size_t i = 0; i < basisFunction.size(); ++i) {
            // Apply causality-preserving weights
            double weight = 1.0 / (i + 1);
            numerator += weight * basisFunction[i] * residuals[i];
            denominator += weight * basisFunction[i] * basisFunction[i];
        }
        
        return (denominator != 0.0) ? numerator / denominator : 0.0;
    }

    /**
     * Recursive Least Squares with Adaptive Forgetting Factor
     */
    static std::vector<float> recursiveLeastSquares(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<double> coeffs(degree + 1, 0.0);
        
        // Initial covariance matrix (diagonal loading)
        std::vector<std::vector<double>> P(degree + 1, 
            std::vector<double>(degree + 1, 1e-4));
        
        for (size_t i = 0; i < x.size(); ++i) {
            // Construct regression vector
            std::vector<double> phi(degree + 1);
            for (int j = 0; j <= degree; ++j) {
                phi[j] = std::pow(x[i], j);
            }
            
            // Adaptive forgetting factor
            double lambda = 1.0 - std::exp(-i / static_cast<double>(x.size()));
            
            // Recursive update
            std::vector<double> k(degree + 1);
            for (int r = 0; r <= degree; ++r) {
                double kNum = 0.0, kDenom = lambda;
                for (int c = 0; c <= degree; ++c) {
                    kNum += P[r][c] * phi[c];
                    kDenom += phi[c] * P[c][r];
                }
                k[r] = kNum / kDenom;
            }
            
            // Update coefficients
            double error = y[i] - std::inner_product(
                phi.begin(), phi.end(), coeffs.begin(), 0.0);
            
            for (int j = 0; j <= degree; ++j) {
                coeffs[j] += k[j] * error;
            }
            
            // Update covariance matrix
            for (int r = 0; r <= degree; ++r) {
                for (int c = 0; c <= degree; ++c) {
                    P[r][c] = (P[r][c] - k[r] * phi[c]) / lambda;
                }
            }
        }
        
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }

    /**
     * Kernel-based Regression with Temporal Locality
     */
    static std::vector<float> kernelBasedRegression(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        std::vector<double> coeffs(degree + 1, 0.0);
        
        for (int k = 0; k <= degree; ++k) {
            double numerator = 0.0, denominator = 0.0;
            
            for (size_t i = 0; i < x.size(); ++i) {
                // Temporal kernel with exponential decay
                double kernel = std::exp(-std::pow(i - x.size() / 2.0, 2) / 
                                         (2.0 * std::pow(x.size() / 4.0, 2)));
                
                numerator += kernel * y[i] * std::pow(x[i], k);
                denominator += kernel * std::pow(x[i], 2*k);
            }
            
            coeffs[k] = (denominator != 0.0) ? numerator / denominator : 0.0;
        }
        
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }
};
