#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

class AdvancedPolynomialFitter {
public:
    enum class OptimizationMethod {
        NORMAL_EQUATION,
        CHAMBOLLE_POCK,
        ADAPTIVE_REGULARIZATION
    };

    /**
     * Fit a polynomial using an advanced iterative approach
     * @param x Input x-coordinates
     * @param y Input y-coordinates
     * @param degree Polynomial degree
     * @param method Optimization method to use
     * @param max_iterations Maximum number of refinement iterations
     * @return Polynomial coefficients
     */
    std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree, 
        OptimizationMethod method = OptimizationMethod::CHAMBOLLE_POCK,
        size_t max_iterations = 50
    ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Initial fit using normal equation
        std::vector<float> coeffs = initialNormalEquationFit(x, y, degree);

        // Iterative refinement based on selected method
        switch (method) {
            case OptimizationMethod::CHAMBOLLE_POCK:
                return chambolle_pock_refinement(x, y, coeffs, max_iterations);
            
            case OptimizationMethod::ADAPTIVE_REGULARIZATION:
                return adaptive_regularization(x, y, coeffs, max_iterations);
            
            default:
                return coeffs;
        }
    }

private:
    // Compute polynomial value at a point
    float evaluatePolynomial(const std::vector<float>& coeffs, float x) {
        float result = 0.0f;
        float x_power = 1.0f;
        for (float coeff : coeffs) {
            result += coeff * x_power;
            x_power *= x;
        }
        return result;
    }

    // Initial fit using normal equation
    std::vector<float> initialNormalEquationFit(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        size_t n = x.size();
        size_t m = degree + 1;

        // Construct Vandermonde matrix
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x[i];
            }
        }

        // Construct normal equation: (A^T * A) * coeffs = A^T * y
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += A[i][j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        // Solve using Gaussian elimination
        std::vector<double> coeffs_double = solveLinearSystem(ATA, ATy);
        
        // Convert to float
        return std::vector<float>(coeffs_double.begin(), coeffs_double.end());
    }

    // Chambolle-Pock algorithm for iterative refinement
    std::vector<float> chambolle_pock_refinement(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        const std::vector<float>& initial_coeffs,
        size_t max_iterations
    ) {
        std::vector<float> coeffs = initial_coeffs;
        std::vector<float> dual_vars(x.size(), 0.0f);
        
        // Hyperparameters
        float tau = 0.1f;     // Primal step size
        float sigma = 0.1f;   // Dual step size
        float lambda = 0.01f; // Regularization strength

        for (size_t iter = 0; iter < max_iterations; ++iter) {
            // Compute error polynomial
            std::vector<float> error_poly(x.size());
            float total_error = 0.0f;
            
            for (size_t i = 0; i < x.size(); ++i) {
                float predicted = evaluatePolynomial(coeffs, x[i]);
                error_poly[i] = predicted - y[i];
                total_error += std::abs(error_poly[i]);
            }

            // Compute gradient of the error
            std::vector<float> gradient(coeffs.size(), 0.0f);
            for (size_t j = 0; j < coeffs.size(); ++j) {
                for (size_t i = 0; i < x.size(); ++i) {
                    float x_power = std::pow(x[i], j);
                    gradient[j] += error_poly[i] * x_power;
                }
                
                // L1 regularization
                gradient[j] += lambda * (coeffs[j] > 0 ? 1 : -1);
            }

            // Update coefficients (Chambolle-Pock primal-dual update)
            std::vector<float> new_coeffs = coeffs;
            for (size_t j = 0; j < coeffs.size(); ++j) {
                new_coeffs[j] -= tau * gradient[j];
            }

            // Proximal operator (soft-thresholding)
            for (size_t j = 0; j < coeffs.size(); ++j) {
                if (std::abs(new_coeffs[j]) < lambda * tau) {
                    new_coeffs[j] = 0.0f;
                } else {
                    new_coeffs[j] -= lambda * tau * (new_coeffs[j] > 0 ? 1 : -1);
                }
            }

            // Update step
            coeffs = new_coeffs;

            // Convergence check
            if (total_error < 1e-6) break;
        }

        return coeffs;
    }

    // Adaptive regularization method
    std::vector<float> adaptive_regularization(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        const std::vector<float>& initial_coeffs,
        size_t max_iterations
    ) {
        std::vector<float> coeffs = initial_coeffs;
        
        // Adaptive regularization parameters
        float base_lambda = 0.01f;
        float decay_rate = 0.95f;

        for (size_t iter = 0; iter < max_iterations; ++iter) {
            // Compute current error
            float total_error = 0.0f;
            std::vector<float> error_poly(x.size());
            
            for (size_t i = 0; i < x.size(); ++i) {
                float predicted = evaluatePolynomial(coeffs, x[i]);
                error_poly[i] = predicted - y[i];
                total_error += std::abs(error_poly[i]);
            }

            // Adaptive regularization strength
            float lambda = base_lambda * std::pow(decay_rate, iter);

            // Compute and apply correction
            std::vector<float> correction(coeffs.size(), 0.0f);
            for (size_t j = 0; j < coeffs.size(); ++j) {
                for (size_t i = 0; i < x.size(); ++i) {
                    float x_power = std::pow(x[i], j);
                    correction[j] += error_poly[i] * x_power;
                }
                
                // Regularization term
                correction[j] += lambda * (coeffs[j] > 0 ? 1 : -1);
            }

            // Update coefficients
            for (size_t j = 0; j < coeffs.size(); ++j) {
                coeffs[j] -= correction[j];
            }

            // Convergence check
            if (total_error < 1e-6) break;
        }

        return coeffs;
    }

    // Solve linear system using Gaussian elimination
    std::vector<double> solveLinearSystem(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        size_t n = A.size();

        // Gaussian elimination
        for (size_t i = 0; i < n; ++i) {
            // Find pivot
            size_t max_row = i;
            for (size_t k = i + 1; k < n; ++k) {
                if (std::abs(A[k][i]) > std::abs(A[max_row][i])) {
                    max_row = k;
                }
            }

            // Swap rows
            std::swap(A[i], A[max_row]);
            std::swap(b[i], b[max_row]);

            // Eliminate column
            for (size_t k = i + 1; k < n; ++k) {
                double factor = A[k][i] / A[i][i];
                for (size_t j = i; j < n; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
                b[k] -= factor * b[i];
            }
        }

        // Back substitution
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (int j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
};
