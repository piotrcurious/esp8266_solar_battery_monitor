#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

enum OptimizationMethod {
    NO_OPTIMIZATION,
    SUBGRADIENT_PROJECTION,
    LEVENBERG_MARQUARDT,
    GRADIENT_DESCENT,
    NELDER_MEAD
};

class AdvancedPolynomialFitter {
public:
    // L1 (Lasso) and L2 (Ridge) regularization parameters
    static constexpr float L1_LAMBDA = 0.01f;
    static constexpr float L2_LAMBDA = 0.01f;

    // Subgradient projection method parameters
    static constexpr float LEARNING_RATE = 0.01f;
    static constexpr int MAX_ITERATIONS = 1000;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-6f;

    // Soft-thresholding operator for L1 regularization
    static float softThreshold(float x, float lambda) {
        return std::copysign(std::max(0.0f, std::abs(x) - lambda), x);
    }

    // Subgradient projection method for polynomial fitting
    static std::vector<float> subgradientProjection(
        const std::vector<std::vector<double>>& A, 
        const std::vector<float>& y, 
        int degree
    ) {
        size_t m = degree + 1;
        size_t n = A.size();

        // Initialize coefficients
        std::vector<float> coeffs(m, 0.0f);
        std::vector<float> best_coeffs = coeffs;
        float best_loss = std::numeric_limits<float>::max();

        // Compute A^T * A and A^T * y for efficient calculations
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

        // Subgradient projection iterations
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // Compute current prediction
            std::vector<float> prediction(n, 0.0f);
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < m; ++j) {
                    prediction[i] += A[i][j] * coeffs[j];
                }
            }

            // Compute residuals
            std::vector<float> residuals(n);
            for (size_t i = 0; i < n; ++i) {
                residuals[i] = prediction[i] - y[i];
            }

            // Compute gradient
            std::vector<float> gradient(m, 0.0f);
            for (size_t j = 0; j < m; ++j) {
                for (size_t i = 0; i < n; ++i) {
                    gradient[j] += 2.0f * residuals[i] * A[i][j];
                }
                
                // Add L2 regularization gradient
                gradient[j] += 2.0f * L2_LAMBDA * coeffs[j];
            }

            // Update coefficients with learning rate
            for (size_t j = 0; j < m; ++j) {
                coeffs[j] -= LEARNING_RATE * gradient[j];
                
                // L1 regularization via soft-thresholding
                coeffs[j] = softThreshold(coeffs[j], L1_LAMBDA * LEARNING_RATE);
            }

            // Compute current loss
            float current_loss = 0.0f;
            for (size_t i = 0; i < n; ++i) {
                current_loss += std::pow(prediction[i] - y[i], 2);
            }
            
            // Add regularization terms
            for (size_t j = 0; j < m; ++j) {
                current_loss += L1_LAMBDA * std::abs(coeffs[j]) + 
                                L2_LAMBDA * coeffs[j] * coeffs[j];
            }

            // Track best solution
            if (current_loss < best_loss) {
                best_loss = current_loss;
                best_coeffs = coeffs;
            }

            // Check convergence
            if (std::abs(current_loss - best_loss) < CONVERGENCE_THRESHOLD) {
                break;
            }
        }

        return best_coeffs;
    }

    // Main fitting method (enhanced to support subgradient projection)
    std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree, 
        OptimizationMethod method = SUBGRADIENT_PROJECTION
    ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Construct the Vandermonde matrix
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x[i];
            }
        }

        std::vector<float> result;
        switch (method) {
            case SUBGRADIENT_PROJECTION:
                result = subgradientProjection(A, y, degree);
                break;
            case LEVENBERG_MARQUARDT:
                // Existing Levenberg-Marquardt implementation
                result = levenbergMarquardt(result, x, y, degree);
                break;
            case NO_OPTIMIZATION:
            default:
                // Solve using normal equations
                result = solveNormalEquation(A, y);
                break;
        }

        return result;
    }

private:
    // Solve normal equation using Gaussian elimination
    static std::vector<float> solveNormalEquation(
        const std::vector<std::vector<double>>& A, 
        const std::vector<float>& y
    ) {
        size_t m = A[0].size();
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < A.size(); ++i) {
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += A[i][j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        return solveLinearSystem(ATA, ATy);
    }

    // Existing methods like solveLinearSystem and levenbergMarquardt 
    // would remain the same as in the original implementation
};
