#include <Arduino.h>
#include <Eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>

// Consolidated includes - only include the main Eigen headers
#include <Eigen3/Eigen/LU>
#include <Eigen3/Eigen/Cholesky>
#include <Eigen3/Eigen/QR>

class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod {
        GAUSSIAN_ELIMINATION
    };
    
    struct SegmentPolynomial {
        std::vector<float> coeffs; // Coefficients in MONOMIAL basis
        int degree;
        std::pair<double, double> x_range;
    };
    
    struct SegmentData {
        std::vector<double> x;
        std::vector<float> y;
        std::pair<double, double> x_range;
    };

    AdvancedPolynomialFitter() = default;
    ~AdvancedPolynomialFitter() = default;

    // Helper function to solve linear system using Eigen's Gaussian elimination (FullPivLU for robustness)
    std::vector<double> solveLinearSystem(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& rhs) {
        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomposition(matrix);
        if (lu_decomposition.isInvertible()) {
            Eigen::VectorXd solution = lu_decomposition.solve(rhs);
            return std::vector<double>(solution.data(), solution.data() + solution.size());
        } else {
            Serial.println("Warning: Linear system is singular, cannot solve using Gaussian elimination.");
            return {}; // Return empty vector to indicate failure
        }
    }

    // Helper function to evaluate polynomial in MONOMIAL basis at a given x value
    float evaluatePolynomial(double x_val, const std::vector<float>& coefficients) const {
        float y_val = 0.0;
        double x_power = 1.0;
        
        for (float coeff : coefficients) {
            y_val += coeff * x_power;
            x_power *= x_val;
        }
        return y_val;
    }

    // Helper function to perform the standard polynomial fit using normal equations (Monomial basis)
    std::vector<float> fitPolynomialInternal(const std::vector<double>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Construct the Vandermonde matrix (Monomial basis)
        Eigen::MatrixXd A(n, m);
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A(i, j) = xi;
                xi *= x[i];
            }
        }

        // Construct the normal equation: (A^T * A) * coeffs = A^T * y
        Eigen::MatrixXd ATA = A.transpose() * A;
        Eigen::VectorXd ATy = A.transpose() * Eigen::Map<const Eigen::VectorXf>(y.data(), y.size()).cast<double>();

        // Solve the normal equation using Gaussian elimination
        std::vector<double> coeffs_double = solveLinearSystem(ATA, ATy);

        // Convert coefficients to float
        std::vector<float> result(coeffs_double.begin(), coeffs_double.end());
        return result;
    }
    
    // Added missing fitPolynomialBanachSpaceRegularizedD method
    std::vector<float> fitPolynomialBanachSpaceRegularizedD(const std::vector<double>& x, const std::vector<float>& y, int degree, double lambda) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Normalize x values to [-1, 1] for better numerical stability with Chebyshev basis
        double x_min = *std::min_element(x.begin(), x.end());
        double x_max = *std::max_element(x.begin(), x.end());
        double x_range = x_max - x_min;
        
        std::vector<double> x_normalized(n);
        for (size_t i = 0; i < n; ++i) {
            // Map [x_min, x_max] to [-1, 1]
            x_normalized[i] = 2.0 * (x[i] - x_min) / x_range - 1.0;
        }

        // Construct the Chebyshev design matrix
        Eigen::MatrixXd A(n, m);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                A(i, j) = chebyshevT(j, x_normalized[i]);
            }
        }

        // Construct the regularization matrix S (analytical)
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(m, m);
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = 0; k < m; ++k) {
                S(j, k) = computeSmoothnessMatrixElementAnalyticChebyshev(j, k, -1.0, 1.0);
            }
        }

        // Construct the regularized normal equation: (A^T * A + lambda * S) * coeffs = A^T * y
        Eigen::MatrixXd ATA = A.transpose() * A;
        Eigen::MatrixXd regularized_matrix = ATA + lambda * S;
        Eigen::VectorXd ATy = A.transpose() * Eigen::Map<const Eigen::VectorXf>(y.data(), y.size()).cast<double>();

        // Solve the regularized system
        std::vector<double> chebyshev_coeffs_double = solveLinearSystem(regularized_matrix, ATy);
        if (chebyshev_coeffs_double.empty()) {
            return {}; // Failed to solve
        }

        // Convert to float and then to monomial basis
        std::vector<float> chebyshev_coeffs(chebyshev_coeffs_double.begin(), chebyshev_coeffs_double.end());
        return chebyshevToMonomialCoefficients(chebyshev_coeffs);
    }

    // Improved Fitter function based on Banach Space Regularization - ANALYTICAL Smoothness Matrix, CHEBYSHEV BASIS, SMOOTHNESS-AWARE SEGMENTATION
    std::vector<SegmentPolynomial> segmentedFitPolynomialBanachSpaceRegularized(
            const std::vector<double>& x_data, 
            const std::vector<float>& y_data, 
            int max_degree, 
            double base_lambda, 
            double smoothness_threshold_residual_variance) {
        
        std::vector<SegmentPolynomial> segment_polynomials_result;
        std::vector<SegmentData> segments = segment_data_smoothness_aware(
            x_data, y_data, smoothness_threshold_residual_variance); // Smoothness-aware segmentation

        for (const SegmentData& segment : segments) {
            const std::vector<double>& x_segment = segment.x;
            const std::vector<float>& y_segment = segment.y;

            if (x_segment.size() < 2) { // Skip segments with too few points
                Serial.println("Skipping segment with too few points.");
                continue;
            }

            // Calculate segment-specific lambda based on segment 'non-smoothness'
            double segment_residual_variance = calculate_residual_variance_low_degree_fit(x_segment, y_segment, 2);
            double segment_lambda = calculateSegmentLambda(
                base_lambda, smoothness_threshold_residual_variance, segment_residual_variance);

            // Limit degree by segment size and max_degree, ensure degree is at least 1
            int current_degree = std::min((int)x_segment.size() - 1, max_degree);
            current_degree = std::max(current_degree, 1);

            std::vector<float> segment_coeffs_monomial;
            if (current_degree >= 1) {
                // Banach space regularized fit with Chebyshev basis and segment-specific lambda
                segment_coeffs_monomial = fitPolynomialBanachSpaceRegularizedD(
                    x_segment, y_segment, current_degree, segment_lambda);

                if (segment_coeffs_monomial.empty()) {
                    Serial.println("Warning: Banach Space Regularized Fit failed for a segment, falling back to unregularized fit.");
                    // Fallback to basic fit
                    segment_coeffs_monomial = fitPolynomialInternal(
                        x_segment, y_segment, current_degree, OptimizationMethod::GAUSSIAN_ELIMINATION);
                }
            } else {
                // Degenerate case, should not happen due to degree limits but included for robustness
                Serial.println("Warning: Degenerate case - using linear fit (degree 1) for segment.");
                segment_coeffs_monomial = fitPolynomialInternal(
                    x_segment, y_segment, 1, OptimizationMethod::GAUSSIAN_ELIMINATION);
            }

            if (!segment_coeffs_monomial.empty()) {
                segment_polynomials_result.push_back({segment_coeffs_monomial, current_degree, segment.x_range});
            } else {
                Serial.println("Error: Polynomial fitting completely failed for a segment.");
            }
        }
        return segment_polynomials_result;
    }

private:
    double calculateSegmentLambda(double base_lambda, double smoothness_threshold_residual_variance, double segment_residual_variance) {
        // Heuristic function to adjust lambda based on segment residual variance.
        double lambda_factor = 1.0;
        
        if (segment_residual_variance > 0) {
            // Inverse relation - smoother segments get higher lambda
            lambda_factor = smoothness_threshold_residual_variance / segment_residual_variance;
        } else {
            lambda_factor = 10.0; // Very smooth segment, or constant data - use higher lambda
        }

        return base_lambda * lambda_factor;
    }

    double calculate_residual_variance_low_degree_fit(
            const std::vector<double>& x_segment, 
            const std::vector<float>& y_segment, 
            int low_degree) {
        
        if (x_segment.size() <= low_degree + 1 || low_degree < 1) {
            return 0.0; // Not enough data points for low degree fit, or invalid degree
        }

        std::vector<float> low_degree_coeffs = fitPolynomialInternal(
            x_segment, y_segment, low_degree, OptimizationMethod::GAUSSIAN_ELIMINATION);
        
        if (low_degree_coeffs.empty()) {
            return 0.0; // Fit failed
        }

        double sum_squared_residuals = 0.0;
        for (size_t i = 0; i < x_segment.size(); ++i) {
            float fitted_y = evaluatePolynomial(x_segment[i], low_degree_coeffs);
            sum_squared_residuals += pow(y_segment[i] - fitted_y, 2);
        }
        
        // Sample variance - unbiased estimate
        return sum_squared_residuals / (x_segment.size() - low_degree - 1);
    }

    // Helper function to evaluate Chebyshev polynomial T_n(x) of the first kind using recurrence relation
    double chebyshevT(int n, double x) const {
        if (n == 0) return 1.0;
        if (n == 1) return x;
        
        double t_prev = 1.0;
        double t_curr = x;
        double t_next;
        
        for (int i = 2; i <= n; ++i) {
            t_next = 2.0 * x * t_curr - t_prev;
            t_prev = t_curr;
            t_curr = t_next;
        }
        return t_curr;
    }

    // Helper function to compute element of Smoothness Matrix S ANALYTICALLY for CHEBYSHEV basis
    double computeSmoothnessMatrixElementAnalyticChebyshev(int j_index, int k_index, double interval_start, double interval_end) {
        int j = j_index;
        int k = k_index;

        // Second derivative of T_0 and T_1 is zero
        if (j < 2 || k < 2) {
            return 0.0;
        }

        // Analytical calculation for Chebyshev polynomials T_n(x) on [-1, 1]
        if (j == k) {
            // Corrected formula for j=k>=2
            return (double)(M_PI / 2.0) * j * j * (j * j - 1.0) * (j + 1.0) * (k - 1.0);
        } else if ((j + k) % 2 != 0) {
            // Orthogonality property - integral is zero if (j+k) is odd
            return 0.0;
        } else {
            // Orthogonality-like property for even sum and different indices
            return 0.0;
        }
    }

    // Helper function to calculate the second derivative of Chebyshev polynomial T_n''(x) - ANALYTICAL FORM
    double chebyshevTSecondDerivative(int n, double x) const {
        if (n <= 1) return 0.0;

        // Direct formulas for efficiency for lower degrees
        if (n == 2) return 2.0;
        if (n == 3) return 6.0 * x;
        if (n == 4) return 32.0 * x * x - 8.0;

        // Numerical approximation for higher degrees
        const double h = 1e-4;
        return (chebyshevT(n, x + h) - 2.0 * chebyshevT(n, x) + chebyshevT(n, x - h)) / (h * h);
    }

    // Helper function to convert Chebyshev basis coefficients to Monomial basis coefficients
    std::vector<float> chebyshevToMonomialCoefficients(const std::vector<float>& chebyshev_coeffs) {
        int degree = chebyshev_coeffs.size() - 1;
        std::vector<float> monomial_coeffs(degree + 1, 0.0);

        if (degree == 0) {
            monomial_coeffs[0] = chebyshev_coeffs[0];
            return monomial_coeffs;
        }
        if (degree == 1) {
            monomial_coeffs[0] = chebyshev_coeffs[0];
            monomial_coeffs[1] = chebyshev_coeffs[1];
            return monomial_coeffs;
        }

        std::vector<std::vector<float>> conversion_matrix(degree + 1, std::vector<float>(degree + 1, 0.0));

        conversion_matrix[0][0] = 1.0; // T_0 = 1
        conversion_matrix[1][1] = 1.0; // T_1 = x

        for (int n = 2; n <= degree; ++n) {
            // Recurrence relation: T_n(x) = 2xT_{n-1}(x) - T_{n-2}(x)
            for (int i = 1; i <= n - 1; ++i) {
                conversion_matrix[n][i] += 2.0 * conversion_matrix[n-1][i-1]; // from 2x * T_{n-1}(x) term
            }
            conversion_matrix[n][0] -= conversion_matrix[n-2][0]; // from - T_{n-2}(x) term
            for (int i = 1; i <= n-2; ++i) {
                conversion_matrix[n][i] -= conversion_matrix[n-2][i]; // from - T_{n-2}(x) term
            }
            conversion_matrix[n][n] = 2.0 * conversion_matrix[n-1][n-1]; // highest power term
        }

        for (int i = 0; i <= degree; ++i) {
            for (int j = 0; j <= degree; ++j) {
                monomial_coeffs[i] += chebyshev_coeffs[j] * conversion_matrix[j][i];
            }
        }
        return monomial_coeffs;
    }

    std::vector<SegmentData> segment_data_smoothness_aware(
            const std::vector<double>& x_data, 
            const std::vector<float>& y_data, 
            double smoothness_threshold_residual_variance) {
        return segment_data_smoothness_aware_static(x_data, y_data, smoothness_threshold_residual_variance);
    }

    // Public for testing purposes, otherwise would be private
    static std::vector<SegmentData> segment_data_smoothness_aware_static(
            const std::vector<double>& x_data, 
            const std::vector<float>& y_data, 
            double smoothness_threshold_residual_variance) {
        
        std::vector<SegmentData> segments;
        if (x_data.empty()) return segments;

        SegmentData current_segment;
        current_segment.x_range.first = x_data[0];
        current_segment.x_range.second = x_data[0];
        current_segment.x.push_back(x_data[0]);
        current_segment.y.push_back(y_data[0]);

        for (size_t i = 1; i < x_data.size(); ++i) {
            std::vector<double> x_test_segment = current_segment.x;
            std::vector<float> y_test_segment = current_segment.y;
            x_test_segment.push_back(x_data[i]);
            y_test_segment.push_back(y_data[i]);

            // Need at least 3 points to reliably estimate variance of residuals
            if (x_test_segment.size() <= 3) {
                current_segment.x.push_back(x_data[i]);
                current_segment.y.push_back(y_data[i]);
                current_segment.x_range.second = x_data[i];
                continue; // Segment too short, cannot assess smoothness reliably yet
            }

            // Check smoothness of extended segment with degree 2 fit
            double current_residual_variance = calculate_residual_variance_low_degree_fit_static(
                x_test_segment, y_test_segment, 2);

            if (current_residual_variance <= smoothness_threshold_residual_variance) {
                // Segment still considered smooth enough, extend it
                current_segment.x.push_back(x_data[i]);
                current_segment.y.push_back(y_data[i]);
                current_segment.x_range.second = x_data[i];
            } else {
                // Segment becomes non-smooth if extended, start a new segment
                segments.push_back(current_segment);
                current_segment = SegmentData();
                current_segment.x_range.first = x_data[i];
                current_segment.x_range.second = x_data[i];
                current_segment.x.push_back(x_data[i]);
                current_segment.y.push_back(y_data[i]);
            }
        }
        
        segments.push_back(current_segment); // Add the last segment
        return segments;
    }

    static double calculate_residual_variance_low_degree_fit_static(
            const std::vector<double>& x_segment, 
            const std::vector<float>& y_segment, 
            int low_degree) {
        
        AdvancedPolynomialFitter tempFitter;
        if (x_segment.size() <= low_degree + 1 || low_degree < 1) {
            return 0.0; // Not enough data points or invalid degree
        }

        std::vector<float> low_degree_coeffs = tempFitter.fitPolynomialInternal(
            x_segment, y_segment, low_degree, OptimizationMethod::GAUSSIAN_ELIMINATION);
        
        if (low_degree_coeffs.empty()) {
            return 0.0; // Fit failed
        }

        double sum_squared_residuals = 0.0;
        for (size_t i = 0; i < x_segment.size(); ++i) {
            float fitted_y = tempFitter.evaluatePolynomial(x_segment[i], low_degree_coeffs);
            sum_squared_residuals += pow(y_segment[i] - fitted_y, 2);
        }
        
        // Sample variance - unbiased estimate
        return sum_squared_residuals / (x_segment.size() - low_degree - 1);
    }

    // Helper function for monomial basis (fallback)
    double computeSmoothnessMatrixElementAnalytic(int j_index, int k_index, double interval_start, double interval_end) {
        int j = j_index;
        int k = k_index;

        // Second derivative of x^0 and x^1 is zero
        if (j < 2 || k < 2) {
            return 0.0;
        }

        int power = j + k - 4; // Power of x in integrand

        // Check for divergent integral
        if (power < -1) {
            return 0.0;
        }
        
        double integral_value;
        if (power == -1) {
            integral_value = log(fabs(interval_end)) - log(fabs(interval_start));
        } else {
            integral_value = (pow(interval_end, power + 1) - pow(interval_start, power + 1)) / (power + 1.0);
        }

        return (double)(j * (j - 1) * k * (k - 1)) * integral_value;
    }
};
