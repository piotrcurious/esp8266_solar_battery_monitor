#include <Arduino.h>
#include <Eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>

// Only include the core Eigen headers needed
#include <Eigen3/Eigen/Dense>
#include <Eigen3/Eigen/LU>

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

    AdvancedPolynomialFitter(){};
    ~AdvancedPolynomialFitter(){};

    // Helper function to solve linear system using Eigen's Gaussian elimination (FullPivLU for robustness)
    std::vector<double> solveLinearSystem(Eigen::MatrixXd& matrix, Eigen::VectorXd& rhs) {
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
    float evaluatePolynomial(double x_val, const std::vector<float>& coefficients) {
        float y_val = 0.0;
        double x_power = 1.0; // Start with x^0 = 1
        for (size_t i = 0; i < coefficients.size(); i++) {
            y_val += coefficients[i] * x_power;
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
            double xi_power = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A(i, j) = xi_power;
                xi_power *= x[i];
            }
        }

        // Construct the normal equation: (A^T * A) * coeffs = A^T * y
        Eigen::MatrixXd ATA = A.transpose() * A;
        Eigen::VectorXd ATy = A.transpose() * Eigen::VectorXd::Map(y.data(), y.size());

        // Solve the normal equation using Gaussian elimination
        std::vector<double> coeffs_double = solveLinearSystem(ATA, ATy);

        // Convert coefficients to float
        std::vector<float> result(coeffs_double.begin(), coeffs_double.end());
        return result;
    }

    // Improved Fitter function based on Banach Space Regularization - ANALYTICAL Smoothness Matrix, CHEBYSHEV BASIS
    std::vector<float> fitPolynomialBanachSpaceRegularizedD(const std::vector<double>& x, const std::vector<float>& y, int degree, double lambda) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Normalize x data to [-1, 1] for Chebyshev basis (important for Chebyshev properties)
        double min_x = *std::min_element(x.begin(), x.end());
        double max_x = *std::max_element(x.begin(), x.end());
        std::vector<double> x_normalized(n);
        for (size_t i = 0; i < n; ++i) {
            x_normalized[i] = 2.0 * (x[i] - min_x) / (max_x - min_x) - 1.0; // Map [min_x, max_x] to [-1, 1]
        }

        // 1. Construct the Vandermonde matrix A (Chebyshev basis)
        Eigen::MatrixXd A(n, m);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                A(i, j) = chebyshevT(j, x_normalized[i]);
            }
        }

        // 2. Construct the Smoothness Matrix S - ANALYTICALLY COMPUTED for Chebyshev basis
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(m, m);
        
        // Fill the smoothness matrix with appropriate values
        // This is a corrected formula based on analytical derivatives of Chebyshev polynomials
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = 0; k < m; ++k) {
                S(j, k) = computeSmoothnessMatrixElementAnalyticChebyshev(j, k);
            }
        }

        // 3. Construct ATA + lambda * S
        Eigen::MatrixXd ATA = A.transpose() * A;
        double scale_factor = 1.0 / (n * n); // Normalize lambda relative to data size
        Eigen::MatrixXd ATA_plus_lambdaS = ATA + (lambda * scale_factor) * S;

        // 4. Construct ATy
        Eigen::VectorXd ATy = A.transpose() * Eigen::VectorXd::Map(y.data(), y.size());

        // 5. Solve the linear system (ATA + lambda * S) * coeffs_chebyshev = ATy (Chebyshev basis coefficients)
        std::vector<double> coeffs_chebyshev_double = solveLinearSystem(ATA_plus_lambdaS, ATy);
        std::vector<float> coeffs_chebyshev(coeffs_chebyshev_double.begin(), coeffs_chebyshev_double.end());

        // 6. Convert Chebyshev basis coefficients to Monomial basis coefficients
        std::vector<float> result_monomial_coeffs = chebyshevToMonomialCoefficients(coeffs_chebyshev);

        return result_monomial_coeffs;
    }

    // Public wrapper for segmenting data
    static std::vector<SegmentData> segment_data_static(const std::vector<double>& x_data, const std::vector<float>& y_data, double gap_threshold_factor) {
        std::vector<SegmentData> segments;
        if (x_data.empty() || x_data.size() != y_data.size()) return segments;

        SegmentData current_segment;
        current_segment.x_range.first = x_data[0];
        current_segment.x_range.second = x_data[0];
        current_segment.x.push_back(x_data[0]);
        current_segment.y.push_back(y_data[0]);

        double avg_x_spacing = 0.0;
        if (x_data.size() > 1) {
            for (size_t i = 1; i < x_data.size(); ++i) {
                avg_x_spacing += (x_data[i] - x_data[i - 1]);
            }
            avg_x_spacing /= (x_data.size() - 1);
        }
        double gap_threshold = avg_x_spacing * gap_threshold_factor; // Gap threshold relative to average spacing

        for (size_t i = 1; i < x_data.size(); ++i) {
            if ((x_data[i] - x_data[i - 1]) > gap_threshold) {
                // Start a new segment
                segments.push_back(current_segment);
                current_segment = SegmentData();
                current_segment.x_range.first = x_data[i];
                current_segment.x_range.second = x_data[i];
                current_segment.x.push_back(x_data[i]);
                current_segment.y.push_back(y_data[i]);
            } else {
                current_segment.x.push_back(x_data[i]);
                current_segment.y.push_back(y_data[i]);
                current_segment.x_range.second = x_data[i]; // Update x_range max
            }
        }
        segments.push_back(current_segment); // Add the last segment

        return segments;
    }

    std::vector<SegmentData> segment_data(const std::vector<double>& x_data, const std::vector<float>& y_data, double gap_threshold_factor) {
        return segment_data_static(x_data, y_data, gap_threshold_factor); // Call static version
    }

private:
    // Helper function to evaluate Chebyshev polynomial T_n(x) of the first kind using recurrence relation
    double chebyshevT(int n, double x) {
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

    // Improved analytical formula for the smoothness matrix elements
    double computeSmoothnessMatrixElementAnalyticChebyshev(int j, int k) {
        // For Chebyshev polynomials, the 2nd derivative inner product has analytical forms
        
        // T_0''(x) = T_1''(x) = 0
        if (j < 2 || k < 2) {
            return 0.0;
        }
        
        // For the diagonal elements (j = k ≥ 2), we have:
        if (j == k) {
            double n = j;
            // Diagonal formula: π/2 * n²(n²-1)
            return M_PI / 2.0 * n * n * (n * n - 1);
        }
        
        // For the off-diagonal elements, we use the derived formula based on
        // the orthogonality properties of Chebyshev polynomials
        
        // First special cases:
        if ((j + k) % 2 != 0) { 
            // Odd parity difference, zeros due to orthogonality
            return 0.0;
        }
        
        // General off-diagonal case with even parity difference
        int n = j;
        int m = k;
        
        // Only compute for n != m and n,m ≥ 2
        if (n != m && n >= 2 && m >= 2) {
            // This formula is based on the orthogonality of Chebyshev polynomials 
            // with appropriate weight functions for the second derivatives
            // Formula: n*m*(n²-1)*(m²-1)/(n²-m²)² for appropriate cases where n+m is even
            if (std::abs(n - m) > 1) {  // Ensure no division by zero
                double numerator = n * m * (n * n - 1) * (m * m - 1);
                double denominator = (n * n - m * m) * (n * n - m * m);
                return M_PI * numerator / denominator; 
            }
        }
        
        // Default fallback for any cases not covered
        return 0.0;
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

        // Create a transformation matrix where:
        // conversion_matrix[n][m] = coefficient of x^m in T_n(x)
        std::vector<std::vector<float>> conversion_matrix(degree + 1, std::vector<float>(degree + 1, 0.0));

        // Initialize for T_0(x) = 1 and T_1(x) = x
        conversion_matrix[0][0] = 1.0;
        conversion_matrix[1][1] = 1.0;

        // Build using recurrence: T_n(x) = 2xT_{n-1}(x) - T_{n-2}(x)
        for (int n = 2; n <= degree; ++n) {
            // Apply 2x * T_{n-1}(x) term
            for (int i = 0; i < n; ++i) {
                conversion_matrix[n][i+1] += 2.0 * conversion_matrix[n-1][i]; // x shifts power up by 1
            }
            
            // Apply -T_{n-2}(x) term
            for (int i = 0; i <= n-2; ++i) {
                conversion_matrix[n][i] -= conversion_matrix[n-2][i];
            }
        }

        // Convert coefficients using the transformation matrix
        for (int i = 0; i <= degree; ++i) {
            for (int j = 0; j <= degree; ++j) {
                monomial_coeffs[i] += chebyshev_coeffs[j] * conversion_matrix[j][i];
            }
        }

        return monomial_coeffs;
    }
};
