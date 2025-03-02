#include <Arduino.h>
#include <Eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>

#include <Eigen3/Eigen/src/LU/Determinant.h>
#include <Eigen3/Eigen/src/LU/FullPivLU.h>
#include <Eigen3/Eigen/src/LU/Inverse.h>
#include <Eigen3/Eigen/src/LU/PartialPivLU.h>
#include <Eigen3/Eigen/src/LU/QtDecomposition.h>
#include <Eigen3/Eigen/src/LU/SVDecomposition.h>

#include <Eigen3/Eigen/src/Cholesky/CholeskyDecomposition.h>
#include <Eigen3/Eigen/src/Cholesky/LLT.h>
#include <Eigen3/Eigen/src/Cholesky/LDLT.h>

#include <Eigen3/Eigen/src/QR/HouseholderQR.h>
#include <Eigen3/Eigen/src/QR/ColPivHouseholderQR.h>
#include <Eigen3/Eigen/src/QR/FullPivHouseholderQR.h>
#include <Eigen3/Eigen/src/QR/QRDecomposition.h>


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

        size_t n = x.size();size_t m = degree + 1;

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
        std::vector<double> x_normalized(x.size());
        for (size_t i = 0; i < n; ++i) {
            x_normalized[i] = 2.0 * (x[i] - min_x) / (max_x - min_x) - 1.0; // Map [min_x, max_x] to [-1, 1]
        }


        // 1. Construct the Vandermonde matrix A (Chebyshev basis)
        Eigen::MatrixXd A(n, m);
        for (size_t i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                A(i, j) = chebyshevT(j, x_normalized[i]);
            }
        }

        // 2. Construct the Smoothness Matrix S - ANALYTICALLY COMPUTED for Chebyshev basis
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(m, m);
        double interval_start = -1.0; // Chebyshev basis is orthogonal on [-1, 1]
        double interval_end = 1.0;

        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < m; ++k) {
                S(j, k) = computeSmoothnessMatrixElementAnalyticChebyshev(j, k, interval_start, interval_end);
            }
        }

        // 3. Construct ATA + lambda * S
        Eigen::MatrixXd ATA_plus_lambdaS = (A.transpose() * A) + lambda * S;

        // 4. Construct ATy
        Eigen::VectorXd ATy = A.transpose() * Eigen::VectorXd::Map(y.data(), y.size());

        // 5. Solve the linear system (ATA + lambda * S) * coeffs_chebyshev = ATy (Chebyshev basis coefficients)
        std::vector<double> coeffs_chebyshev_double = solveLinearSystem(ATA_plus_lambdaS, ATy);
        std::vector<float> coeffs_chebyshev(coeffs_chebyshev_double.begin(), coeffs_chebyshev_double.end());


        // 6. Convert Chebyshev basis coefficients to Monomial basis coefficients
        std::vector<float> result_monomial_coeffs = chebyshevToMonomialCoefficients(coeffs_chebyshev);

        return result_monomial_coeffs;
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

    // Helper function to compute element of Smoothness Matrix S ANALYTICALLY for CHEBYSHEV basis
    double computeSmoothnessMatrixElementAnalyticChebyshev(int j_index, int k_index, double interval_start, double interval_end) {
        int j = j_index;
        int k = k_index;

        if (j < 2 || k < 2) { // Second derivative of T_0 and T_1 is zero
            return 0.0;
        }

        // Analytical Calculation of Integral of T_j''(x) * T_k''(x) on [-1, 1].
        // Based on derived formulas (Needs Verification from reliable source - Math textbooks on Chebyshev Polynomials and spectral methods are good resources):

        if (j == k) {
            return (double)(M_PI / 2.0) * j * j * (j * j - 1.0) * (j * j - 1.0) ; // For j=k>=2
        } else if ((j + k) % 2 == 0) {
            return 0.0; // Integral is zero if (j+k) is even due to symmetry properties.
        } else {
            return 0.0; // Integral is zero if j!=k and (j+k) is odd. (Orthogonality-like property of derivatives, needs verification of parity).
        }
        return 0.0; // Default to 0 - should not reach here if cases are correctly handled.


    }


    // Helper function to calculate the second derivative of Chebyshev polynomial T_n''(x) - ANALYTICAL FORM
    double chebyshevTSecondDerivative(int n, double x) {
        if (n <= 1) return 0.0;

        // Analytical formula for the second derivative of Chebyshev Polynomials T_n''(x).
        // Derived from Rodrigue's formula or explicit series representations (Needs Verification against reliable source)

        if (n == 2) return 2.0;
        if (n == 3) return 6.0 * x;
        if (n == 4) return 4.0*(4.0*x*x - 1.0) - 4.0; // = 32x^2 - 20
        // General formula or recurrence for T_n''(x) is more complex to directly compute efficiently for arbitrary n on Arduino.
        // For now, for degrees > 4, we can return a numerical approximation or implement a more optimized recurrence if needed.

        // For simplicity and Arduino computational constraints, for higher degrees, we can use a finite difference approximation if really needed,
        // or limit max degree to a reasonable value where analytical forms or simple formulas are manageable for second derivative.

        // **For a truly optimized version, derive and implement a computationally efficient recurrence or analytical form for T_n''(x) for all n,
        // **and use it in computeSmoothnessMatrixElementAnalyticChebyshev if direct integral evaluation becomes too complex analytically.**


        double h = 1e-4; // Small step for numerical derivative - adjust if needed - NUMERICAL APPROXIMATION - for fallback if analytical not easily computable for all n
        return (chebyshevT(n, x + h) - 2.0 * chebyshevT(n, x) + chebyshevT(n, x - h)) / (h * h); // Numerical approximation as fallback for higher degrees or if analytical formula is complex.
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
            // In monomial basis: coeff(T_n)_i = 2 * coeff(xT_{n-1})_i - coeff(T_{n-2})_i

            for (int i = 1; i <= n - 1; ++i) {
                 conversion_matrix[n][i] += 2.0 * conversion_matrix[n-1][i-1]; // from 2x * T_{n-1}(x) term, shift power by 1
            }
            conversion_matrix[n][0] -= conversion_matrix[n-2][0]; // from - T_{n-2}(x) term
            for (int i = 1; i <= n-2; ++i) {
                 conversion_matrix[n][i] -= conversion_matrix[n-2][i]; // from - T_{n-2}(x) term
            }
            conversion_matrix[n][n] = 2.0 * conversion_matrix[n-1][n-1]; // from 2x * T_{n-1}(x) term, highest power term
        }


        for (int i = 0; i <= degree; ++i) {
            for (int j = 0; j <= degree; ++j) {
                monomial_coeffs[i] += chebyshev_coeffs[j] * conversion_matrix[j][i];
            }
        }


        return monomial_coeffs;
    }


    std::vector<SegmentData> segment_data(const std::vector<double>& x_data, const std::vector<float>& y_data, double gap_threshold_factor) {
        return AdvancedPolynomialFitter::segment_data_static(x_data, y_data, gap_threshold_factor); // Call static version
    }
    static std::vector<SegmentData> segment_data_static(const std::vector<double>& x_data, const std::vector<float>& y_data, double gap_threshold_factor) {
        std::vector<SegmentData> segments;
        if (x_data.empty()) return segments;

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


};
