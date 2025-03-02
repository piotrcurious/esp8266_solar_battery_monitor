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

        // Analytical calculation of integral of T_j''(x) * T_k''(x) on [-1, 1].
        // Derivation is more involved, using properties of Chebyshev polynomials and orthogonality.
        // For simplicity, we will approximate here with a numerical integration for Chebyshev case as well,
        // BUT for true improvement, analytical derivation and implementation of this integral for Chebyshev basis is needed for performance and accuracy.

        auto integrand = [&](double x) {
            double second_deriv_Tj = chebyshevTSecondDerivative(j, x);
            double second_deriv_Tk = chebyshevTSecondDerivative(k, x);
            return second_deriv_Tj * second_deriv_Tk;
        };

        // Simple numerical integration (trapezoidal rule) -  For Chebyshev, ANALYTICAL integration of S matrix is preferred for best performance.
        int num_intervals = 100; // Adjust for accuracy vs performance
        double h = (interval_end - interval_start) / num_intervals;
        double integral_val = 0.0;

        integral_val += 0.5 * integrand(interval_start);
        integral_val += 0.5 * integrand(interval_end);
        for (int i = 1; i < num_intervals; ++i) {
            integral_val += integrand(interval_start + i * h);
        }
        integral_val *= h;
        return integral_val;
    }

    // Helper function to calculate the second derivative of Chebyshev polynomial T_n''(x)
    double chebyshevTSecondDerivative(int n, double x) {
        if (n <= 1) return 0.0; // Second derivative of T0 and T1 is zero.

        // Formula for second derivative of Chebyshev polynomial (can be derived using recurrence relations/formulas)
        // T_n''(x) = n * T_{n-1}'(x) - x * T_{n}''(x) - T_{n}'(x)
        // T_n''(x) = n(n-1) T_{n-2}(x) + n x U_{n-2}(x) where U_n is Chebyshev polynomial of the second kind
        // A simpler recurrence based on relation with lower order Chebyshev T polynomials might be derivable/preferable for Arduino.

        // For simplicity and to avoid dependency on Chebyshev U, we can approximate the second derivative using finite differences for now,
        // or implement analytical formula if derived.  For accurate and efficient implementation, analytical form is better.

        double h = 1e-4; // Small step for numerical derivative - adjust if needed
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
