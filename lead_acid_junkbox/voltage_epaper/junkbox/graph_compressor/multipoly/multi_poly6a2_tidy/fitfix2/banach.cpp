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
        std::vector<float> coeffs;
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
    // Helper function to evaluate the polynomial at a given x value
    float evaluatePolynomial(double x_val, const std::vector<float>& coefficients) {
        float y_val = 0.0;
        double x_power = 1.0;
        for (float coeff : coefficients) {
            y_val += coeff * x_power;
            x_power *= x_val;
        }
        return y_val;
    }
    // Helper function to perform the initial polynomial fit (your original logic)
    std::vector<float> fitPolynomialInternal(const std::vector<double>& x, const std::vector<float>& y, int degree, OptimizationMethod method) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();size_t m = degree + 1;

        // Construct the Vandermonde matrix
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

    // New Fitter function based on Banach Space Regularization
    std::vector<float> fitPolynomialBanachSpaceRegularizedD(const std::vector<double>& x, const std::vector<float>& y, int degree, double lambda) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // 1. Construct the Vandermonde matrix A
        Eigen::MatrixXd A(n, m);
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A(i, j) = xi;
                xi *= x[i];
            }
        }

        // 2. Construct the Smoothness Matrix S
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(m, m);
        double interval_start = *std::min_element(x.begin(), x.end());
        double interval_end = *std::max_element(x.begin(), x.end());

        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < m; ++k) {
                S(j, k) = computeSmoothnessMatrixElement(j, k, interval_start, interval_end);
            }
        }

        // 3. Construct ATA + lambda * S
        Eigen::MatrixXd ATA_plus_lambdaS = (A.transpose() * A) + lambda * S;

        // 4. Construct ATy
        Eigen::VectorXd ATy = A.transpose() * Eigen::VectorXd::Map(y.data(), y.size());

        // 5. Solve the linear system (ATA + lambda * S) * coeffs = ATy
        std::vector<double> coeffs_double = solveLinearSystem(ATA_plus_lambdaS, ATy);

        // 6. Convert coefficients to float
        std::vector<float> result(coeffs_double.begin(), coeffs_double.end());
        return result;
    }

private:
    // Helper function to compute an element of the Smoothness Matrix S for monomial basis
    double computeSmoothnessMatrixElement(int j_index, int k_index, double interval_start, double interval_end) {
        int j = j_index;
        int k = k_index;

        if (j < 2 || k < 2) { // Second derivative of x^0 and x^1 is zero
            return 0.0;
        }

        auto integrand = [&](double x) {
            double second_deriv_j = (j >= 2) ? (double)(j * (j - 1) * pow(x, j - 2)) : 0.0;
            double second_deriv_k = (k >= 2) ? (double)(k * (k - 1) * pow(x, k - 2)) : 0.0;
            return second_deriv_j * second_deriv_k;
        };

        // Simple numerical integration (e.g., trapezoidal rule - can be improved for higher accuracy if needed)
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


    std::vector<SegmentData> segment_data(const std::vector<double>& x_data, const std::vector<float>& y_data, double gap_threshold_factor) {
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
