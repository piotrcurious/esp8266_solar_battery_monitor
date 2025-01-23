#include <vector>
#include <cmath>
#include <iostream>

class SuperpositionPolynomialFitter {
public:
    static std::vector<double> fitPolynomial(
        const std::vector<double>& x, 
        const std::vector<double>& y, 
        int degree
    ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Return empty vector on invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Compute A^T * A and A^T * y directly without explicit matrix A storage
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            std::vector<double> row(m, 1.0);
            double x_power = 1.0;

            for (size_t j = 1; j < m; ++j) {
                x_power *= x[i];
                row[j] = x_power;
            }

            for (size_t j = 0; j < m; ++j) {
                ATy[j] += row[j] * y[i];
                for (size_t k = 0; k <= j; ++k) {  // Only compute upper half (symmetric matrix)
                    ATA[j][k] += row[j] * row[k];
                }
            }
        }

        // Fill lower half of ATA (since it's symmetric)
        for (size_t j = 0; j < m; ++j) {
            for (size_t k = j + 1; k < m; ++k) {
                ATA[j][k] = ATA[k][j];
            }
        }

        return solveQR(ATA, ATy);
    }

private:
    // Householder QR decomposition solver for linear least squares
    static std::vector<double> solveQR(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        size_t n = A.size();

        // Householder QR decomposition
        for (size_t k = 0; k < n; ++k) {
            double norm_x = 0.0;
            for (size_t i = k; i < n; ++i) {
                norm_x += A[i][k] * A[i][k];
            }
            norm_x = std::sqrt(norm_x);

            double alpha = (A[k][k] > 0) ? -norm_x : norm_x;
            double r = std::sqrt(0.5 * (alpha * alpha - A[k][k] * alpha));

            std::vector<double> v(n, 0.0);
            v[k] = (A[k][k] - alpha) / (2 * r);
            for (size_t i = k + 1; i < n; ++i) {
                v[i] = A[i][k] / (2 * r);
            }

            // Apply Householder transformation to A and b
            for (size_t j = k; j < n; ++j) {
                double dot = 0.0;
                for (size_t i = k; i < n; ++i) {
                    dot += v[i] * A[i][j];
                }
                for (size_t i = k; i < n; ++i) {
                    A[i][j] -= 2 * v[i] * dot;
                }
            }

            double dot_b = 0.0;
            for (size_t i = k; i < n; ++i) {
                dot_b += v[i] * b[i];
            }
            for (size_t i = k; i < n; ++i) {
                b[i] -= 2 * v[i] * dot_b;
            }

            A[k][k] = alpha;
            for (size_t i = k + 1; i < n; ++i) {
                A[i][k] = 0.0;
            }
        }

        // Back substitution to solve Rx = Q^T b
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = b[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
};
