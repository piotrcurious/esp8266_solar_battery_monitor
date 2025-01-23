#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

class SuperpositionPolynomialFitter {
public:
    static std::vector<double> fitPolynomial(
        const std::vector<double>& x, 
        const std::vector<double>& y, 
        int degree
    ) {
        // Validate input
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        // System size
        size_t n = x.size();
        size_t m = degree + 1;

        // Construct basis function matrix using superposition principle
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                // Fundamental linear combination of basis functions
                A[i][j] = std::pow(x[i], j);
            }
        }

        // Compute A^T * A and A^T * y using linear superposition
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        // Superposition applied to matrix multiplication
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                // Compute A^T * y component
                ATy[j] += A[i][j] * y[i];

                // Compute A^T * A components
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += A[i][j] * A[i][k];
                }
            }
        }

        // Solve using linear system decomposition
        return solveLinearSystem(ATA, ATy);
    }

private:
    /**
     * Solve linear system using Gaussian elimination with partial pivoting
     * Implements direct solution of linear system respecting superposition
     */
    static std::vector<double> solveLinearSystem(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        size_t n = A.size();

        // Augment matrix with right-hand side
        for (size_t i = 0; i < n; ++i) {
            A[i].push_back(b[i]);
        }

        // Gaussian elimination with partial pivoting
        for (size_t i = 0; i < n; ++i) {
            // Partial pivoting
            size_t maxRow = i;
            for (size_t k = i + 1; k < n; ++k) {
                if (std::abs(A[k][i]) > std::abs(A[maxRow][i])) {
                    maxRow = k;
                }
            }
            std::swap(A[i], A[maxRow]);

            // Check for singular matrix
            if (std::abs(A[i][i]) < 1e-10) {
                return {};  // Singular or near-singular matrix
            }

            // Eliminate below
            for (size_t k = i + 1; k < n; ++k) {
                double factor = A[k][i] / A[i][i];
                for (size_t j = i; j <= n; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }

        // Back-substitution (superposition of individual solutions)
        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = A[i][n];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }

        return x;
    }
};
