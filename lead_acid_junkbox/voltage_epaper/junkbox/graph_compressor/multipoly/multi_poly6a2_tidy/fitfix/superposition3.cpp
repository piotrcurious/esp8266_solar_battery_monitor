#include <vector>
#include <algorithm>
#include <cmath>

class SuperpositionPolynomialFitter {
public:
    static std::vector<double> fitPolynomial(
        const std::vector<double>& x, 
        const std::vector<double>& y, 
        int degree
    ) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Compute A^T * A and A^T * y directly without constructing A
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            double x_power = 1.0;  // Start with x^0
            std::vector<double> row(m, 1.0);
            
            for (size_t j = 1; j < m; ++j) {
                x_power *= x[i];  // Efficient power computation
                row[j] = x_power;
            }

            for (size_t j = 0; j < m; ++j) {
                ATy[j] += row[j] * y[i];
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += row[j] * row[k];
                }
            }
        }

        return solveLinearSystem(ATA, ATy);
    }

private:
    // Solve linear system using Gaussian elimination with full pivoting
    static std::vector<double> solveLinearSystem(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        size_t n = A.size();

        // Augment the matrix A with vector b
        for (size_t i = 0; i < n; ++i) {
            A[i].push_back(b[i]);
        }

        // Gaussian elimination with full pivoting
        for (size_t i = 0; i < n; ++i) {
            // Find pivot element
            size_t maxRow = i, maxCol = i;
            for (size_t k = i; k < n; ++k) {
                for (size_t j = i; j < n; ++j) {
                    if (std::abs(A[k][j]) > std::abs(A[maxRow][maxCol])) {
                        maxRow = k;
                        maxCol = j;
                    }
                }
            }
            
            if (std::abs(A[maxRow][maxCol]) < 1e-12) {
                return {};  // Singular matrix
            }

            // Swap rows and columns for pivoting
            std::swap(A[i], A[maxRow]);
            for (size_t k = 0; k < n; ++k) {
                std::swap(A[k][i], A[k][maxCol]);
            }
            
            // Forward elimination
            for (size_t k = i + 1; k < n; ++k) {
                double factor = A[k][i] / A[i][i];
                for (size_t j = i; j <= n; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }

        // Back-substitution
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
