#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

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

        // Compute A^T * A and A^T * y without constructing A explicitly
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
        std::vector<double> ATy(m, 0.0);

        for (size_t i = 0; i < n; ++i) {
            double x_power = 1.0;
            std::vector<double> row(m, 1.0);

            for (size_t j = 1; j < m; ++j) {
                x_power *= x[i];
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
    // Solve linear system using LU decomposition with full pivoting
    static std::vector<double> solveLinearSystem(
        std::vector<std::vector<double>>& A, 
        std::vector<double>& b
    ) {
        size_t n = A.size();
        std::vector<size_t> rowPiv(n), colPiv(n);

        // Initialize row and column pivot indices
        for (size_t i = 0; i < n; ++i) {
            rowPiv[i] = colPiv[i] = i;
        }

        // Full pivoting and LU decomposition
        for (size_t k = 0; k < n; ++k) {
            // Find the pivot (maximum element)
            size_t maxRow = k, maxCol = k;
            for (size_t i = k; i < n; ++i) {
                for (size_t j = k; j < n; ++j) {
                    if (std::abs(A[i][j]) > std::abs(A[maxRow][maxCol])) {
                        maxRow = i;
                        maxCol = j;
                    }
                }
            }

            // Swap rows
            std::swap(A[k], A[maxRow]);
            std::swap(rowPiv[k], rowPiv[maxRow]);

            // Swap columns
            for (size_t i = 0; i < n; ++i) {
                std::swap(A[i][k], A[i][maxCol]);
            }
            std::swap(colPiv[k], colPiv[maxCol]);

            // Check for singularity
            if (std::abs(A[k][k]) < 1e-12) {
                std::cerr << "Singular matrix detected, no solution." << std::endl;
                return {};
            }

            // Gaussian elimination
            for (size_t i = k + 1; i < n; ++i) {
                A[i][k] /= A[k][k];
                for (size_t j = k + 1; j < n; ++j) {
                    A[i][j] -= A[i][k] * A[k][j];
                }
            }
        }

        // Apply row permutation to the right-hand side vector b
        std::vector<double> b_permuted(n);
        for (size_t i = 0; i < n; ++i) {
            b_permuted[i] = b[rowPiv[i]];
        }

        // Forward substitution (solve L * y = b)
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < i; ++j) {
                b_permuted[i] -= A[i][j] * b_permuted[j];
            }
        }

        // Back substitution (solve U * x = y)
        for (int i = n - 1; i >= 0; --i) {
            for (size_t j = i + 1; j < n; ++j) {
                b_permuted[i] -= A[i][j] * b_permuted[j];
            }
            b_permuted[i] /= A[i][i];
        }

        // Reorder solution based on column pivoting
        std::vector<double> x(n);
        for (size_t i = 0; i < n; ++i) {
            x[colPiv[i]] = b_permuted[i];
        }

        return x;
    }
};
