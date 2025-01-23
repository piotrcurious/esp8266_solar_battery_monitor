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

        // Check condition number of ATA to detect potential instability
        if (conditionNumber(ATA) > 1e12) {
            std::cerr << "Warning: The system appears to be ill-conditioned." << std::endl;
            return {};  // Return empty if system is ill-conditioned
        }

        return solveLinearSystem(ATA, ATy);
    }

private:
    // Solve linear system using QR Decomposition
    static std::vector<double> solveLinearSystem(
        const std::vector<std::vector<double>>& A, 
        const std::vector<double>& b
    ) {
        size_t n = A.size();
        
        // Using Gram-Schmidt process to perform QR Decomposition
        std::vector<std::vector<double>> Q(n, std::vector<double>(n, 0.0));
        std::vector<std::vector<double>> R(n, std::vector<double>(n, 0.0));

        // Gram-Schmidt process to compute Q and R
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                Q[j][i] = A[j][i];  // Copy the matrix A into Q initially
            }

            // Compute R[i][i]
            R[i][i] = 0;
            for (size_t j = 0; j < n; ++j) {
                R[i][i] += Q[j][i] * Q[j][i];
            }
            R[i][i] = std::sqrt(R[i][i]);

            // Normalize the i-th column
            for (size_t j = 0; j < n; ++j) {
                Q[j][i] /= R[i][i];
            }

            // Compute the rest of the R matrix
            for (size_t j = i + 1; j < n; ++j) {
                R[i][j] = 0;
                for (size_t k = 0; k < n; ++k) {
                    R[i][j] += Q[k][i] * A[k][j];
                }

                // Subtract from the j-th column
                for (size_t k = 0; k < n; ++k) {
                    A[k][j] -= Q[k][i] * R[i][j];
                }
            }
        }

        // Solve R * x = Q^T * b using back-substitution
        std::vector<double> Qtb(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                Qtb[i] += Q[j][i] * b[j];
            }
        }

        // Back-substitution to solve for the coefficients
        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = Qtb[i];
            for (size_t j = i + 1; j < n; ++j) {
                x[i] -= R[i][j] * x[j];
            }
            x[i] /= R[i][i];
        }

        return x;
    }

    // Condition number calculation (ratio of largest to smallest eigenvalue)
    static double conditionNumber(const std::vector<std::vector<double>>& A) {
        size_t n = A.size();
        std::vector<double> eigenvalues = eigenDecompose(A);
        double maxEig = *std::max_element(eigenvalues.begin(), eigenvalues.end());
        double minEig = *std::min_element(eigenvalues.begin(), eigenvalues.end());
        return maxEig / minEig;
    }

    // Eigen decomposition (for simplicity, we assume symmetric matrix)
    static std::vector<double> eigenDecompose(const std::vector<std::vector<double>>& A) {
        size_t n = A.size();
        std::vector<double> eigenvalues(n, 0.0);
        
        // For simplicity, we perform an approximate eigenvalue decomposition
        // This is just a placeholder; a full eigenvalue solver would be needed here.
        for (size_t i = 0; i < n; ++i) {
            eigenvalues[i] = A[i][i];  // Diagonal elements are eigenvalues for symmetric matrices
        }
        return eigenvalues;
    }
};
