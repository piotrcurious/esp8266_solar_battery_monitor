// Optimized polynomial fitter using Householder transformation
class OptimizedPolynomialFitter {
public:
    // Function to compute A(i,j) element of Vandermonde matrix on the fly
    static double computeA(const std::vector<float>& x, size_t i, size_t j) {
        return std::pow(x[i], static_cast<double>(j));
    }

    // Function to compute (A^T * A)(i,j) element on the fly
    static double computeATA(const std::vector<float>& x, size_t i, size_t j, size_t n) {
        double sum = 0.0;
        for (size_t k = 0; k < n; ++k) {
            sum += computeA(x, k, i) * computeA(x, k, j);
        }
        return sum;
    }

    // Function to compute (A^T * y)(i) element on the fly
    static double computeATy(const std::vector<float>& x, const std::vector<float>& y, 
                           size_t i, size_t n) {
        double sum = 0.0;
        for (size_t k = 0; k < n; ++k) {
            sum += computeA(x, k, i) * y[k];
        }
        return sum;
    }

    // Householder reflection implementation
    static void applyHouseholder(std::vector<double>& vec, const std::vector<double>& u, 
                               double beta) {
        double dot = 0.0;
        for (size_t i = 0; i < u.size(); ++i) {
            dot += u[i] * vec[i];
        }
        for (size_t i = 0; i < vec.size(); ++i) {
            vec[i] -= beta * dot * u[i];
        }
    }

    // Solve linear system using Householder QR decomposition
    static std::vector<double> solveLinearSystem(const std::vector<float>& x,
                                               const std::vector<float>& y,
                                               size_t m, size_t n) {
        std::vector<double> R(m * m, 0.0);
        std::vector<double> QTy(m, 0.0);
        
        // Initialize R with ATA and QTy with ATy
        for (size_t i = 0; i < m; ++i) {
            QTy[i] = computeATy(x, y, i, n);
            for (size_t j = 0; j < m; ++j) {
                R[i * m + j] = computeATA(x, i, j, n);
            }
        }

        // Perform Householder QR decomposition
        std::vector<double> u(m);
        for (size_t k = 0; k < m - 1; ++k) {
            // Compute Householder vector
            double norm = 0.0;
            for (size_t i = k; i < m; ++i) {
                norm += R[i * m + k] * R[i * m + k];
            }
            norm = std::sqrt(norm);

            if (norm > 1e-10) {
                double alpha = (R[k * m + k] > 0) ? -norm : norm;
                u[k] = R[k * m + k] - alpha;
                double uu = u[k] * u[k];
                
                for (size_t i = k + 1; i < m; ++i) {
                    u[i] = R[i * m + k];
                    uu += u[i] * u[i];
                }

                double beta = -2.0 / uu;

                // Apply Householder reflection to R
                for (size_t j = k; j < m; ++j) {
                    std::vector<double> column(m);
                    for (size_t i = 0; i < m; ++i) {
                        column[i] = R[i * m + j];
                    }
                    applyHouseholder(column, u, beta);
                    for (size_t i = 0; i < m; ++i) {
                        R[i * m + j] = column[i];
                    }
                }

                // Apply Householder reflection to QTy
                applyHouseholder(QTy, u, beta);
            }
        }

        // Back substitution
        std::vector<double> coeffs(m);
        for (int i = m - 1; i >= 0; --i) {
            double sum = QTy[i];
            for (size_t j = i + 1; j < m; ++j) {
                sum -= R[i * m + j] * coeffs[j];
            }
            coeffs[i] = sum / R[i * m + i];
        }

        return coeffs;
    }

    static std::vector<float> fitPolynomial(const std::vector<float>& x,
                                          const std::vector<float>& y,
                                          int degree) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Solve using Householder method
        std::vector<double> coeffs = solveLinearSystem(x, y, m, n);

        // Convert coefficients to float
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }
};
