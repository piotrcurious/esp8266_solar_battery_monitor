class AdvancedPolynomialFitter {
private:
    // Compute Householder vector and beta
    static std::pair<std::vector<double>, double> computeHouseholder(std::vector<double>& x, size_t offset) {
        double norm = 0.0;
        size_t n = x.size() - offset;
        std::vector<double> v(n);
        
        // Calculate norm of vector excluding the first offset elements
        for (size_t i = offset; i < x.size(); i++) {
            norm += x[i] * x[i];
        }
        norm = std::sqrt(norm);

        if (norm < 1e-10) {
            return {std::vector<double>(n), 0.0};
        }

        double sigma = (x[offset] >= 0) ? norm : -norm;
        double beta = sigma * (sigma + x[offset]);
        
        if (std::abs(beta) < 1e-10) {
            return {std::vector<double>(n), 0.0};
        }

        // Compute Householder vector
        v[0] = x[offset] + sigma;
        for (size_t i = 1; i < n; i++) {
            v[i] = x[offset + i];
        }
        
        // Scale vector
        for (double& vi : v) {
            vi /= v[0];
        }
        
        return {v, 2.0 / beta};
    }

    // Apply Householder transformation to matrix A and vector b
    static void applyHouseholder(std::vector<std::vector<double>>& A, std::vector<double>& b,
                                const std::vector<double>& v, double beta, size_t col) {
        size_t m = A.size();
        size_t n = A[0].size();

        // Apply to matrix A
        for (size_t j = col; j < n; j++) {
            double sum = 0.0;
            for (size_t i = 0; i < v.size(); i++) {
                sum += v[i] * A[col + i][j];
            }
            sum *= beta;
            
            for (size_t i = 0; i < v.size(); i++) {
                A[col + i][j] -= sum * v[i];
            }
        }

        // Apply to vector b
        double sum = 0.0;
        for (size_t i = 0; i < v.size(); i++) {
            sum += v[i] * b[col + i];
        }
        sum *= beta;
        
        for (size_t i = 0; i < v.size(); i++) {
            b[col + i] -= sum * v[i];
        }
    }

    // Solve upper triangular system
    static std::vector<double> backSubstitution(const std::vector<std::vector<double>>& R, 
                                              const std::vector<double>& b) {
        size_t n = R.size();
        std::vector<double> x(n);
        
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (size_t j = i + 1; j < n; j++) {
                sum += R[i][j] * x[j];
            }
            x[i] = (b[i] - sum) / R[i][i];
        }
        
        return x;
    }

    // Solve linear system using Householder QR decomposition
    static std::vector<double> solveLinearSystem(std::vector<std::vector<double>> A, 
                                               std::vector<double> b) {
        size_t m = A.size();
        size_t n = A[0].size();

        // Perform QR decomposition using Householder transformations
        for (size_t k = 0; k < std::min(m - 1, n); k++) {
            // Extract column k starting from row k
            std::vector<double> col(m - k);
            for (size_t i = k; i < m; i++) {
                col[i - k] = A[i][k];
            }

            // Compute Householder vector and beta
            auto [v, beta] = computeHouseholder(col, 0);
            
            if (beta == 0.0) continue;

            // Apply Householder transformation
            applyHouseholder(A, b, v, beta, k);
        }

        return backSubstitution(A, b);
    }

public:
    static std::vector<float> fitPolynomial(const std::vector<float>& x, 
                                          const std::vector<float>& y, 
                                          int degree,
                                          OptimizationMethod method = OptimizationMethod::NORMAL_EQUATION) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Directly construct A^T * A and A^T * y without storing A
        std::vector<double> ATy(m, 0.0);
        std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));

        // For each data point
        for (size_t i = 0; i < n; ++i) {
            // Compute powers of x[i] on the fly
            std::vector<double> powers(m);
            powers[0] = 1.0;
            for (size_t j = 1; j < m; ++j) {
                powers[j] = powers[j-1] * x[i];
            }

            // Update A^T * y
            for (size_t j = 0; j < m; ++j) {
                ATy[j] += powers[j] * y[i];
            }

            // Update A^T * A
            for (size_t j = 0; j < m; ++j) {
                for (size_t k = 0; k < m; ++k) {
                    ATA[j][k] += powers[j] * powers[k];
                }
            }
        }

        // Solve using Householder QR decomposition
        std::vector<double> coeffs = solveLinearSystem(ATA, ATy);

        // Convert coefficients to float
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }
};
