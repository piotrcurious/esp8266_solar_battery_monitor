std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                                           OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); 
    Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Initialize matrices for QR decomposition
    std::vector<std::vector<double>> Q(n, std::vector<double>(m, 0.0));
    std::vector<std::vector<double>> R(m, std::vector<double>(m, 0.0));
    std::vector<double> b(n, 0.0);

    // Build the Vandermonde matrix row by row and apply QR decomposition incrementally
    for (size_t i = 0; i < n; ++i) {
        std::vector<double> row(m, 1.0);
        for (size_t j = 1; j < m; ++j) {
            row[j] = row[j - 1] * x[i];  // Compute powers of x
        }
        b[i] = y[i];

        // Apply Gram-Schmidt process to update Q and R
        for (size_t j = 0; j < m; ++j) {
            double dot_product = 0.0;
            for (size_t k = 0; k < n; ++k) {
                dot_product += Q[k][j] * row[j];
            }
            R[j][j] += dot_product;
            for (size_t k = 0; k < n; ++k) {
                row[j] -= Q[k][j] * dot_product;
            }
            double norm = 0.0;
            for (size_t k = 0; k < n; ++k) {
                norm += row[j] * row[j];
            }
            norm = sqrt(norm);
            for (size_t k = 0; k < n; ++k) {
                Q[k][j] = row[j] / norm;
            }
        }
    }

    // Solve for coefficients using back-substitution from the triangular matrix R
    std::vector<double> coeffs(m, 0.0);
    for (int j = m - 1; j >= 0; --j) {
        double sum = 0.0;
        for (size_t k = j + 1; k < m; ++k) {
            sum += R[j][k] * coeffs[k];
        }
        coeffs[j] = (Q[j][0] * b[j] - sum) / R[j][j];
    }

    // Convert coefficients to float and return
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
