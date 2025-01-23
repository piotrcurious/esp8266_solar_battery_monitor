std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                                           OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); 
    Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Initialize coefficient accumulators
    std::vector<double> coeffs(m, 0.0);
    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);

    // Superposition: Incrementally accumulate ATA and ATy contributions
    for (size_t i = 0; i < n; ++i) {
        std::vector<double> row(m, 1.0);
        for (size_t j = 1; j < m; ++j) {
            row[j] = row[j - 1] * x[i];  // Compute powers of x
        }

        // Add contribution of current data point to ATA and ATy
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += row[j] * y[i];
            for (size_t k = 0; k < m; ++k) {
                ATA[j][k] += row[j] * row[k];
            }
        }
    }

    // Additive state decomposition: Solve coefficients iteratively
    for (size_t i = 0; i < m; ++i) {
        coeffs[i] = ATy[i];
        for (size_t j = 0; j < i; ++j) {
            coeffs[i] -= ATA[i][j] * coeffs[j];
        }
        coeffs[i] /= ATA[i][i];  // Normalize by diagonal element
    }

    // Convert coefficients to float and return
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
