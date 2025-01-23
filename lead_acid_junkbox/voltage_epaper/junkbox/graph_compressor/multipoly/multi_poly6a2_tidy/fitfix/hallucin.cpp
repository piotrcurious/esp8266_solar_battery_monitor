std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method ) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Initialize coefficients to zero
    std::vector<double> coeffs(m, 0.0);

    // Solve for each basis function individually (superposition principle)
    for (size_t j = 0; j < m; ++j) {
        double sum_xy = 0.0;
        double sum_xx = 0.0;

        for (size_t i = 0; i < n; ++i) {
            double xi_j = pow(x[i], j);  // Basis function x^j
            sum_xy += xi_j * y[i];       // Weighted sum of y values
            sum_xx += xi_j * xi_j;       // Weighted sum of basis function squared
        }

        if (sum_xx != 0.0) {
            coeffs[j] = sum_xy / sum_xx;  // Solve for coefficient independently
        }
    }

    // Convert coefficients to float
    std::vector<float> result(coeffs.begin(), coeffs.end());

    switch (method) {
        case GRADIENT_DESCENT:
            // Implement gradient descent here if needed
            break;
        case LEVENBERG_MARQUARDT:
            result = levenbergMarquardt(result, x, y, degree);
            break;
        case NELDER_MEAD:
            // Implement Nelder-Mead here if needed
            break;
        default: 
            break;
    }

    return result;
}
