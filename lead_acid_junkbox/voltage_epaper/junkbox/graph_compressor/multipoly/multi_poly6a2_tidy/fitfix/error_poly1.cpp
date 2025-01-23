std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree, 
                                                           OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); 
    Serial.println(x[x.size() - 1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Construct the Vandermonde matrix
    std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        for (size_t j = 0; j < m; ++j) {
            A[i][j] = xi;
            xi *= x[i];
        }
    }

    // Construct the normal equation: (A^T * A) * coeffs = A^T * y
    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += A[i][j] * y[i];
            for (size_t k = 0; k < m; ++k) {
                ATA[j][k] += A[i][j] * A[i][k];
            }
        }
    }

    // Initial polynomial fitting using normal equations
    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);

    // Chambolle-Pock iterative refinement
    const int max_iterations = 50;
    const double tau = 1.0 / spectralNorm(ATA);  // Step size based on matrix norm
    const double sigma = 1.0 / tau;
    const double lambda = 1.0e-4;  // Regularization factor

    std::vector<double> error(n, 0.0);
    std::vector<double> dual(m, 0.0);
    std::vector<double> primal(m, 0.0);
    std::vector<double> prev_primal = coeffs;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Compute the error polynomial based on the residuals
        for (size_t i = 0; i < n; ++i) {
            double approx = 0.0;
            for (size_t j = 0; j < m; ++j) {
                approx += A[i][j] * coeffs[j];
            }
            error[i] = y[i] - approx;
        }

        // Update dual variable
        for (size_t j = 0; j < m; ++j) {
            dual[j] += sigma * (ATy[j] - ATA[j][j] * coeffs[j]);
        }

        // Update primal variable
        for (size_t j = 0; j < m; ++j) {
            primal[j] = coeffs[j] - tau * (ATA[j][j] * coeffs[j] - ATy[j] + dual[j]);
        }

        // Apply relaxation
        for (size_t j = 0; j < m; ++j) {
            coeffs[j] = primal[j] + lambda * (primal[j] - prev_primal[j]);
            prev_primal[j] = primal[j];
        }

        // Check convergence based on error
        double error_norm = 0.0;
        for (size_t i = 0; i < n; ++i) {
            error_norm += error[i] * error[i];
        }
        if (sqrt(error_norm) < 1e-6) break;
    }

    // Convert coefficients to float
    std::vector<float> result(coeffs.begin(), coeffs.end());

    return result;
}
