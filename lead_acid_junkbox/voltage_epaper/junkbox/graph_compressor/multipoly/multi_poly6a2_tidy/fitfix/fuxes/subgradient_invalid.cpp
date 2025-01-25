std::vector<float> AdvancedPolynomialFitter::fitPolynomial(
    const std::vector<float>& x, 
    const std::vector<float>& y, 
    int degree, 
    OptimizationMethod method 
) { 
    if (x.size() != y.size() || x.empty() || degree < 1) { 
        return {};  // Invalid input 
    }
    Serial.println(x[0]); 
    Serial.println(x[x.size()-1]);

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

    // Solve the normal equation using Gaussian elimination
    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);

    // Convert coefficients to float
    std::vector<float> result(coeffs.begin(), coeffs.end());

    switch (method) {
        case GRADIENT_DESCENT:
            result = gradientDescent(result, x, y, degree);
            break;
        case LEVENBERG_MARQUARDT:
            result = levenbergMarquardt(result, x, y, degree);
            break;
        case NELDER_MEAD:
            result = nelderMeadOptimization(result, x, y, degree);
            break;
        case SUBGRADIENT_PROJECTION:
            result = subgradientProjection(result, x, y, degree);
            break;
        default: 
            break;
    }
    
    return result;
}

// Subgradient projection method implementation
std::vector<float> AdvancedPolynomialFitter::subgradientProjection(
    std::vector<float>& coeffs, 
    const std::vector<float>& x, 
    const std::vector<float>& y, 
    int degree
) {
    float stepSize = 0.01f;  // Step size (can be reduced gradually)
    int maxIterations = 1000;
    float tolerance = 1e-6f;

    for (int iter = 0; iter < maxIterations; ++iter) {
        std::vector<float> subgradient(degree + 1, 0.0f);

        // Compute subgradient (sum of absolute error subgradients)
        for (size_t i = 0; i < x.size(); ++i) {
            float predicted = 0.0f;
            float xi = 1.0f;
            for (int j = 0; j <= degree; ++j) {
                predicted += coeffs[j] * xi;
                xi *= x[i];
            }
            float error = predicted - y[i];

            // Subgradient for L1-norm: sign of error
            xi = 1.0f;
            float signError = (error > 0) ? 1.0f : ((error < 0) ? -1.0f : 0.0f);
            for (int j = 0; j <= degree; ++j) {
                subgradient[j] += signError * xi;
                xi *= x[i];
            }
        }

        // Update coefficients using subgradient descent
        for (int j = 0; j <= degree; ++j) {
            coeffs[j] -= stepSize * subgradient[j];

            // Project onto feasible set (e.g., bounded region [-10,10])
            if (coeffs[j] > 10.0f) coeffs[j] = 10.0f;
            else if (coeffs[j] < -10.0f) coeffs[j] = -10.0f;
        }

        // Check convergence using subgradient norm
        float subgradNorm = 0.0f;
        for (float g : subgradient) {
            subgradNorm += g * g;
        }
        if (sqrt(subgradNorm) < tolerance) {
            break;
        }

        // Optionally, reduce step size over iterations
        stepSize *= 0.99f;
    }

    return coeffs;
}
