class AdvancedPolynomialFitter {
public:
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
                                   OptimizationMethod method = NO_OPTIMIZATION) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};  // Invalid input
        }

        size_t n = x.size();
        size_t m = degree + 1;

        // Use Legendre polynomials as basis functions for better numerical stability
        std::vector<std::vector<double>> basis(m, std::vector<double>(n));
        computeLegendrePolynomials(x, basis);

        // Compute coefficients using superposition principle
        std::vector<double> coeffs(m);
        for (size_t j = 0; j < m; ++j) {
            double numerator = 0.0;
            double denominator = 0.0;
            
            for (size_t i = 0; i < n; ++i) {
                numerator += basis[j][i] * y[i];
                denominator += basis[j][i] * basis[j][i];
            }
            
            coeffs[j] = (denominator != 0.0) ? numerator / denominator : 0.0;
        }

        // Convert to standard polynomial coefficients
        std::vector<double> standardCoeffs = convertLegendreToPowerBasis(coeffs);
        
        // Convert to float for return value
        std::vector<float> result(standardCoeffs.begin(), standardCoeffs.end());

        // Apply optimization if requested
        switch (method) {
            case LEVENBERG_MARQUARDT:
                result = levenbergMarquardt(result, x, y, degree);
                break;
            case GRADIENT_DESCENT:
            case NELDER_MEAD:
                // Implementation for other methods could be added here
                break;
            default:
                break;
        }

        return result;
    }

private:
    void computeLegendrePolynomials(const std::vector<float>& x, std::vector<std::vector<double>>& basis) {
        size_t n = x.size();
        size_t m = basis.size();

        // Scale x to [-1, 1] for numerical stability
        double xMin = *std::min_element(x.begin(), x.end());
        double xMax = *std::max_element(x.begin(), x.end());
        double scale = 2.0 / (xMax - xMin);
        double offset = -(xMax + xMin) / (xMax - xMin);

        // P₀(x) = 1
        for (size_t i = 0; i < n; ++i) {
            basis[0][i] = 1.0;
        }

        if (m > 1) {
            // P₁(x) = x
            for (size_t i = 0; i < n; ++i) {
                double xScaled = x[i] * scale + offset;
                basis[1][i] = xScaled;
            }

            // Recurrence relation: (n+1)Pₙ₊₁(x) = (2n+1)xPₙ(x) - nPₙ₋₁(x)
            for (size_t k = 2; k < m; ++k) {
                for (size_t i = 0; i < n; ++i) {
                    double xScaled = x[i] * scale + offset;
                    basis[k][i] = ((2.0 * k - 1.0) * xScaled * basis[k-1][i] - 
                                 (k - 1.0) * basis[k-2][i]) / k;
                }
            }
        }
    }

    std::vector<double> convertLegendreToPowerBasis(const std::vector<double>& legendreCoeffs) {
        size_t degree = legendreCoeffs.size() - 1;
        std::vector<double> powerCoeffs(degree + 1, 0.0);

        // Conversion matrix from Legendre to power basis
        std::vector<std::vector<double>> conversionMatrix(degree + 1, std::vector<double>(degree + 1, 0.0));
        
        // P₀(x) = 1
        conversionMatrix[0][0] = 1.0;
        
        if (degree > 0) {
            // P₁(x) = x
            conversionMatrix[1][1] = 1.0;
            
            // Generate higher degree conversions
            for (size_t n = 2; n <= degree; ++n) {
                for (size_t k = 0; k <= n; ++k) {
                    if (n > 1) {
                        if (k > 0) {
                            conversionMatrix[n][k] += (2.0 * n - 1.0) * conversionMatrix[n-1][k-1] / n;
                        }
                        if (k <= n - 2) {
                            conversionMatrix[n][k] -= (n - 1.0) * conversionMatrix[n-2][k] / n;
                        }
                    }
                }
            }
        }

        // Multiply conversion matrix by Legendre coefficients
        for (size_t i = 0; i <= degree; ++i) {
            for (size_t j = 0; j <= degree; ++j) {
                powerCoeffs[i] += conversionMatrix[j][i] * legendreCoeffs[j];
            }
        }

        return powerCoeffs;
    }
};
