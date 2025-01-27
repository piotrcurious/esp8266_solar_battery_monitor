#include <vector>
#include <cmath>

// Function to solve linear system using the Householder transformation
std::vector<double> solveLinearSystemHouseholder(const std::vector<std::vector<double>>& ATA, const std::vector<double>& ATy, int m) {
    std::vector<std::vector<double>> R = ATA;
    std::vector<double> b = ATy;

    for (int k = 0; k < m; ++k) {
        double norm_x = 0.0;
        for (int i = k; i < m; ++i) {
            norm_x += R[i][k] * R[i][k];
        }
        norm_x = std::sqrt(norm_x);

        double alpha = (R[k][k] > 0) ? -norm_x : norm_x;
        double r = std::sqrt(0.5 * (alpha * alpha - R[k][k] * alpha));
        std::vector<double> v(m - k, 0.0);
        v[0] = (R[k][k] - alpha) / (2.0 * r);
        for (int i = k + 1; i < m; ++i) {
            v[i - k] = R[i][k] / (2.0 * r);
        }

        // Apply Householder transformation to R
        for (int j = k; j < m; ++j) {
            double dot = 0.0;
            for (int i = k; i < m; ++i) {
                dot += v[i - k] * R[i][j];
            }
            for (int i = k; i < m; ++i) {
                R[i][j] -= 2.0 * v[i - k] * dot;
            }
        }

        // Apply transformation to b
        double dot_b = 0.0;
        for (int i = k; i < m; ++i) {
            dot_b += v[i - k] * b[i];
        }
        for (int i = k; i < m; ++i) {
            b[i] -= 2.0 * v[i - k] * dot_b;
        }
    }

    // Back substitution
    std::vector<double> x(m, 0.0);
    for (int i = m - 1; i >= 0; --i) {
        double sum = 0.0;
        for (int j = i + 1; j < m; ++j) {
            sum += R[i][j] * x[j];
        }
        x[i] = (b[i] - sum) / R[i][i];
    }

    return x;
}

// Optimized polynomial fitting function
std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Compute ATA and ATy on the fly without storing the full Vandermonde matrix
    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        std::vector<double> row(m);
        for (size_t j = 0; j < m; ++j) {
            row[j] = xi;
            xi *= x[i];
        }
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += row[j] * y[i];
            for (size_t k = 0; k < m; ++k) {
                ATA[j][k] += row[j] * row[k];
            }
        }
    }

    // Solve the normal equation using Householder transformation
    std::vector<double> coeffs = solveLinearSystemHouseholder(ATA, ATy, m);

    // Convert coefficients to float
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
