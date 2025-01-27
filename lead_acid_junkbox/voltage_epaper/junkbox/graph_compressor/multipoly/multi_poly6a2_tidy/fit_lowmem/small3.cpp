#include <vector>
#include <cmath>

// Function to solve the normal equation using Householder reflections with minimal memory
void solveLinearSystemHouseholder(std::vector<double>& ATA, std::vector<double>& ATy, int m) {
    for (int k = 0; k < m; ++k) {
        double norm_x = 0.0;
        for (int i = k; i < m; ++i) {
            norm_x += ATA[i * m + k] * ATA[i * m + k];
        }
        norm_x = std::sqrt(norm_x);

        double alpha = (ATA[k * m + k] > 0) ? -norm_x : norm_x;
        double r = std::sqrt(0.5 * (alpha * alpha - ATA[k * m + k] * alpha));
        std::vector<double> v(m - k);
        v[0] = (ATA[k * m + k] - alpha) / (2.0 * r);
        for (int i = k + 1; i < m; ++i) {
            v[i - k] = ATA[i * m + k] / (2.0 * r);
        }

        // Apply Householder transformation to ATA (in-place update)
        for (int j = k; j < m; ++j) {
            double dot = 0.0;
            for (int i = k; i < m; ++i) {
                dot += v[i - k] * ATA[i * m + j];
            }
            for (int i = k; i < m; ++i) {
                ATA[i * m + j] -= 2.0 * v[i - k] * dot;
            }
        }

        // Apply transformation to ATy
        double dot_b = 0.0;
        for (int i = k; i < m; ++i) {
            dot_b += v[i - k] * ATy[i];
        }
        for (int i = k; i < m; ++i) {
            ATy[i] -= 2.0 * v[i - k] * dot_b;
        }
    }

    // Back substitution to get solution
    for (int i = m - 1; i >= 0; --i) {
        double sum = 0.0;
        for (int j = i + 1; j < m; ++j) {
            sum += ATA[i * m + j] * ATy[j];
        }
        ATy[i] = (ATy[i] - sum) / ATA[i * m + i];
    }
}

// Optimized polynomial fitting function
std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;

    // Allocate flattened ATA matrix and ATy vector
    std::vector<double> ATA(m * m, 0.0);
    std::vector<double> ATy(m, 0.0);

    // Compute ATA and ATy on the fly without storing full matrices
    for (size_t i = 0; i < n; ++i) {
        double xi = 1.0;
        std::vector<double> row(m);
        for (size_t j = 0; j < m; ++j) {
            row[j] = xi;
            xi *= x[i];
        }
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += row[j] * y[i];
            for (size_t k = 0; k <= j; ++k) {
                ATA[j * m + k] += row[j] * row[k];
            }
        }
    }

    // Fill symmetric elements
    for (size_t j = 0; j < m; ++j) {
        for (size_t k = j + 1; k < m; ++k) {
            ATA[j * m + k] = ATA[k * m + j];
        }
    }

    // Solve the normal equation using Householder method
    solveLinearSystemHouseholder(ATA, ATy, m);

    // Convert coefficients to float
    std::vector<float> result(ATy.begin(), ATy.end());
    return result;
}
