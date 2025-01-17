#include "AdvancedPolynomialFitter.hpp"
#include <algorithm>
#include <cmath>

// Calculate MSE using Welford's method
double AdvancedPolynomialFitter::calculateMSE(const std::vector<float>& coeffs, 
                   const std::vector<float>& x, const std::vector<float>& y) {
    double mean = 0, M2 = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        double pred = 0;
        for (size_t j = 0; j < coeffs.size(); ++j)
            pred += coeffs[j] * pow(x[i], j);
        double err = pred - y[i];
        double sqErr = err * err, delta = sqErr - mean;
        mean += delta / (i + 1);
        M2 += delta * (sqErr - mean);
    }
    return mean;
}

// Fit polynomial to data
std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) return {};
    std::vector<float> x_norm(x);
    double x_min = *std::min_element(x.begin(), x.end()), 
           x_max = *std::max_element(x.begin(), x.end());
#ifdef REVERSED_NORMALIZATION
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_max](double val) { return val - x_max; });
#else
    std::transform(x.begin(), x.end(), x_norm.begin(), [x_min](double val) { return val - x_min; });
#endif

    size_t n = x_norm.size(), m = degree + 1;

//    std::vector<std::vector<double>> A(n, std::vector<double>(m));
//    for (size_t i = 0; i < n; ++i)
//        for (size_t j = 0, xi = 1; j < m; ++j, xi *= x_norm[i])
//            A[i][j] = xi;

 // Construct the Vandermonde matrix
        std::vector<std::vector<double>> A(n, std::vector<double>(m, 0.0));
        for (size_t i = 0; i < n; ++i) {
            double xi = 1.0;
            for (size_t j = 0; j < m; ++j) {
                A[i][j] = xi;
                xi *= x_norm[i];
            }
        }

    std::vector<std::vector<double>> ATA(m, std::vector<double>(m));
    std::vector<double> ATy(m);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += A[i][j] * y[i];
            for (size_t k = 0; k < m; ++k)
                ATA[j][k] += A[i][j] * A[i][k];
        }

    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);
    std::vector<float> result(coeffs.begin(), coeffs.end());
    if (method == LEVENBERG_MARQUARDT)
        result = levenbergMarquardt(result, x_norm, y, degree);
    return result;
}

// Fit segmented polynomials
std::vector<float> AdvancedPolynomialFitter::fitSegmentedPolynomials(const std::vector<float>& x, const std::vector<float>& y, int degree, int segments) {
    std::vector<float> result;
    size_t segSize = x.size() / segments;
    for (int i = 0; i < segments; ++i) {
        size_t start = i * segSize, end = (i == segments - 1) ? x.size() : start + segSize;
        auto coeffs = fitPolynomial({x.begin() + start, x.begin() + end}, {y.begin() + start, y.begin() + end}, degree);
        result.insert(result.end(), coeffs.begin(), coeffs.end());
    }
    return result;
}

// Levenberg-Marquardt algorithm
std::vector<float> AdvancedPolynomialFitter::levenbergMarquardt(std::vector<float>& coeffs, const std::vector<float>& x, const std::vector<float>& y, int degree) {
    const int maxIters = 200;
    const double lambdaInit = 0.1, lambdaFactor = 2, tol = 1e-6;
    double lambda = lambdaInit, prevMSE = calculateMSE(coeffs, x, y);

    for (int iter = 0; iter < maxIters; ++iter) {
        std::vector<std::vector<double>> J(x.size(), std::vector<double>(degree + 1));
        std::vector<double> residuals(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            for (int j = 0, xi = 1; j <= degree; ++j, xi *= x[i])
                J[i][j] = xi;
            double pred = 0;
            for (int j = 0; j <= degree; ++j) pred += coeffs[j] * pow(x[i], j);
            residuals[i] = y[i] - pred;
        }

        std::vector<std::vector<double>> JTJ(degree + 1, std::vector<double>(degree + 1));
        std::vector<double> JTr(degree + 1);
        for (size_t i = 0; i < x.size(); ++i)
            for (int j = 0; j <= degree; ++j) {
                for (int k = 0; k <= degree; ++k)
                    JTJ[j][k] += J[i][j] * J[i][k];
                JTr[j] += J[i][j] * residuals[i];
            }

        for (int j = 0; j <= degree; ++j) JTJ[j][j] += lambda;

        auto delta = solveLinearSystem(JTJ, JTr);
        auto newCoeffs = coeffs;
        for (int j = 0; j <= degree; ++j) newCoeffs[j] += delta[j];
        double newMSE = calculateMSE(newCoeffs, x, y);

        if (std::abs(prevMSE - newMSE) < tol) break;
        if (newMSE < prevMSE) {
            lambda /= lambdaFactor;
            coeffs = newCoeffs;
            prevMSE = newMSE;
        } else lambda *= lambdaFactor;
    }
    return coeffs;
}

// Solve linear system using Gaussian elimination
std::vector<double> AdvancedPolynomialFitter::solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b) {
    size_t n = A.size();
    for (size_t k = 0; k < n; ++k) {
        for (size_t i = k + 1; i < n; ++i)
            if (fabs(A[i][k]) > fabs(A[k][k])) {
                std::swap(A[k], A[i]);
                std::swap(b[k], b[i]);
            }

        for (size_t i = k + 1; i < n; ++i) {
            double factor = A[i][k] / A[k][k];
            for (size_t j = k; j < n; ++j) A[i][j] -= factor * A[k][j];
            b[i] -= factor * b[k];
        }
    }

    std::vector<double> x(n);
    for (int i = n - 1; i >= 0; --i) {
        x[i] = b[i];
        for (size_t j = i + 1; j < n; ++j) x[i] -= A[i][j] * x[j];
        x[i] /= A[i][i];
    }
    return x;
}
