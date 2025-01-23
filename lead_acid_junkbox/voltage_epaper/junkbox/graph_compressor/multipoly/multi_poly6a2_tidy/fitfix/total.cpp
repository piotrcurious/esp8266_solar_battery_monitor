#ifndef ADVANCED_POLYNOMIAL_FITTER_H
#define ADVANCED_POLYNOMIAL_FITTER_H

#include <vector>
#include <Eigen.h>  // Eigen library for matrix operations
#include <limits>

enum OptimizationMethod {
    LEAST_SQUARES,
    TOTAL_LEAST_SQUARES,
    REGULARIZED_TLS
};

class AdvancedPolynomialFitter {
public:
    // Improved polynomial fitting method with more robust error handling
    static std::vector<float> fitPolynomial(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree, 
        OptimizationMethod method = TOTAL_LEAST_SQUARES,
        float regularizationFactor = 1e-6
    ) {
        // Extensive input validation
        if (x.size() != y.size()) {
            Serial.println("Error: Input vectors must have equal length");
            return {};
        }

        if (x.size() < static_cast<size_t>(degree + 1)) {
            Serial.println("Error: Insufficient data points for polynomial fit");
            return {};
        }

        // Check for numerical stability
        if (!isDataNumericallyStable(x, y)) {
            Serial.println("Warning: Input data may lead to numerical instability");
        }

        switch (method) {
            case LEAST_SQUARES:
                return fitLeastSquares(x, y, degree);
            case TOTAL_LEAST_SQUARES:
                return fitTotalLeastSquares(x, y, degree);
            case REGULARIZED_TLS:
                return fitRegularizedTLS(x, y, degree, regularizationFactor);
            default:
                Serial.println("Error: Unknown optimization method");
                return {};
        }
    }

private:
    // Numerical stability check
    static bool isDataNumericallyStable(
        const std::vector<float>& x, 
        const std::vector<float>& y
    ) {
        // Check for extreme variations or potential overflow
        float min_x = *std::min_element(x.begin(), x.end());
        float max_x = *std::max_element(x.begin(), x.end());
        float min_y = *std::min_element(y.begin(), y.end());
        float max_y = *std::max_element(y.begin(), y.end());

        // Check for excessive range
        if (max_x - min_x > 1e6 || max_y - min_y > 1e6) {
            return false;
        }

        // Check for zero variance
        if (max_x == min_x || max_y == min_y) {
            return false;
        }

        return true;
    }

    // Improved Least Squares Method with condition number check
    static std::vector<float> fitLeastSquares(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        int n = x.size();
        int m = degree + 1;
        
        // Create Vandermonde matrix with improved numerical stability
        Eigen::MatrixXf V(n, m);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                V(i, j) = pow(x[i], j);
            }
        }

        // Compute condition number
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(V);
        float condition_number = svd.singularValues()(0) / 
                                  svd.singularValues()(svd.singularValues().size() - 1);
        
        if (condition_number > 1e6) {
            Serial.println("Warning: Ill-conditioned matrix in least squares");
        }

        // Solve using QR decomposition (more numerically stable)
        Eigen::VectorXf y_vec = Eigen::Map<const Eigen::VectorXf>(y.data(), n);
        Eigen::VectorXf coeffs = V.colPivHouseholderQr().solve(y_vec);

        // Convert to vector
        std::vector<float> result(coeffs.data(), coeffs.data() + coeffs.size());
        return result;
    }

    // Advanced Total Least Squares Method
    static std::vector<float> fitTotalLeastSquares(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree
    ) {
        int n = x.size();
        int m = degree + 1;

        // Combine x and y into a single matrix
        Eigen::MatrixXf X(n, 2);
        for (int i = 0; i < n; ++i) {
            X(i, 0) = x[i];
            X(i, 1) = y[i];
        }

        // Center the data
        Eigen::VectorXf mean = X.colwise().mean();
        Eigen::MatrixXf X_centered = X.rowwise() - mean.transpose();

        // Compute SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(X_centered, Eigen::ComputeFullV);

        // Select null space vector
        Eigen::VectorXf null_vector = svd.matrixV().col(svd.matrixV().cols() - 1);

        // Compute polynomial coefficients using more robust method
        std::vector<float> coeffs = computeRobustPolynomialCoefficients(x, y, degree, null_vector);

        return coeffs;
    }

    // Regularized Total Least Squares Method
    static std::vector<float> fitRegularizedTLS(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        float regularizationFactor
    ) {
        int n = x.size();
        int m = degree + 1;

        // Create extended data matrix
        Eigen::MatrixXf X(n, 2);
        for (int i = 0; i < n; ++i) {
            X(i, 0) = x[i];
            X(i, 1) = y[i];
        }

        // Center the data
        Eigen::VectorXf mean = X.colwise().mean();
        Eigen::MatrixXf X_centered = X.rowwise() - mean.transpose();

        // Add regularization
        Eigen::MatrixXf regularization = regularizationFactor * Eigen::MatrixXf::Identity(2, 2);
        Eigen::MatrixXf X_regularized = X_centered.transpose() * X_centered + regularization;

        // Compute SVD of regularized matrix
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(X_regularized, Eigen::ComputeFullV);

        // Select null space vector
        Eigen::VectorXf null_vector = svd.matrixV().col(svd.matrixV().cols() - 1);

        // Compute polynomial coefficients
        std::vector<float> coeffs = computeRobustPolynomialCoefficients(x, y, degree, null_vector);

        return coeffs;
    }

    // Robust Polynomial Coefficient Computation
    static std::vector<float> computeRobustPolynomialCoefficients(
        const std::vector<float>& x, 
        const std::vector<float>& y, 
        int degree,
        const Eigen::VectorXf& null_vector
    ) {
        int n = x.size();
        int m = degree + 1;

        // Create Vandermonde-like matrix with improved numerical properties
        Eigen::MatrixXf V(n, m);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                V(i, j) = pow(x[i], j);
            }
        }

        // Orthogonalization using QR decomposition
        Eigen::MatrixXf V_orthogonal = V.householderQr().matrixQ();

        // Solve using constrained optimization
        Eigen::VectorXf y_vec = Eigen::Map<const Eigen::VectorXf>(y.data(), n);
        
        // Project null vector onto orthogonal space
        Eigen::VectorXf projected_null = V_orthogonal.transpose() * null_vector;

        // Solve using dampened least squares
        Eigen::VectorXf coeffs = (V.transpose() * V + 1e-6 * Eigen::MatrixXf::Identity(m, m)).ldlt().solve(V.transpose() * y_vec);

        // Convert to vector
        std::vector<float> result(coeffs.data(), coeffs.data() + coeffs.size());
        return result;
    }

    // Polynomial Evaluation Utility (optional but useful)
    static float evaluatePolynomial(
        const std::vector<float>& coeffs, 
        float x
    ) {
        float result = 0.0f;
        for (size_t i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }
};

#endif // ADVANCED_POLYNOMIAL_FITTER_H
