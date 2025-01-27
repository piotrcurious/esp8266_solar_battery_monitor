#include <Arduino.h>

class ESP32PolynomialFitter {
private:
    // Compact matrix class optimized for small memory footprint
    class Matrix {
    private:
        double* data;
        size_t rows_, cols_;
        bool owned_;

    public:
        Matrix(size_t rows, size_t cols, bool preallocated = false) : 
            rows_(rows), cols_(cols), owned_(!preallocated) {
            if (!preallocated) {
                data = (double*)ps_malloc(rows * cols * sizeof(double));
                if (data) {
                    memset(data, 0, rows * cols * sizeof(double));
                }
            }
        }

        ~Matrix() {
            if (owned_ && data) {
                free(data);
            }
        }

        void setData(double* ptr) { 
            if (!owned_) data = ptr; 
        }

        bool isValid() const { return data != nullptr; }
        double& operator()(size_t i, size_t j) { return data[i * cols_ + j]; }
        const double& operator()(size_t i, size_t j) const { return data[i * cols_ + j]; }
        size_t rows() const { return rows_; }
        size_t cols() const { return cols_; }
    };

    // Efficient power calculation with minimal memory usage
    static double computePower(double x, size_t power) {
        double result = 1.0;
        double current_x = x;
        
        while (power > 0) {
            if (power & 1) {
                result *= current_x;
            }
            current_x *= current_x;
            power >>= 1;
        }
        return result;
    }

    // Memory-efficient QR decomposition using Givens rotations
    static void applyGivensRotation(Matrix& R, std::vector<double>& b, 
                                   size_t i, size_t j, double c, double s) {
        for (size_t k = j; k < R.cols(); k++) {
            double temp = R(i,k);
            R(i,k) = c * temp + s * R(j,k);
            R(j,k) = -s * temp + c * R(j,k);
        }
        
        double temp = b[i];
        b[i] = c * temp + s * b[j];
        b[j] = -s * temp + c * b[j];
    }

    static void computeGivensRotation(double a, double b, double& c, double& s) {
        if (b == 0.0) {
            c = 1.0;
            s = 0.0;
        } else {
            double t;
            if (fabs(b) > fabs(a)) {
                t = -a / b;
                s = 1.0 / sqrt(1.0 + t * t);
                c = s * t;
            } else {
                t = -b / a;
                c = 1.0 / sqrt(1.0 + t * t);
                s = c * t;
            }
        }
    }

    // Optimized back-substitution for small systems
    static void backSubstitution(const Matrix& R, std::vector<double>& x, 
                               const std::vector<double>& b) {
        const size_t n = R.cols();
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (size_t j = i + 1; j < n; j++) {
                sum += R(i,j) * x[j];
            }
            x[i] = (b[i] - sum) / R(i,i);
        }
    }

public:
    // Error codes for better error handling
    enum class FitResult {
        SUCCESS = 0,
        INVALID_INPUT = 1,
        MEMORY_ERROR = 2,
        NUMERICAL_ERROR = 3
    };

    static FitResult fitPolynomial(const std::vector<float>& x,
                                 const std::vector<float>& y,
                                 int degree,
                                 std::vector<float>& coefficients) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return FitResult::INVALID_INPUT;
        }

        const size_t n = x.size();
        const size_t m = degree + 1;

        // Allocate matrix in PSRAM if available
        Matrix R(m, m);
        if (!R.isValid()) {
            return FitResult::MEMORY_ERROR;
        }

        std::vector<double> b(m, 0.0);
        
        // Process data points in chunks to save memory
        const size_t CHUNK_SIZE = 32;
        for (size_t chunk_start = 0; chunk_start < n; chunk_start += CHUNK_SIZE) {
            size_t chunk_end = min(chunk_start + CHUNK_SIZE, n);
            
            // Process one chunk of data points
            for (size_t i = chunk_start; i < chunk_end; i++) {
                // Compute powers efficiently
                std::vector<double> powers(m);
                powers[0] = 1.0;
                double xi = x[i];
                for (size_t j = 1; j < m; j++) {
                    powers[j] = powers[j-1] * xi;
                }

                // Update normal equations for this point
                for (size_t j = 0; j < m; j++) {
                    b[j] += powers[j] * y[i];
                    for (size_t k = j; k < m; k++) {
                        R(j,k) += powers[j] * powers[k];
                    }
                }

                // Allow other tasks to run periodically
                if ((i & 0x0F) == 0) {
                    yield();
                }
            }
        }

        // Mirror the symmetric matrix
        for (size_t i = 0; i < m; i++) {
            for (size_t j = 0; j < i; j++) {
                R(i,j) = R(j,i);
            }
        }

        // Solve using QR decomposition with Givens rotations
        for (size_t j = 0; j < m; j++) {
            for (size_t i = j + 1; i < m; i++) {
                if (R(i,j) != 0.0) {
                    double c, s;
                    computeGivensRotation(R(j,j), R(i,j), c, s);
                    applyGivensRotation(R, b, j, i, c, s);
                }
            }
            
            // Check for numerical stability
            if (fabs(R(j,j)) < 1e-10) {
                return FitResult::NUMERICAL_ERROR;
            }
            
            yield(); // Allow other tasks to run
        }

        // Solve the triangular system
        std::vector<double> solution(m);
        backSubstitution(R, solution, b);

        // Copy results to output vector
        coefficients.resize(m);
        for (size_t i = 0; i < m; i++) {
            coefficients[i] = static_cast<float>(solution[i]);
        }

        return FitResult::SUCCESS;
    }

    // Helper function to evaluate the polynomial at a point
    static float evaluatePolynomial(const std::vector<float>& coeffs, float x) {
        float result = coeffs[0];
        float power = x;
        
        for (size_t i = 1; i < coeffs.size(); i++) {
            result += coeffs[i] * power;
            power *= x;
        }
        
        return result;
    }
};
