#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

class AdvancedPolynomialFitter {
private:
    // Aligned memory allocator for better SIMD performance
    template<typename T>
    static T* alignedAlloc(size_t n) {
        void* ptr = nullptr;
        if (posix_memalign(&ptr, 32, n * sizeof(T)) != 0) return nullptr;
        return static_cast<T*>(ptr);
    }

    // Cache-friendly matrix class with contiguous memory layout
    class Matrix {
    private:
        std::unique_ptr<double[], void(*)(void*)> data;
        size_t rows_, cols_;

    public:
        Matrix(size_t rows, size_t cols) : 
            data(alignedAlloc<double>(rows * cols), free),
            rows_(rows), cols_(cols) {
            std::fill_n(data.get(), rows * cols, 0.0);
        }

        double& operator()(size_t i, size_t j) { return data[i * cols_ + j]; }
        const double& operator()(size_t i, size_t j) const { return data[i * cols_ + j]; }
        double* ptr() { return data.get(); }
        const double* ptr() const { return data.get(); }
        size_t rows() const { return rows_; }
        size_t cols() const { return cols_; }
    };

    // Pre-computed powers lookup table
    class PowerTable {
    private:
        Matrix powers_;
        const size_t max_degree_;

    public:
        PowerTable(const std::vector<float>& x, size_t degree) :
            powers_(x.size(), degree + 1), max_degree_(degree) {
            
            #pragma omp parallel for schedule(static)
            for (size_t i = 0; i < x.size(); ++i) {
                powers_(i, 0) = 1.0;
                double xi = x[i];
                for (size_t j = 1; j <= degree; ++j) {
                    powers_(i, j) = powers_(i, j-1) * xi;
                }
            }
        }

        double get(size_t point_idx, size_t power) const {
            return powers_(point_idx, power);
        }
    };

    // Optimized Householder transformation
    static void applyHouseholderBlock(Matrix& A, std::vector<double>& b,
                                    const std::vector<double>& v, double beta,
                                    size_t col, size_t block_size) {
        const size_t m = A.rows();
        const size_t n = A.cols();
        constexpr size_t CACHE_LINE = 64 / sizeof(double);
        
        // Process matrix A in blocks for better cache utilization
        #pragma omp parallel for schedule(static)
        for (size_t j = col; j < n; j += block_size) {
            const size_t end_j = std::min(j + block_size, n);
            alignas(32) double sums[CACHE_LINE] = {0};
            
            // Compute partial sums for current block
            for (size_t jj = j; jj < end_j; ++jj) {
                double sum = 0.0;
                for (size_t i = 0; i < v.size(); ++i) {
                    sum += v[i] * A(col + i, jj);
                }
                sums[jj - j] = sum * beta;
            }

            // Apply transformation for current block
            for (size_t i = 0; i < v.size(); ++i) {
                for (size_t jj = j; jj < end_j; ++jj) {
                    A(col + i, jj) -= sums[jj - j] * v[i];
                }
            }
        }

        // Apply to vector b
        double b_sum = 0.0;
        for (size_t i = 0; i < v.size(); ++i) {
            b_sum += v[i] * b[col + i];
        }
        b_sum *= beta;
        
        for (size_t i = 0; i < v.size(); ++i) {
            b[col + i] -= b_sum * v[i];
        }
    }

    // Optimized QR solver using blocked Householder
    static std::vector<double> solveQR(Matrix& A, std::vector<double>& b) {
        const size_t m = A.rows();
        const size_t n = A.cols();
        constexpr size_t BLOCK_SIZE = 32;  // Tune based on cache size

        std::vector<double> tau(n);  // Storage for Householder scalars

        // QR decomposition with blocking
        for (size_t k = 0; k < std::min(m - 1, n); k += BLOCK_SIZE) {
            const size_t block_end = std::min(k + BLOCK_SIZE, std::min(m - 1, n));
            
            for (size_t j = k; j < block_end; ++j) {
                // Extract column j
                std::vector<double> col(m - j);
                for (size_t i = j; i < m; ++i) {
                    col[i - j] = A(i, j);
                }

                // Compute Householder transformation
                double norm = 0.0;
                for (const double& val : col) {
                    norm += val * val;
                }
                norm = std::sqrt(norm);

                if (norm > 1e-10) {
                    double alpha = (col[0] >= 0) ? -norm : norm;
                    double r = std::sqrt(0.5 * ((alpha * alpha) - (alpha * col[0])));
                    col[0] = (col[0] - alpha) / (2.0 * r);
                    
                    for (size_t i = 1; i < col.size(); ++i) {
                        col[i] /= (2.0 * r);
                    }

                    // Apply transformation
                    applyHouseholderBlock(A, b, col, -2.0, j, BLOCK_SIZE);
                    tau[j] = 2.0 * r / alpha;
                }
            }
        }

        return backsolveTriangular(A, b);
    }

    // Optimized back substitution
    static std::vector<double> backsolveTriangular(const Matrix& R, const std::vector<double>& b) {
        const size_t n = R.cols();
        std::vector<double> x(n);
        
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            #pragma omp simd reduction(+:sum)
            for (size_t j = i + 1; j < n; ++j) {
                sum += R(i, j) * x[j];
            }
            x[i] = (b[i] - sum) / R(i, i);
        }
        
        return x;
    }

public:
    static std::vector<float> fitPolynomial(const std::vector<float>& x,
                                          const std::vector<float>& y,
                                          int degree,
                                          OptimizationMethod method = OptimizationMethod::NORMAL_EQUATION) {
        if (x.size() != y.size() || x.empty() || degree < 1) {
            return {};
        }

        const size_t n = x.size();
        const size_t m = degree + 1;

        // Pre-compute powers of x for better locality
        PowerTable powers(x, degree);

        // Construct normal equations with better cache utilization
        Matrix ATA(m, m);
        std::vector<double> ATy(m, 0.0);

        constexpr size_t TILE_SIZE = 32;  // Tune based on cache size
        
        // Tiled matrix multiplication for better cache usage
        #pragma omp parallel for collapse(2) schedule(static)
        for (size_t j = 0; j < m; j += TILE_SIZE) {
            for (size_t k = 0; k < m; k += TILE_SIZE) {
                const size_t j_end = std::min(j + TILE_SIZE, m);
                const size_t k_end = std::min(k + TILE_SIZE, m);
                
                for (size_t i = 0; i < n; ++i) {
                    for (size_t jj = j; jj < j_end; ++jj) {
                        const double power_j = powers.get(i, jj);
                        #pragma omp simd
                        for (size_t kk = k; kk < k_end; ++kk) {
                            ATA(jj, kk) += power_j * powers.get(i, kk);
                        }
                    }
                }
            }
        }

        // Compute A^T * y with SIMD optimization
        #pragma omp parallel for schedule(static)
        for (size_t j = 0; j < m; ++j) {
            double sum = 0.0;
            #pragma omp simd reduction(+:sum)
            for (size_t i = 0; i < n; ++i) {
                sum += powers.get(i, j) * y[i];
            }
            ATy[j] = sum;
        }

        // Solve using blocked Householder QR
        std::vector<double> coeffs = solveQR(ATA, ATy);

        // Convert to float
        return std::vector<float>(coeffs.begin(), coeffs.end());
    }
};
