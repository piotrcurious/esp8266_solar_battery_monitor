std::vector<float> AdvancedPolynomialFitter::fitsvm(std::vector<float>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;
    
    // Normalize input data to improve numerical stability
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    double x_range = x_max - x_min;
    
    std::vector<double> x_norm(n);
    for (size_t i = 0; i < n; ++i) {
        x_norm[i] = (x[i] - x_min) / x_range;
    }
    
    // Create feature matrix (Vandermonde matrix)
    std::vector<std::vector<double>> X(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        X[i][0] = 1.0;  // Constant term
        for (size_t j = 1; j < m; ++j) {
            X[i][j] = pow(x_norm[i], j);  // Use normalized x values
        }
    }
    
    // Hyperparameters - adjusted for better fit
    const double C = 100.0;     // Regularization parameter - higher value for tighter fit
    const double epsilon = 0.01; // Epsilon in epsilon-SVR model - increased for better generalization
    const double tol = 1e-5;    // Tolerance for convergence - slightly relaxed
    const int maxIter = 5000;   // Maximum iterations - reduced to prevent overfitting
    
    // Initialize alphas and function values
    std::vector<double> alpha(2*n, 0.0);  // Combined alpha (alpha+ and alpha-)
    std::vector<double> f(n, 0.0);        // Function values
    double b = 0.0;                       // Bias term
    
    // Compute kernel matrix for linear case (dot products of feature vectors)
    std::vector<std::vector<double>> K(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i; j < n; ++j) {  // Only compute upper triangle due to symmetry
            double dot = 0.0;
            for (size_t k = 0; k < m; ++k) {
                dot += X[i][k] * X[j][k];
            }
            K[i][j] = dot;
            if (i != j) {
                K[j][i] = dot;  // Fill lower triangle
            }
        }
    }
    
    // SMO Algorithm for SVR
    int iter = 0;
    int numChanged = 0;
    bool examineAll = true;
    
    while ((numChanged > 0 || examineAll) && iter < maxIter) {
        numChanged = 0;
        
        if (examineAll) {
            // Loop over all training examples
            for (size_t i = 0; i < n; ++i) {
                numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b);
            }
        } else {
            // Loop over examples where alpha is not at bounds
            for (size_t i = 0; i < n; ++i) {
                if ((alpha[i] > 0 && alpha[i] < C) || (alpha[i+n] > 0 && alpha[i+n] < C)) {
                    numChanged += examineExample(i, n, alpha, y, f, epsilon, C, K, tol, b);
                }
            }
        }
        
        if (examineAll) {
            examineAll = false;
        } else if (numChanged == 0) {
            examineAll = true;
        }
        
        iter++;
        
        // Debug progress
        if (iter % 100 == 0) {
            Serial.print("Iteration: ");
            Serial.print(iter);
            Serial.print(", Changed: ");
            Serial.println(numChanged);
        }
    }
    
    // Calculate polynomial coefficients directly from the primal form
    std::vector<double> w(m, 0.0);
    
    // For each dimension in feature space
    for (size_t j = 0; j < m; ++j) {
        for (size_t i = 0; i < n; ++i) {
            // alpha[i] is alpha+, alpha[i+n] is alpha-
            double alpha_diff = alpha[i] - alpha[i+n];
            w[j] += alpha_diff * X[i][j];
        }
    }
    
    // Apply denormalization to coefficients
    std::vector<double> denorm_coeffs = denormalizeCoefficients(w, x_min, x_range, degree);
    
    // Convert to float
    std::vector<float> result(denorm_coeffs.begin(), denorm_coeffs.end());
    
    // Debug output
    Serial.print("SVM Coefficients: ");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.print(result[i]); Serial.print(" ");
    }
    Serial.println();
    
    // Calculate and print RMSE
    double rmse = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double pred = 0.0;
        for (size_t j = 0; j < m; ++j) {
            pred += result[j] * pow(x[i], j);
        }
        double err = pred - y[i];
        rmse += err * err;
    }
    rmse = sqrt(rmse / n);
    Serial.print("RMSE: ");
    Serial.println(rmse);
    
    return result;
}

// Helper function for examining training examples in SMO
int AdvancedPolynomialFitter::examineExample(
    size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
    std::vector<double>& f, double epsilon, double C,
    const std::vector<std::vector<double>>& K, double tol, double& b) {
    
    // Get error
    double y2 = y[i2];
    double F2 = f[i2];  // f already includes Wx part
    double r2 = F2 + b - y2;  // Add bias term separately
    
    // Check if KKT conditions are violated
    bool kkt_violated = false;
    
    // Check upper and lower bound violations
    if ((r2 > epsilon && alpha[i2] > 0) || (r2 < -epsilon && alpha[i2+n] > 0) ||
        (r2 < -epsilon && alpha[i2] < C) || (r2 > epsilon && alpha[i2+n] < C)) {
        kkt_violated = true;
    }
    
    if (kkt_violated) {
        // Find index with maximum objective function change
        double max_delta = 0.0;
        size_t i1 = i2;
        
        // First heuristic - find example with maximum error difference
        for (size_t j = 0; j < n; ++j) {
            if ((alpha[j] > 0 && alpha[j] < C) || (alpha[j+n] > 0 && alpha[j+n] < C)) {
                double F1 = f[j];
                double r1 = F1 + b - y[j];
                double delta = fabs(r1 - r2);
                
                if (delta > max_delta) {
                    max_delta = delta;
                    i1 = j;
                }
            }
        }
        
        if (i1 != i2) {
            // Try to optimize with chosen index
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                return 1;
            }
        }
        
        // Second heuristic - random start
        size_t rand_start = rand() % n;
        for (size_t j = 0; j < n; ++j) {
            i1 = (rand_start + j) % n;
            if ((alpha[i1] > 0 && alpha[i1] < C) || (alpha[i1+n] > 0 && alpha[i1+n] < C)) {
                if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                    return 1;
                }
            }
        }
        
        // Third heuristic - go through all examples
        rand_start = rand() % n;
        for (size_t j = 0; j < n; ++j) {
            i1 = (rand_start + j) % n;
            if (optimizePair(i1, i2, n, alpha, y, f, epsilon, C, K, tol, b)) {
                return 1;
            }
        }
    }
    
    return 0;
}

// Helper function for joint optimization of two alphas in SMO
int AdvancedPolynomialFitter::optimizePair(
    size_t i1, size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
    std::vector<double>& f, double epsilon, double C,
    const std::vector<std::vector<double>>& K, double tol, double& b) {
    
    if (i1 == i2) return 0;
    
    // Get errors and kernel values
    double y1 = y[i1];
    double y2 = y[i2];
    double F1 = f[i1];
    double F2 = f[i2];
    double r1 = F1 + b - y1;
    double r2 = F2 + b - y2;
    
    double k11 = K[i1][i1];
    double k12 = K[i1][i2];
    double k22 = K[i2][i2];
    double eta = k11 + k22 - 2 * k12;  // Correct formula for eta
    
    // Check if the quadratic is positive definite
    if (eta <= 0) {
        // Use a small positive value to ensure numerical stability
        eta = 1e-12;
    }
    
    // Store old alpha values
    double a1p_old = alpha[i1];      // alpha+_1
    double a1m_old = alpha[i1+n];    // alpha-_1
    double a2p_old = alpha[i2];      // alpha+_2
    double a2m_old = alpha[i2+n];    // alpha-_2
    
    // Calculate objective function derivatives
    double g1 = r1 + epsilon;
    double g2 = r2 + epsilon;
    double g3 = r1 - epsilon;
    double g4 = r2 - epsilon;
    
    // Compute new alpha values using a more stable approach
    double s1 = a1p_old - a1m_old;  // Net alpha for first point
    double s2 = a2p_old - a2m_old;  // Net alpha for second point
    
    // Update s2 using the optimality condition
    double s2_new = s2 + (g1 - g2) / eta;
    
    // Enforce constraints
    double L = std::max(-C, s1 - C);
    double H = std::min(C, s1 + C);
    
    if (s2_new < L) s2_new = L;
    if (s2_new > H) s2_new = H;
    
    // Calculate corresponding s1 value
    double s1_new = s1 + (s2 - s2_new);
    
    // Convert back to alpha+ and alpha- components
    double a1p_new, a1m_new, a2p_new, a2m_new;
    
    // For simplicity, keep one of the components at zero (complementarity)
    if (s1_new >= 0) {
        a1p_new = s1_new;
        a1m_new = 0;
    } else {
        a1p_new = 0;
        a1m_new = -s1_new;
    }
    
    if (s2_new >= 0) {
        a2p_new = s2_new;
        a2m_new = 0;
    } else {
        a2p_new = 0;
        a2m_new = -s2_new;
    }
    
    // Ensure bounds
    if (a1p_new > C) a1p_new = C;
    if (a1m_new > C) a1m_new = C;
    if (a2p_new > C) a2p_new = C;
    if (a2m_new > C) a2m_new = C;
    
    // If no significant change, return
    if (fabs(a1p_new - a1p_old) < tol && fabs(a1m_new - a1m_old) < tol &&
        fabs(a2p_new - a2p_old) < tol && fabs(a2m_new - a2m_old) < tol) {
        return 0;
    }
    
    // Update alpha values
    alpha[i1] = a1p_new;
    alpha[i1+n] = a1m_new;
    alpha[i2] = a2p_new;
    alpha[i2+n] = a2m_new;
    
    // Calculate changes in alpha values
    double delta1 = (a1p_new - a1p_old) - (a1m_new - a1m_old);
    double delta2 = (a2p_new - a2p_old) - (a2m_new - a2m_old);
    
    // Update bias term using KKT conditions
    double b1 = b;
    double b2 = b;
    
    // Update bias if alpha1 is not at bounds
    if (a1p_new > 0 && a1p_new < C) {
        b1 = y1 - F1 - epsilon - delta1 * k11 - delta2 * k12;
    } else if (a1m_new > 0 && a1m_new < C) {
        b1 = y1 - F1 + epsilon - delta1 * k11 - delta2 * k12;
    }
    
    // Update bias if alpha2 is not at bounds
    if (a2p_new > 0 && a2p_new < C) {
        b2 = y2 - F2 - epsilon - delta1 * k12 - delta2 * k22;
    } else if (a2m_new > 0 && a2m_new < C) {
        b2 = y2 - F2 + epsilon - delta1 * k12 - delta2 * k22;
    }
    
    // Average the two bias values
    if ((a1p_new > 0 && a1p_new < C) || (a1m_new > 0 && a1m_new < C) ||
        (a2p_new > 0 && a2p_new < C) || (a2m_new > 0 && a2m_new < C)) {
        b = (b1 + b2) / 2.0;
    } else {
        // If all alphas at bounds, use average of limits
        b = (b1 + b2) / 2.0;
    }
    
    // Update function values
    for (size_t i = 0; i < n; ++i) {
        f[i] += delta1 * K[i1][i] + delta2 * K[i2][i];
    }
    
    return 1;
}

// Helper function to denormalize coefficients
std::vector<double> AdvancedPolynomialFitter::denormalizeCoefficients(
    const std::vector<double>& w, double x_min, double x_range, int degree) {
    
    std::vector<double> coeffs(degree + 1, 0.0);
    
    // Use binomial expansion to compute the denormalized coefficients
    // For a polynomial of form: w0 + w1*x + w2*x^2 + ... + wn*x^n
    // where x is normalized as x = (t - x_min) / x_range
    // and t is the original variable
    
    for (int i = 0; i <= degree; ++i) {
        for (int j = i; j <= degree; ++j) {
            // Coefficient for x^i in the expansion of (t - x_min)^j / x_range^j
            double binCoeff = binomialCoefficient(j, i);
            double term = binCoeff * pow(-x_min, j-i) / pow(x_range, j) * w[j];
            coeffs[i] += term;
        }
    }
    
    return coeffs;
}

// Helper function to calculate binomial coefficient (n choose k)
double AdvancedPolynomialFitter::binomialCoefficient(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    double res = 1;
    k = std::min(k, n - k);  // Optimize calculation
    
    for (int i = 0; i < k; ++i) {
        res *= (n - i);
        res /= (i + 1);
    }
    
    return res;
}
