std::vector<float> AdvancedPolynomialFitter::fitPolynomialD(const std::vector<double>& x, const std::vector<float>& y, int degree,
    OptimizationMethod method) {
    if (x.size() != y.size() || x.empty() || degree < 1) {
        return {};  // Invalid input
    }
    Serial.println(x[0]); Serial.println(x[x.size()-1]);

    size_t n = x.size();
    size_t m = degree + 1;
    
    // Hyperparameters for SVR
    const double C = 1.0;        // Regularization parameter
    const double epsilon = 0.1;  // Epsilon in the epsilon-SVR model
    const double tol = 1e-3;     // Tolerance for KKT conditions
    const int maxPasses = 100;   // Maximum number of passes without alpha change
    
    // Initialize alphas (Lagrange multipliers)
    std::vector<double> alpha_plus(n, 0.0);  // Alpha plus (above the epsilon tube)
    std::vector<double> alpha_minus(n, 0.0); // Alpha minus (below the epsilon tube)
    
    // Initialize function values
    std::vector<double> f(n, 0.0);
    
    // Precompute kernel matrix (polynomial kernel)
    std::vector<std::vector<double>> K(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            // Polynomial kernel: K(x,y) = (x·y + 1)^degree
            double dot = 0.0;
            for (int d = 0; d <= degree; ++d) {
                double xi = (d == 0) ? 1.0 : pow(x[i], d);
                double xj = (d == 0) ? 1.0 : pow(x[j], d);
                dot += xi * xj;
            }
            K[i][j] = dot;
        }
    }
    
    // SMO algorithm
    int passes = 0;
    while (passes < maxPasses) {
        int num_changed_alphas = 0;
        
        for (size_t i = 0; i < n; ++i) {
            // Calculate error: Ei = f(xi) - yi
            double Ei = f[i] - y[i];
            
            // Check if the point violates KKT conditions
            if ((Ei > epsilon && alpha_minus[i] > 0) || 
                (Ei < -epsilon && alpha_plus[i] > 0) || 
                ((Ei <= epsilon && Ei >= -epsilon) && 
                 (alpha_plus[i] > 0 || alpha_minus[i] > 0))) {
                
                // Select j randomly, different from i
                size_t j;
                do {
                    j = rand() % n;
                } while (j == i);
                
                double Ej = f[j] - y[j];
                
                // Save old alphas
                double alpha_plus_i_old = alpha_plus[i];
                double alpha_minus_i_old = alpha_minus[i];
                double alpha_plus_j_old = alpha_plus[j];
                double alpha_minus_j_old = alpha_minus[j];
                
                // Compute L and H bounds
                double L1 = std::max(0.0, alpha_plus[j] - alpha_plus[i]);
                double H1 = std::min(C, C + alpha_plus[j] - alpha_plus[i]);
                double L2 = std::max(0.0, alpha_minus[j] - alpha_minus[i]);
                double H2 = std::min(C, C + alpha_minus[j] - alpha_minus[i]);
                
                if (L1 == H1 && L2 == H2) {
                    continue;
                }
                
                // Compute eta (second derivative of objective function)
                double eta = 2 * K[i][j] - K[i][i] - K[j][j];
                
                if (eta >= 0) {
                    continue;
                }
                
                // Update alpha_plus_j
                alpha_plus[j] = alpha_plus_j_old - (Ej + epsilon - Ei - epsilon) / eta;
                if (alpha_plus[j] > H1) {
                    alpha_plus[j] = H1;
                } else if (alpha_plus[j] < L1) {
                    alpha_plus[j] = L1;
                }
                
                // Update alpha_minus_j
                alpha_minus[j] = alpha_minus_j_old - (Ej - epsilon - Ei + epsilon) / eta;
                if (alpha_minus[j] > H2) {
                    alpha_minus[j] = H2;
                } else if (alpha_minus[j] < L2) {
                    alpha_minus[j] = L2;
                }
                
                if (std::abs(alpha_plus[j] - alpha_plus_j_old) < tol && 
                    std::abs(alpha_minus[j] - alpha_minus_j_old) < tol) {
                    continue;
                }
                
                // Update alpha_plus_i and alpha_minus_i
                alpha_plus[i] = alpha_plus_i_old + (alpha_plus_j_old - alpha_plus[j]);
                alpha_minus[i] = alpha_minus_i_old + (alpha_minus_j_old - alpha_minus[j]);
                
                // Update function values
                for (size_t k = 0; k < n; ++k) {
                    f[k] += (alpha_plus[i] - alpha_plus_i_old) * K[i][k] + 
                            (alpha_plus[j] - alpha_plus_j_old) * K[j][k] - 
                            (alpha_minus[i] - alpha_minus_i_old) * K[i][k] - 
                            (alpha_minus[j] - alpha_minus_j_old) * K[j][k];
                }
                
                ++num_changed_alphas;
            }
        }
        
        if (num_changed_alphas == 0) {
            ++passes;
        } else {
            passes = 0;
        }
    }
    
    // Calculate bias term
    double b = 0.0;
    int count = 0;
    
    for (size_t i = 0; i < n; ++i) {
        if (alpha_plus[i] > 0 && alpha_plus[i] < C) {
            b += y[i] + epsilon;
            for (size_t j = 0; j < n; ++j) {
                b -= (alpha_plus[j] - alpha_minus[j]) * K[i][j];
            }
            ++count;
        } else if (alpha_minus[i] > 0 && alpha_minus[i] < C) {
            b += y[i] - epsilon;
            for (size_t j = 0; j < n; ++j) {
                b -= (alpha_plus[j] - alpha_minus[j]) * K[i][j];
            }
            ++count;
        }
    }
    
    if (count > 0) {
        b /= count;
    }
    
    // Calculate polynomial coefficients
    std::vector<double> coeffs(m, 0.0);
    coeffs[0] = b;  // Bias term
    
    // For each basis function in our polynomial
    for (size_t d = 1; d < m; ++d) {
        double coef = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double xi = pow(x[i], d);
            coef += (alpha_plus[i] - alpha_minus[i]) * xi;
        }
        coeffs[d] = coef;
    }
    
    // Convert coefficients to float
    std::vector<float> result(coeffs.begin(), coeffs.end());
    return result;
}
