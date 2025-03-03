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
    double eta = k11 + k22 - 2 * k12;
    if (eta <= 0) {
        // Ensure numerical stability
        eta = 1e-12;
    }
    
    // Store old alpha values
    double a1p_old = alpha[i1];      // α⁺ for first point
    double a1m_old = alpha[i1+n];    // α⁻ for first point
    double a2p_old = alpha[i2];      // α⁺ for second point
    double a2m_old = alpha[i2+n];    // α⁻ for second point
    
    // Compute net alpha values for the pair
    double s1 = a1p_old - a1m_old;
    double s2 = a2p_old - a2m_old;
    double s_total = s1 + s2;  // This sum remains constant for the pair

    // Compute the update step using the difference in errors
    double delta = (r1 - r2) / eta;
    double s2_new = s2 + delta;
    
    // --- FIXED BOUNDS CALCULATION ---
    // For the pair update, s₂_new must lie in:
    //   L = max(–C, s_total – C)  and  H = min(C, s_total + C)
    double L = std::max(-C, s_total - C);
    double H = std::min(C, s_total + C);
    if (s2_new < L) s2_new = L;
    if (s2_new > H) s2_new = H;
    
    double s1_new = s_total - s2_new;
    // --- END FIX ---

    // Convert net values back into separate α⁺ and α⁻
    double a1p_new, a1m_new, a2p_new, a2m_new;
    
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
    
    // Ensure each component is within [0, C]
    if (a1p_new > C) a1p_new = C;
    if (a1m_new > C) a1m_new = C;
    if (a2p_new > C) a2p_new = C;
    if (a2m_new > C) a2m_new = C;
    
    // If changes are not significant, do nothing
    if (fabs(a1p_new - a1p_old) < tol && fabs(a1m_new - a1m_old) < tol &&
        fabs(a2p_new - a2p_old) < tol && fabs(a2m_new - a2m_old) < tol) {
        return 0;
    }
    
    // Update alpha values for both points
    alpha[i1] = a1p_new;
    alpha[i1+n] = a1m_new;
    alpha[i2] = a2p_new;
    alpha[i2+n] = a2m_new;
    
    // Calculate changes in net alpha values for bias update
    double delta1 = (a1p_new - a1p_old) - (a1m_new - a1m_old);
    double delta2 = (a2p_new - a2p_old) - (a2m_new - a2m_old);
    
    // Update bias term b using KKT conditions
    double b1 = b, b2 = b;
    if (a1p_new > 0 && a1p_new < C) {
        b1 = y1 - F1 - epsilon - delta1 * k11 - delta2 * k12;
    } else if (a1m_new > 0 && a1m_new < C) {
        b1 = y1 - F1 + epsilon - delta1 * k11 - delta2 * k12;
    }
    
    if (a2p_new > 0 && a2p_new < C) {
        b2 = y2 - F2 - epsilon - delta1 * k12 - delta2 * k22;
    } else if (a2m_new > 0 && a2m_new < C) {
        b2 = y2 - F2 + epsilon - delta1 * k12 - delta2 * k22;
    }
    
    // Use the average of the two if possible
    if ((a1p_new > 0 && a1p_new < C) || (a1m_new > 0 && a1m_new < C) ||
        (a2p_new > 0 && a2p_new < C) || (a2m_new > 0 && a2m_new < C)) {
        b = (b1 + b2) / 2.0;
    }
    
    // Update the function values for all training examples
    for (size_t i = 0; i < n; ++i) {
        f[i] += delta1 * K[i1][i] + delta2 * K[i2][i];
    }
    
    return 1;
}
