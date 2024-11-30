class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 4) {  // Need at least 4 points for cubic fitting
      coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0;
      return;
    }

    // More robust normalization
    float t0 = data[0].timestamp;
    float tMax = data[count-1].timestamp;
    float timeScale = tMax - t0;

    // Prevent division by zero
    if (timeScale == 0) {
      coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0;
      return;
    }

    // Normalized time
    std::vector<float> normTimestamps(count);
    std::vector<float> values(count);
    for (int i = 0; i < count; i++) {
      normTimestamps[i] = (data[i].timestamp - t0) / timeScale;
      values[i] = data[i].value;
    }

    // Use Eigen for more robust matrix solving
    Eigen::MatrixXf A(count, 4);
    Eigen::VectorXf b(count);

    // Construct design matrix
    for (int i = 0; i < count; i++) {
      float t = normTimestamps[i];
      A(i, 0) = pow(t, 3);
      A(i, 1) = pow(t, 2);
      A(i, 2) = t;
      A(i, 3) = 1;
      b(i) = values[i];
    }

    // Weighted least squares with robust weight calculation
    Eigen::VectorXf weights(count);
    calculateWeights(values, weights);

    // Apply weights to the design matrix and target vector
    A.col(0) *= weights;
    A.col(1) *= weights;
    A.col(2) *= weights;
    A.col(3) *= weights;
    b *= weights;

    // Solve using QR decomposition for numerical stability
    Eigen::Vector4f solution = A.colPivHouseholderQr().solve(b);

    // Scale coefficients back to original time scale
    coeffs[0] = solution(0) / pow(timeScale, 3);   // a3
    coeffs[1] = solution(1) / pow(timeScale, 2);   // a2
    coeffs[2] = solution(2) / timeScale;           // a1
    coeffs[3] = solution(3);                       // a0

    // Iterative refinement with adaptive learning rate
    refineCoefficients(data, count, coeffs, t0, timeScale);
  }

private:
  // Robust weight calculation to reduce impact of outliers
  void calculateWeights(const std::vector<float>& values, Eigen::VectorXf& weights) {
    // Median Absolute Deviation (MAD) based robust weighting
    std::vector<float> sortedValues = values;
    std::sort(sortedValues.begin(), sortedValues.end());
    float median = sortedValues[sortedValues.size() / 2];
    
    std::vector<float> absDeviations(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
      absDeviations[i] = std::abs(values[i] - median);
    }
    std::sort(absDeviations.begin(), absDeviations.end());
    float madValue = absDeviations[absDeviations.size() / 2];

    // Tukey's Biweight function for robust weighting
    for (size_t i = 0; i < values.size(); ++i) {
      float u = std::abs(values[i] - median) / (6 * madValue);
      if (u <= 1) {
        weights(i) = pow(1 - u * u, 2);
      } else {
        weights(i) = 0;  // Assign zero weight to extreme outliers
      }
    }
  }

  // Adaptive iterative refinement
  void refineCoefficients(DataPoint* data, int count, float* coeffs, 
                          float t0, float timeScale) {
    float learningRate = 0.1;
    int maxIterations = 5;

    for (int iter = 0; iter < maxIterations; ++iter) {
      float totalError = 0;
      std::vector<float> gradients(4, 0);

      // Calculate residuals and gradients
      for (int i = 0; i < count; ++i) {
        float normT = (data[i].timestamp - t0) / timeScale;
        float predictedValue = 
          coeffs[0] * pow(normT, 3) + 
          coeffs[1] * pow(normT, 2) + 
          coeffs[2] * normT + 
          coeffs[3];
        
        float residual = data[i].value - predictedValue;
        totalError += residual * residual;

        // Compute gradients
        gradients[0] -= 2 * residual * pow(normT, 3);
        gradients[1] -= 2 * residual * pow(normT, 2);
        gradients[2] -= 2 * residual * normT;
        gradients[3] -= 2 * residual;
      }

      // Adaptive learning rate
      learningRate *= (totalError < 1e-6) ? 0.5 : 1.0;

      // Update coefficients
      for (int j = 0; j < 4; ++j) {
        coeffs[j] += learningRate * gradients[j] / count;
      }

      // Early stopping
      if (totalError < 1e-6) break;
    }
  }
};
