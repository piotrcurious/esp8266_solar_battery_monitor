class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 4) {
      // Not enough points for cubic fitting
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Normalize timestamps
    float t0 = data[0].timestamp;
    float tMax = data[count-1].timestamp;
    float timeSpan = tMax - t0;

    if (timeSpan == 0) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Gauss-Jordan elimination for solving linear system
    float A[4][4] = {0};
    float B[4] = {0};

    // Construct normal equations matrix
    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;
      float v = data[i].value;

      // Weight calculation (simple inverse distance)
      float weight = 1.0f / (1.0f + std::abs(v - computeMean(data, count)));

      A[0][0] += weight * pow(t, 6);
      A[0][1] += weight * pow(t, 5);
      A[0][2] += weight * pow(t, 4);
      A[0][3] += weight * pow(t, 3);

      A[1][0] += weight * pow(t, 5);
      A[1][1] += weight * pow(t, 4);
      A[1][2] += weight * pow(t, 3);
      A[1][3] += weight * pow(t, 2);

      A[2][0] += weight * pow(t, 4);
      A[2][1] += weight * pow(t, 3);
      A[2][2] += weight * pow(t, 2);
      A[2][3] += weight * t;

      A[3][0] += weight * pow(t, 3);
      A[3][1] += weight * pow(t, 2);
      A[3][2] += weight * t;
      A[3][3] += weight;

      B[0] += weight * v * pow(t, 3);
      B[1] += weight * v * pow(t, 2);
      B[2] += weight * v * t;
      B[3] += weight * v;
    }

    // Gauss-Jordan elimination
    for (int i = 0; i < 4; ++i) {
      // Find pivot
      int maxRow = i;
      for (int k = i + 1; k < 4; ++k) {
        if (std::abs(A[k][i]) > std::abs(A[maxRow][i])) {
          maxRow = k;
        }
      }

      // Swap rows
      std::swap(A[i], A[maxRow]);
      std::swap(B[i], B[maxRow]);

      // Make diagonal 1
      float diag = A[i][i];
      if (std::abs(diag) < 1e-10) {
        // Singular matrix, fallback to simple averaging
        coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0.0f;
        return;
      }

      for (int j = i; j < 4; ++j) {
        A[i][j] /= diag;
      }
      B[i] /= diag;

      // Eliminate column
      for (int k = 0; k < 4; ++k) {
        if (k != i) {
          float factor = A[k][i];
          for (int j = i; j < 4; ++j) {
            A[k][j] -= factor * A[i][j];
          }
          B[k] -= factor * B[i];
        }
      }
    }

    // Rescale coefficients
    coeffs[0] = B[0] / pow(timeSpan, 3);   // a3
    coeffs[1] = B[1] / pow(timeSpan, 2);   // a2
    coeffs[2] = B[2] / timeSpan;           // a1
    coeffs[3] = B[3];                      // a0

    // Optional: Simple refinement step
    refineCoefficients(data, count, coeffs, t0, timeSpan);
  }

private:
  // Compute mean to help with weighting
  float computeMean(DataPoint* data, int count) {
    float sum = 0;
    for (int i = 0; i < count; ++i) {
      sum += data[i].value;
    }
    return sum / count;
  }

  // Lightweight refinement
  void refineCoefficients(DataPoint* data, int count, float* coeffs, 
                          float t0, float timeSpan) {
    const int maxIter = 3;
    for (int iter = 0; iter < maxIter; ++iter) {
      float totalError = 0;
      std::vector<float> errors(count, 0);

      // Compute residuals
      for (int i = 0; i < count; ++i) {
        float t = (data[i].timestamp - t0) / timeSpan;
        float predicted = coeffs[0] * pow(t, 3) + 
                          coeffs[1] * pow(t, 2) + 
                          coeffs[2] * t + 
                          coeffs[3];
        errors[i] = data[i].value - predicted;
        totalError += std::abs(errors[i]);
      }

      // Break if error is very small
      if (totalError < 1e-6) break;

      // Adjust coefficients
      for (int i = 0; i < count; ++i) {
        float t = (data[i].timestamp - t0) / timeSpan;
        coeffs[0] += 0.01f * errors[i] * pow(t, 3);
        coeffs[1] += 0.01f * errors[i] * pow(t, 2);
        coeffs[2] += 0.01f * errors[i] * t;
        coeffs[3] += 0.01f * errors[i];
      }
    }
  }
};
