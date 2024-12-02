class OptimizedQuantizedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Normalize timestamps to avoid numerical instability
    float t0 = data[0].timestamp;
    float normalizedT[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      normalizedT[i] = data[i].timestamp - t0;
    }

    // Step 1: Coarse least-squares fit
    float floatCoeffs[4];
    leastSquaresFit(normalizedT, data, count, floatCoeffs);

    // Step 2: Adaptive quantization refinement
    int8_t quantizedCoeffs[4];
    refineQuantizedFit(normalizedT, data, count, floatCoeffs, quantizedCoeffs);

    // Step 3: Convert quantized coefficients back to floating-point
    coeffs[0] = dequantizeCoefficient(quantizedCoeffs[0]);
    coeffs[1] = dequantizeCoefficient(quantizedCoeffs[1]);
    coeffs[2] = dequantizeCoefficient(quantizedCoeffs[2]);
    coeffs[3] = dequantizeCoefficient(quantizedCoeffs[3]);
  }

private:
  void leastSquaresFit(float* t, DataPoint* data, int count, float* coeffs) {
    // Initialize Vandermonde matrix and RHS vector
    float vandermonde[4][4] = {0};
    float rhs[4] = {0};

    for (int i = 0; i < count; i++) {
      float ti = t[i];
      float vi = data[i].value;

      float t2 = ti * ti;
      float t3 = t2 * ti;

      // Fill Vandermonde matrix
      vandermonde[0][0] += t3 * t2;
      vandermonde[0][1] += t3;
      vandermonde[0][2] += t2;
      vandermonde[0][3] += ti;

      vandermonde[1][1] += t2;
      vandermonde[1][2] += ti;
      vandermonde[1][3] += 1;

      // Fill RHS vector
      rhs[0] += t3 * vi;
      rhs[1] += t2 * vi;
      rhs[2] += ti * vi;
      rhs[3] += vi;
    }

    // Symmetric matrix completion
    for (int row = 1; row < 4; row++) {
      for (int col = 0; col < row; col++) {
        vandermonde[row][col] = vandermonde[col][row];
      }
    }

    // Solve linear system for floating-point coefficients
    solveLinearSystem(vandermonde, rhs, coeffs);
  }

  void refineQuantizedFit(float* t, DataPoint* data, int count, float* initialCoeffs, int8_t* quantizedCoeffs) {
    float bestError = std::numeric_limits<float>::max();
    int8_t bestQuantized[4];

    // Adaptive range based on initial coefficients
    int8_t range = 16;  // Search range around initial quantization
    int8_t q0 = quantizeCoefficient(initialCoeffs[0]);
    int8_t q1 = quantizeCoefficient(initialCoeffs[1]);
    int8_t q2 = quantizeCoefficient(initialCoeffs[2]);
    int8_t q3 = quantizeCoefficient(initialCoeffs[3]);

    for (int8_t dq3 = -range; dq3 <= range; dq3++) {
      for (int8_t dq2 = -range; dq2 <= range; dq2++) {
        for (int8_t dq1 = -range; dq1 <= range; dq1++) {
          for (int8_t dq0 = -range; dq0 <= range; dq0++) {
            int8_t candidate[4] = {q3 + dq3, q2 + dq2, q1 + dq1, q0 + dq0};
            float error = computeResidualErrorQuantized(t, data, count, candidate);
            if (error < bestError) {
              bestError = error;
              bestQuantized[0] = candidate[0];
              bestQuantized[1] = candidate[1];
              bestQuantized[2] = candidate[2];
              bestQuantized[3] = candidate[3];
            }
          }
        }
      }
    }

    // Assign best quantized coefficients
    quantizedCoeffs[0] = bestQuantized[0];
    quantizedCoeffs[1] = bestQuantized[1];
    quantizedCoeffs[2] = bestQuantized[2];
    quantizedCoeffs[3] = bestQuantized[3];
  }

  float computeResidualErrorQuantized(float* t, DataPoint* data, int count, int8_t* quantizedCoeffs) {
    float a3 = dequantizeCoefficient(quantizedCoeffs[0]);
    float a2 = dequantizeCoefficient(quantizedCoeffs[1]);
    float a1 = dequantizeCoefficient(quantizedCoeffs[2]);
    float a0 = dequantizeCoefficient(quantizedCoeffs[3]);

    float error = 0;
    for (int i = 0; i < count; i++) {
      float ti = t[i];
      float predicted = a3 * ti * ti * ti + a2 * ti * ti + a1 * ti + a0;
      float residual = data[i].value - predicted;
      error += residual * residual;  // Squared error
    }
    return error;
  }

  void solveLinearSystem(float matrix[4][4], float rhs[4], float* solution) {
    // Gaussian elimination for solving 4x4 system
    for (int i = 0; i < 4; i++) {
      float pivot = matrix[i][i];
      for (int j = i; j < 4; j++) matrix[i][j] /= pivot;
      rhs[i] /= pivot;

      for (int k = i + 1; k < 4; k++) {
        float factor = matrix[k][i];
        for (int j = i; j < 4; j++) matrix[k][j] -= factor * matrix[i][j];
        rhs[k] -= factor * rhs[i];
      }
    }

    // Back substitution
    for (int i = 3; i >= 0; i--) {
      solution[i] = rhs[i];
      for (int j = i + 1; j < 4; j++) {
        solution[i] -= matrix[i][j] * solution[j];
      }
    }
  }
};
