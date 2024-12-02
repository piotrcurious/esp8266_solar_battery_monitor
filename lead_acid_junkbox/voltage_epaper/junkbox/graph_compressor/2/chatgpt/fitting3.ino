class AdvancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Normalize timestamps to reduce numerical instability
    float t0 = data[0].timestamp;
    float normalizedT[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      normalizedT[i] = data[i].timestamp - t0;
    }

    // Step 1: Initialize coefficients using least squares
    float initialCoeffs[4];
    leastSquaresFit(normalizedT, data, count, initialCoeffs);

    // Step 2: Refine coefficients using adaptive binary search
    int8_t quantizedCoeffs[4];
    adaptiveBinarySearchFit(normalizedT, data, count, initialCoeffs, quantizedCoeffs);

    // Step 3: Final refinement using Monte Carlo sampling
    monteCarloRefinement(normalizedT, data, count, quantizedCoeffs);

    // Step 4: Convert quantized coefficients back to floating-point
    coeffs[0] = dequantizeCoefficient(quantizedCoeffs[0]);
    coeffs[1] = dequantizeCoefficient(quantizedCoeffs[1]);
    coeffs[2] = dequantizeCoefficient(quantizedCoeffs[2]);
    coeffs[3] = dequantizeCoefficient(quantizedCoeffs[3]);
  }

private:
  void leastSquaresFit(float* t, DataPoint* data, int count, float* coeffs) {
    // Vandermonde matrix and RHS vector initialization
    float vandermonde[4][4] = {0};
    float rhs[4] = {0};

    for (int i = 0; i < count; i++) {
      float ti = t[i];
      float vi = data[i].value;
      float t2 = ti * ti;
      float t3 = t2 * ti;

      vandermonde[0][0] += t3 * t2;
      vandermonde[0][1] += t3;
      vandermonde[0][2] += t2;
      vandermonde[0][3] += ti;

      vandermonde[1][1] += t2;
      vandermonde[1][2] += ti;
      vandermonde[1][3] += 1;

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

    // Solve linear system
    solveLinearSystem(vandermonde, rhs, coeffs);
  }

  void adaptiveBinarySearchFit(float* t, DataPoint* data, int count, float* initialCoeffs, int8_t* quantizedCoeffs) {
    for (int i = 0; i < 4; i++) {
      quantizedCoeffs[i] = quantizeCoefficient(initialCoeffs[i]);
    }

    for (int coeffIndex = 0; coeffIndex < 4; coeffIndex++) {
      int8_t low = quantizedCoeffs[coeffIndex] - 16;
      int8_t high = quantizedCoeffs[coeffIndex] + 16;

      while (low <= high) {
        int8_t mid = (low + high) / 2;
        quantizedCoeffs[coeffIndex] = mid;

        float error = computeResidualErrorQuantized(t, data, count, quantizedCoeffs);
        if (error < settings.maxError) {
          low = mid + 1;  // Expand search range
        } else {
          high = mid - 1; // Reduce search range
        }
      }
    }
  }

  void monteCarloRefinement(float* t, DataPoint* data, int count, int8_t* quantizedCoeffs) {
    int8_t bestCoeffs[4];
    float bestError = computeResidualErrorQuantized(t, data, count, quantizedCoeffs);

    for (int i = 0; i < 100; i++) { // Number of Monte Carlo iterations
      int8_t candidateCoeffs[4];
      for (int j = 0; j < 4; j++) {
        candidateCoeffs[j] = quantizedCoeffs[j] + random(-2, 2); // Small random perturbation
      }

      float candidateError = computeResidualErrorQuantized(t, data, count, candidateCoeffs);
      if (candidateError < bestError) {
        bestError = candidateError;
        for (int j = 0; j < 4; j++) {
          bestCoeffs[j] = candidateCoeffs[j];
        }
      }
    }

    for (int j = 0; j < 4; j++) {
      quantizedCoeffs[j] = bestCoeffs[j];
    }
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
      error += residual * residual;
    }
    return error;
  }

  void solveLinearSystem(float matrix[4][4], float rhs[4], float* solution) {
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

    for (int i = 3; i >= 0; i--) {
      solution[i] = rhs[i];
      for (int j = i + 1; j < 4; j++) {
        solution[i] -= matrix[i][j] * solution[j];
      }
    }
  }
};
