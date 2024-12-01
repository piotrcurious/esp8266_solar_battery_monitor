class AdvancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0;
      return;
    }

    // Normalize timestamps to avoid numerical instability
    float t0 = data[0].timestamp;
    float normT[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      normT[i] = data[i].timestamp - t0;
    }

    // Initialize weight vector
    float weights[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      weights[i] = 1.0; // Start with uniform weights
    }

    // Initial polynomial coefficients
    float prevCoeffs[4] = {0, 0, 0, 0};
    float residuals[MAX_TEMPORARY] = {0};

    // Iterative refinement loop
    for (int iter = 0; iter < 5; iter++) {
      // Reset sums
      float sumT[4] = {0}, sumV = 0, sumTV[4] = {0};

      // Weighted least squares computation
      for (int i = 0; i < count; i++) {
        float t = normT[i];
        float v = data[i].value * weights[i];

        sumT[0] += weights[i] * pow(t, 6);  // t^6 for higher order fitting
        sumT[1] += weights[i] * pow(t, 4);  // t^4
        sumT[2] += weights[i] * pow(t, 2);  // t^2
        sumT[3] += weights[i];             // t^0

        sumV += v;
        sumTV[0] += weights[i] * pow(t, 3) * data[i].value;
        sumTV[1] += weights[i] * pow(t, 2) * data[i].value;
        sumTV[2] += weights[i] * t * data[i].value;
        sumTV[3] += weights[i] * data[i].value;
      }

      // Solve for coefficients
      coeffs[0] = sumTV[0] / sumT[0];  // a3
      coeffs[1] = sumTV[1] / sumT[1];  // a2
      coeffs[2] = sumTV[2] / sumT[2];  // a1
      coeffs[3] = sumTV[3] / sumT[3];  // a0

      // Calculate residuals and update weights
      for (int i = 0; i < count; i++) {
        float t = normT[i];
        float fittedValue = coeffs[0] * pow(t, 3) + coeffs[1] * pow(t, 2) + coeffs[2] * t + coeffs[3];
        residuals[i] = data[i].value - fittedValue;

        // Adjust weights to emphasize points with higher residuals
        weights[i] = 1.0 / (1.0 + fabs(residuals[i]));
      }

      // Check for convergence
      if (fabs(coeffs[0] - prevCoeffs[0]) < 1e-4 &&
          fabs(coeffs[1] - prevCoeffs[1]) < 1e-4 &&
          fabs(coeffs[2] - prevCoeffs[2]) < 1e-4 &&
          fabs(coeffs[3] - prevCoeffs[3]) < 1e-4) {
        break;
      }

      // Update previous coefficients
      memcpy(prevCoeffs, coeffs, sizeof(prevCoeffs));
    }
  }
};
