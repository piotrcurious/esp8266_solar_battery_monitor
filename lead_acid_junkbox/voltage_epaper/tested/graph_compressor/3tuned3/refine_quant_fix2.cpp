void advancedRefinement(DataPoint* data, int count, float* coeffs, 
                        float t0, float timeSpan) {
  const int maxIter = 10;           // Increase max iterations for better convergence
  const float QUANTIZATION_SCALE = 100.0f; // Scale for quantization
  const float momentum = 0.8f;      // Momentum factor for gradient descent
  float learningRate = 0.02f;       // Initial learning rate
  float velocity[4] = {0};          // Momentum for each coefficient

  for (int iter = 0; iter < maxIter; ++iter) {
    float totalError = 0;
    float gradient[4] = {0};        // Gradients for coefficients

    // Compute residuals and gradients
    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;
      float predicted = coeffs[0] * pow(t, 3) + 
                        coeffs[1] * pow(t, 2) + 
                        coeffs[2] * t + 
                        coeffs[3];
      
      // Amplify errors at end-points
      float endPointMultiplier = (i == 0 || i == count - 1) ? 2.0f : 1.0f;
      float error = endPointMultiplier * (data[i].value - predicted);

      totalError += std::abs(error);

      // Compute gradients for each coefficient
      gradient[0] += error * pow(t, 3);
      gradient[1] += error * pow(t, 2);
      gradient[2] += error * t;
      gradient[3] += error;
    }

    // Break early if total error is negligible
    if (totalError < 1e-5) break;

    // Update coefficients using gradient descent with momentum
    for (int j = 0; j < 4; ++j) {
      velocity[j] = momentum * velocity[j] + learningRate * gradient[j];
      coeffs[j] += velocity[j];

      // Apply quantization-aware updates
      coeffs[j] = quantizeCoefficient(coeffs[j], QUANTIZATION_SCALE) / QUANTIZATION_SCALE;
    }

    // Dynamically adjust learning rate based on error magnitude
    if (iter > 0 && totalError < 0.1f) {
      learningRate *= 0.9f;  // Decay learning rate
    }
  }
}
