void advancedRefinement(DataPoint* data, int count, float* coeffs, 
                        float t0, float timeSpan) {
  const int maxIter = 5;
  const float QUANTIZATION_SCALE = 100.0f;  // Assumed scale for quantization
  
  for (int iter = 0; iter < maxIter; ++iter) {
    float totalError = 0;
    std::vector<float> errors(count, 0);

    // Compute residuals with end-point emphasis
    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;
      float predicted = coeffs[0] * pow(t, 3) + 
                        coeffs[1] * pow(t, 2) + 
                        coeffs[2] * t + 
                        coeffs[3];
      
      // Amplify errors at end-points
      float endPointMultiplier = (i == 0 || i == count - 1) ? 2.0f : 1.0f;

      errors[i] = endPointMultiplier * (data[i].value - predicted);
      totalError += std::abs(errors[i]);
    }

    // Break if total error is negligible
    if (totalError < 1e-6) break;

    // Adjust coefficients adaptively
    float adjustment[4] = {0};
    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;

      // Decreasing learning rate with iterations
      float learningRate = 0.02f / (iter + 1);

      // More aggressive adjustment for end-points
      float endPointMultiplier = (i == 0 || i == count - 1) ? 2.0f : 1.0f;

      adjustment[0] += endPointMultiplier * learningRate * errors[i] * pow(t, 3);
      adjustment[1] += endPointMultiplier * learningRate * errors[i] * pow(t, 2);
      adjustment[2] += endPointMultiplier * learningRate * errors[i] * t;
      adjustment[3] += endPointMultiplier * learningRate * errors[i];
    }

    // Apply adjustments considering quantization effects
    for (int j = 0; j < 4; ++j) {
      float updatedValue = coeffs[j] + adjustment[j];
      coeffs[j] = quantizeCoefficient(updatedValue, QUANTIZATION_SCALE) / QUANTIZATION_SCALE;
    }
  }
}
