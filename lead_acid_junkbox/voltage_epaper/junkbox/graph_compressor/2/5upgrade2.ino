class AdaptiveFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Initialize sums for least squares computation
    float sumX[5] = {0}, sumY = 0, sumXY[5] = {0};
    for (int i = 0; i < count; i++) {
      float t = data[i].timestamp - data[0].timestamp;
      float v = data[i].value;

      float t2 = t * t;
      float t3 = t2 * t;
      float t4 = t3 * t;

      sumX[0] += t4 * t;
      sumX[1] += t4;
      sumX[2] += t3;
      sumX[3] += t2;
      sumX[4] += t;

      sumY += v;
      sumXY[0] += t3 * v;
      sumXY[1] += t2 * v;
      sumXY[2] += t * v;
      sumXY[3] += v;
    }

    // Dynamically decide order based on residuals
    for (int order = 3; order >= 0; order--) {
      coeffs[0] = (order >= 3) ? sumXY[0] / sumX[0] : 0;
      coeffs[1] = (order >= 2) ? sumXY[1] / sumX[1] : 0;
      coeffs[2] = (order >= 1) ? sumXY[2] / sumX[2] : 0;
      coeffs[3] = sumXY[3] / sumX[3];

      float residual = computeResidualError(data, count, coeffs);
      if (residual <= settings.maxError) break;  // Use the lowest order that fits
    }
  }
};
