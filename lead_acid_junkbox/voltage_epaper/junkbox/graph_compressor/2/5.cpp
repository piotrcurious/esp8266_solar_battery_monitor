class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 4) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Extended normalization strategy
    float t0 = data[0].timestamp;
    float tMax = data[count-1].timestamp;
    float timeSpan = tMax - t0;

    if (timeSpan == 0) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Compute end-point weights for bias reduction
    float startWeight = computeEndPointWeight(data, count, true);
    float endWeight = computeEndPointWeight(data, count, false);

    // Adaptive matrix construction
    float A[4][4] = {0};
    float B[4] = {0};

    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;
      float v = data[i].value;

      // Adaptive weighting strategy
      float positionWeight = 1.0f;
      if (i == 0) positionWeight *= startWeight;
      if (i == count-1) positionWeight *= endWeight;

      // Inverse variance weighting
      float varianceWeight = 1.0f / (1.0f + std::abs(v - computeMean(data, count)));
      float weight = positionWeight * varianceWeight;

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

    // Gauss-Jordan elimination with improved stability
    for (int i = 0; i < 4; ++i) {
      int maxRow = i;
      for (int k = i + 1; k < 4; ++k) {
        if (std::abs(A[k][i]) > std::abs(A[maxRow][i])) {
          maxRow = k;
        }
      }

      std::swap(A[i], A[maxRow]);
      std::swap(B[i], B[maxRow]);

      float diag = A[i][i];
      if (std::abs(diag) < 1e-10) {
        coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0.0f;
        return;
      }

      for (int j = i; j < 4; ++j) {
        A[i][j] /= diag;
      }
      B[i] /= diag;

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

    // Rescale and apply end-point correction
    coeffs[0] = B[0] / pow(timeSpan, 3);
    coeffs[1] = B[1] / pow(timeSpan, 2);
    coeffs[2] = B[2] / timeSpan;
    coeffs[3] = B[3];

    // Advanced refinement with end-point sensitivity
    advancedRefinement(data, count, coeffs, t0, timeSpan);
  }

private:
  // Compute end-point weight to reduce bias
  float computeEndPointWeight(DataPoint* data, int count, bool isStart) {
    if (count <= 2) return 1.0f;

    int compareIndex = isStart ? 1 : count - 2;
    float endpointValue = isStart ? data[0].value : data[count-1].value;
    float compareValue = data[compareIndex].value;
    
    // Compute local slope and curvature
    float slopeDiff = std::abs(endpointValue - compareValue);
    float meanValue = computeMean(data, count);
    
    // More weight for points close to mean or with less deviation
    float weight = 1.0f / (1.0f + slopeDiff / meanValue);
    
    // Ensure weight is between 0.5 and 1.5
    return std::max(0.5f, std::min(1.5f, weight));
  }

  // Compute mean value
  float computeMean(DataPoint* data, int count) {
    float sum = 0;
    for (int i = 0; i < count; ++i) {
      sum += data[i].value;
    }
    return sum / count;
  }

  // Advanced refinement with end-point sensitivity
  void advancedRefinement(DataPoint* data, int count, float* coeffs, 
                          float t0, float timeSpan) {
    const int maxIter = 5;
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
        
        // End-point error amplification
        float endPointMultiplier = 1.0f;
        if (i == 0 || i == count-1) {
          endPointMultiplier = 2.0f;  // More weight on end points
        }

        errors[i] = endPointMultiplier * (data[i].value - predicted);
        totalError += std::abs(errors[i]);
      }

      // Break if error is very small
      if (totalError < 1e-6) break;

      // Adaptive coefficient adjustment
      for (int i = 0; i < count; ++i) {
        float t = (data[i].timestamp - t0) / timeSpan;
        
        // Decreasing learning rate with iterations
        float learningRate = 0.02f / (iter + 1);
        
        // More aggressive adjustment for end points
        float endPointMultiplier = (i == 0 || i == count-1) ? 2.0f : 1.0f;

        coeffs[0] += endPointMultiplier * learningRate * errors[i] * pow(t, 3);
        coeffs[1] += endPointMultiplier * learningRate * errors[i] * pow(t, 2);
        coeffs[2] += endPointMultiplier * learningRate * errors[i] * t;
        coeffs[3] += endPointMultiplier * learningRate * errors[i];
      }
    }
  }
};
