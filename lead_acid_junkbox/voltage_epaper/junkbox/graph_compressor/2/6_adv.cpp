class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 4) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Extended normalization with robust scaling
    float t0 = data[0].timestamp;
    float tMax = data[count-1].timestamp;
    float timeSpan = tMax - t0;

    if (timeSpan == 0) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Compute data statistics for adaptive scaling
    DataStats stats = computeDataStats(data, count);

    // Initial coefficient estimation using robust method
    float initialCoeffs[4] = {0};
    robustInitialEstimation(data, count, initialCoeffs, t0, timeSpan);

    // Advanced adaptive learning with momentum and adaptive learning rate
    AdaptiveLearningParams params = {
      0.001f,  // Initial learning rate
      0.9f,    // Momentum factor
      1e-4f,   // L2 regularization strength
      0.0f,    // Initial momentum term
      {0, 0, 0, 0}  // Previous gradients
    };

    // Copy initial coefficients
    for (int i = 0; i < 4; ++i) {
      coeffs[i] = initialCoeffs[i];
    }

    // Advanced adaptive refinement
    adaptiveRefinement(data, count, coeffs, params, t0, timeSpan, stats);
  }

private:
  // Robust data statistics structure
  struct DataStats {
    float mean = 0;
    float stdDev = 0;
    float min = 0;
    float max = 0;
    float range = 0;
  };

  // Adaptive learning parameters
  struct AdaptiveLearningParams {
    float learningRate;
    float momentumFactor;
    float regularizationStrength;
    float momentumTerm;
    float prevGradients[4];
  };

  // Compute robust data statistics
  DataStats computeDataStats(DataPoint* data, int count) {
    DataStats stats;
    
    // Compute mean
    float sum = 0, sumSq = 0;
    stats.min = data[0].value;
    stats.max = data[0].value;

    for (int i = 0; i < count; ++i) {
      float val = data[i].value;
      sum += val;
      sumSq += val * val;
      stats.min = std::min(stats.min, val);
      stats.max = std::max(stats.max, val);
    }

    stats.mean = sum / count;
    stats.range = stats.max - stats.min;
    
    // Compute standard deviation using two-pass algorithm for numerical stability
    float variance = 0;
    for (int i = 0; i < count; ++i) {
      float diff = data[i].value - stats.mean;
      variance += diff * diff;
    }
    stats.stdDev = std::sqrt(variance / (count - 1));

    return stats;
  }

  // Robust initial coefficient estimation
  void robustInitialEstimation(DataPoint* data, int count, float* coeffs, 
                                float t0, float timeSpan) {
    // Median of differences approach
    std::vector<float> diffs(count - 1);
    for (int i = 1; i < count; ++i) {
      diffs[i-1] = (data[i].value - data[i-1].value) / 
                   (data[i].timestamp - data[i-1].timestamp);
    }
    
    // Sort and find median
    std::sort(diffs.begin(), diffs.end());
    float medianSlope = diffs[diffs.size() / 2];

    // Initial coefficient estimation with robust methods
    coeffs[2] = medianSlope * timeSpan;  // Linear term
    coeffs[3] = data[0].value;  // Constant term
    coeffs[1] = 0;  // Quadratic term
    coeffs[0] = 0;  // Cubic term
  }

  // Advanced adaptive refinement with momentum and adaptive learning
  void adaptiveRefinement(DataPoint* data, int count, float* coeffs, 
                          AdaptiveLearningParams& params, 
                          float t0, float timeSpan, 
                          const DataStats& stats) {
    const int maxIter = 10;
    float bestError = std::numeric_limits<float>::max();
    float bestCoeffs[4];

    for (int iter = 0; iter < maxIter; ++iter) {
      // Compute gradients with advanced techniques
      float gradients[4] = {0};
      float totalError = computeGradients(data, count, coeffs, gradients, 
                                          t0, timeSpan, stats);

      // Add L2 regularization
      for (int i = 0; i < 4; ++i) {
        gradients[i] += params.regularizationStrength * coeffs[i];
      }

      // Adaptive momentum (Adam-like update)
      for (int i = 0; i < 4; ++i) {
        // Momentum update
        params.momentumTerm = params.momentumFactor * params.prevGradients[i] + 
                               (1.0f - params.momentumFactor) * gradients[i];
        
        // Adaptive learning rate (exponential moving average of gradient)
        float adaptiveLR = params.learningRate / 
          (1.0f + 0.1f * std::sqrt(std::abs(params.momentumTerm)));

        // Update coefficients
        coeffs[i] -= adaptiveLR * params.momentumTerm;

        // Store for next iteration
        params.prevGradients[i] = params.momentumTerm;
      }

      // Early stopping and best model tracking
      if (totalError < bestError) {
        bestError = totalError;
        std::copy(coeffs, coeffs + 4, bestCoeffs);
      }

      // Adaptive learning rate decay
      params.learningRate *= 0.95f;

      // Convergence check
      if (totalError < 1e-6) break;
    }

    // Copy best coefficients
    std::copy(bestCoeffs, bestCoeffs + 4, coeffs);
  }

  // Advanced gradient computation
  float computeGradients(DataPoint* data, int count, float* coeffs, 
                         float* gradients, float t0, float timeSpan, 
                         const DataStats& stats) {
    float totalError = 0;
    
    for (int i = 0; i < count; ++i) {
      // Normalized time
      float t = (data[i].timestamp - t0) / timeSpan;
      
      // Predicted value
      float predicted = coeffs[0] * pow(t, 3) + 
                        coeffs[1] * pow(t, 2) + 
                        coeffs[2] * t + 
                        coeffs[3];
      
      // Compute error with scaling
      float error = data[i].value - predicted;
      
      // Weighted error based on point position and data statistics
      float positionWeight = 1.0f;
      if (i == 0 || i == count-1) {
        positionWeight = 2.0f;  // More weight on endpoints
      }

      // Compute gradients with scaled error
      float scaledError = positionWeight * error / stats.stdDev;
      
      gradients[0] += 2.0f * scaledError * pow(t, 3);
      gradients[1] += 2.0f * scaledError * pow(t, 2);
      gradients[2] += 2.0f * scaledError * t;
      gradients[3] += 2.0f * scaledError;

      // Accumulate total error
      totalError += scaledError * scaledError;
    }

    return totalError;
  }
};
