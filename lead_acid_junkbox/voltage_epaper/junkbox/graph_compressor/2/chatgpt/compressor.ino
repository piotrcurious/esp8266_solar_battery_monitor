class Compressor {
private:
  CompressionSettings settings;
  FittingStrategy* fittingStrategy;

  // Helper: Calculate residual error
  float computeResidualError(DataPoint* data, int count, float* coeffs) {
    float error = 0;
    for (int i = 0; i < count; i++) {
      float t = data[i].timestamp - data[0].timestamp;  // Normalize timestamp
      float fittedValue = coeffs[0] * pow(t, 3) + coeffs[1] * pow(t, 2) + coeffs[2] * t + coeffs[3];
      error = fmax(error, fabs(data[i].value - fittedValue));  // Max absolute error
    }
    return error;
  }

  // Helper: Fit and store differential polynomial
  void fitDifferentialPolynomial(const Polynomial& prevPoly, DataPoint* data, int count, Polynomial& poly) {
    float coeffs[4] = {0};
    fittingStrategy->fit(data, count, coeffs);

    // Quantize coefficients as differentials
    poly.a3 = quantizeCoefficient(coeffs[0]);
    poly.a2 = quantizeCoefficient(coeffs[1]);
    poly.a1 = quantizeCoefficient(coeffs[2]);
    poly.a0 = quantizeCoefficient(coeffs[3]);

    // Calculate tDelta
    float timeSpan = data[count - 1].timestamp - data[0].timestamp;
    poly.tDelta = fmin(timeSpan * 1000, 65535);  // Ensure it fits uint16_t
  }

  // Binary search for optimal segment size
  int findOptimalSegmentSize(DataPoint* data, int bufferCount, int start) {
    int left = 2;  // Minimum segment size
    int right = settings.maxSegmentSize;

    while (left < right) {
      int mid = (left + right + 1) / 2;

      if (start + mid > bufferCount) {
        right = mid - 1;
        continue;
      }

      float coeffs[4] = {0};
      fittingStrategy->fit(&data[start], mid, coeffs);

      // Calculate residual error
      float maxResidual = computeResidualError(&data[start], mid, coeffs);

      // Calculate timestamp delta
      float timeSpan = data[start + mid - 1].timestamp - data[start].timestamp;

      if (maxResidual <= settings.maxError && timeSpan * 1000 <= 65535) {
        left = mid;  // Valid segment, try larger
      } else {
        right = mid - 1;  // Invalid segment, try smaller
      }
    }

    return left;  // Largest valid segment size
  }

public:
  Compressor(CompressionSettings settings, FittingStrategy* strategy)
      : settings(settings), fittingStrategy(strategy) {}

  void compress(DataPoint* buffer, int bufferCount) {
    if (bufferCount == 0) return;

    int start = 0;

    // Fit the first polynomial (absolute coefficients)
    Polynomial firstPoly = {0, 0, 0, 0, 0};
    float coeffs[4] = {0};
    fittingStrategy->fit(&buffer[start], bufferCount, coeffs);
    firstPoly.a3 = quantizeCoefficient(coeffs[0]);
    firstPoly.a2 = quantizeCoefficient(coeffs[1]);
    firstPoly.a1 = quantizeCoefficient(coeffs[2]);
    firstPoly.a0 = quantizeCoefficient(coeffs[3]);
    firstPoly.tDelta = fmin((buffer[bufferCount - 1].timestamp - buffer[start].timestamp) * 1000, 65535);
    storageBuffer[storageCount++] = firstPoly;

    // Fit subsequent polynomials
    while (start < bufferCount) {
      if (storageCount >= MAX_STORAGE) break;

      // Find optimal segment size
      int segmentSize = findOptimalSegmentSize(buffer, bufferCount, start);

      // Fit and store the polynomial
      fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, storageBuffer[storageCount]);
      storageCount++;

      // Move to the next segment
      start += segmentSize;
    }
  }
};
