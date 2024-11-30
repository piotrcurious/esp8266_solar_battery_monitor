void compressTemporaryBuffer(float maxError, int maxSegmentSize) {
  if (tempBufferCount == 0) return;

  int start = 0;

  // Initialize the first polynomial (absolute coefficients)
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  fitPolynomial(&temporaryBuffer[start], tempBufferCount, firstPoly);
  storageBuffer[storageCount++] = firstPoly;

  while (start < tempBufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;  // Minimum segment size
    Polynomial currentPoly = {0, 0, 0, 0, 0};
    float coeffs[4] = {0};
    float maxResidual = 0;

    while (segmentSize <= maxSegmentSize && start + segmentSize <= tempBufferCount) {
      // Fit polynomial for the current segment
      fitPolynomial(&temporaryBuffer[start], segmentSize, coeffs);

      // Compute residual error
      maxResidual = 0;
      for (int i = 0; i < segmentSize; i++) {
        float t = temporaryBuffer[start + i].timestamp - temporaryBuffer[start].timestamp;
        float predictedValue = coeffs[0] * t * t * t +
                               coeffs[1] * t * t +
                               coeffs[2] * t +
                               coeffs[3];
        float error = fabs(predictedValue - temporaryBuffer[start + i].value);
        if (error > maxResidual) maxResidual = error;
      }

      // If error exceeds threshold, stop expanding segment
      if (maxResidual > maxError) break;

      segmentSize++;
    }

    // Fit final polynomial for the valid segment size
    segmentSize--;  // Adjust to last valid segment size
    fitPolynomial(&temporaryBuffer[start], segmentSize, coeffs);

    // Create and store the polynomial
    currentPoly.a3 = quantizeCoefficient(coeffs[0]) - storageBuffer[storageCount - 1].a3;
    currentPoly.a2 = quantizeCoefficient(coeffs[1]) - storageBuffer[storageCount - 1].a2;
    currentPoly.a1 = quantizeCoefficient(coeffs[2]) - storageBuffer[storageCount - 1].a1;
    currentPoly.a0 = quantizeCoefficient(coeffs[3]) - storageBuffer[storageCount - 1].a0;
    currentPoly.tDelta = (uint16_t)((temporaryBuffer[start + segmentSize - 1].timestamp - temporaryBuffer[start].timestamp) * 1000);

    storageBuffer[storageCount++] = currentPoly;

    start += segmentSize;  // Move to the next segment
  }

  tempBufferCount = 0;  // Reset temporary buffer
}
