void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Start with the absolute timestamp
  float previousEndValue = buffer[0].value;          // Start with the absolute value

  // Compression loop
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;  // Start with a minimal segment size
    float maxResidual = 0;

    Polynomial nextPoly = {0, 0, 0, 0, 0};
    float coeffs[4] = {0};

    // Try increasing segment sizes to find the best fit
    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;  // Step back to the last valid size

    // Fit the polynomial for this segment
    fittingStrategy->fit(&buffer[start], segmentSize, coeffs);

    // Quantize the coefficients
    nextPoly.a3 = quantizeCoefficient(coeffs[0]);
    nextPoly.a2 = quantizeCoefficient(coeffs[1]);
    nextPoly.a1 = quantizeCoefficient(coeffs[2]);
    nextPoly.a0 = quantizeCoefficient(coeffs[3]);

    // Compute time delta and correct residuals
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Correct for residual error
    float reconstructedEndValue = previousEndValue +
                                  (dequantizeCoefficient(nextPoly.a3) * pow(tEnd - previousEndTimestamp, 3)) +
                                  (dequantizeCoefficient(nextPoly.a2) * pow(tEnd - previousEndTimestamp, 2)) +
                                  (dequantizeCoefficient(nextPoly.a1) * (tEnd - previousEndTimestamp)) +
                                  dequantizeCoefficient(nextPoly.a0);

    float errorCorrection = buffer[start + segmentSize - 1].value - reconstructedEndValue;
    nextPoly.a0 = quantizeCoefficient(dequantizeCoefficient(nextPoly.a0) + errorCorrection);

    // Update the base values for the next segment
    previousEndTimestamp = tEnd;
    previousEndValue = reconstructedEndValue + errorCorrection;

    // Store the polynomial
    storageBuffer[storageCount++] = nextPoly;

    // Move to the next segment
    start += segmentSize;
  }
}
