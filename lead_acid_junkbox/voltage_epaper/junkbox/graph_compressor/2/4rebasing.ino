void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Start with the absolute timestamp
  float previousEndValue = buffer[0].value;          // Start with the absolute value

  // First polynomial
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  float coeffs[4] = {0};
  fittingStrategy->fit(&buffer[start], bufferCount, coeffs);

  firstPoly.a3 = quantizeCoefficient(coeffs[0]);
  firstPoly.a2 = quantizeCoefficient(coeffs[1]);
  firstPoly.a1 = quantizeCoefficient(coeffs[2]);
  firstPoly.a0 = quantizeCoefficient(coeffs[3]);

  firstPoly.tDelta = 0;  // First polynomial uses absolute timestamp
  storageBuffer[storageCount++] = firstPoly;

  // Fit subsequent polynomials
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;
    float maxResidual = 0;

    // Fit and evaluate segments to find the optimal fit
    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;

    // Create a polynomial for the current segment
    Polynomial nextPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, nextPoly);

    // Compute the relative time delta
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Re-evaluate the reconstructed value at the segment's end
    float reconstructedEndValue = previousEndValue +
                                  (dequantizeCoefficient(nextPoly.a3) * pow(tEnd - previousEndTimestamp, 3)) +
                                  (dequantizeCoefficient(nextPoly.a2) * pow(tEnd - previousEndTimestamp, 2)) +
                                  (dequantizeCoefficient(nextPoly.a1) * (tEnd - previousEndTimestamp)) +
                                  dequantizeCoefficient(nextPoly.a0);

    // Adjust coefficients to minimize residual error
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
