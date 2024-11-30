void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Track the end of the previous segment

  // Fit the first polynomial (absolute coefficients)
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  float coeffs[4] = {0};
  fittingStrategy->fit(&buffer[start], bufferCount, coeffs);
  firstPoly.a3 = quantizeCoefficient(coeffs[0]);
  firstPoly.a2 = quantizeCoefficient(coeffs[1]);
  firstPoly.a1 = quantizeCoefficient(coeffs[2]);
  firstPoly.a0 = quantizeCoefficient(coeffs[3]);

  // Store the absolute timestamp of the first data point
  firstPoly.tDelta = 0;  // First polynomial uses absolute time as reference
  storageBuffer[storageCount++] = firstPoly;

  // Fit subsequent polynomials
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;
    float maxResidual = 0;

    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;
    Polynomial nextPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, nextPoly);

    // Compute the relative time delta
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    // Store time delta in milliseconds, relative to the previous end timestamp
    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Update previous end timestamp
    previousEndTimestamp = tEnd;

    // Store the polynomial
    storageBuffer[storageCount++] = nextPoly;

    // Move to the next segment
    start += segmentSize;
  }
}
