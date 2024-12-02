void compressKernelEnhanced(DataPoint* buffer, int bufferCount, CompressionSettings settings) {
  if (bufferCount == 0) return;

  int start = 0;
  KernelEnhancedFitting kernelFitter(0.5, QUANTIZATION_SCALE); // Example sigma and quantization scale

  // First polynomial (absolute coefficients)
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  float coeffs[6] = {0};
  kernelFitter.fit(&buffer[start], bufferCount, coeffs);
  firstPoly.a3 = quantizeCoefficient(coeffs[0]);
  firstPoly.a2 = quantizeCoefficient(coeffs[1]);
  firstPoly.a1 = quantizeCoefficient(coeffs[2]);
  firstPoly.a0 = quantizeCoefficient(coeffs[3]);
  firstPoly.tDelta = (uint16_t)((buffer[start + bufferCount - 1].timestamp - buffer[start].timestamp) * 1000);
  storageBuffer[storageCount++] = firstPoly;

  // Subsequent polynomials (differential coefficients)
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;
    float maxResidual = 0;

    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      kernelFitter.fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, storageBuffer[storageCount]);
    storageCount++;
    start += segmentSize;
  }
}
