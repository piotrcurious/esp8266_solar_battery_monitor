float evaluateCompressedData(Polynomial* storageBuffer, int storageCount, float timestamp) {
  float baseTimestamp = 0;  // Start with the absolute timestamp
  float baseValue = 0.0;

  for (int i = 0; i < storageCount; i++) {
    Polynomial poly = storageBuffer[i];
    float tDelta = poly.tDelta / 1000.0;

    if (timestamp >= baseTimestamp && timestamp <= baseTimestamp + tDelta) {
      // Timestamp falls within this polynomial's range
      float relativeTime = timestamp - baseTimestamp;

      // Dequantize coefficients
      float a3 = dequantizeCoefficient(poly.a3);
      float a2 = dequantizeCoefficient(poly.a2);
      float a1 = dequantizeCoefficient(poly.a1);
      float a0 = dequantizeCoefficient(poly.a0);

      // Evaluate the polynomial
      return baseValue +
             (a3 * relativeTime * relativeTime * relativeTime) +
             (a2 * relativeTime * relativeTime) +
             (a1 * relativeTime) +
             (a0);
    }

    // Update base timestamp and base value for the next segment
    baseTimestamp += tDelta;
    baseValue += (dequantizeCoefficient(poly.a3) * tDelta * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a2) * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a1) * tDelta) +
                 (dequantizeCoefficient(poly.a0));
  }

  return NAN;  // Timestamp out of range
}
