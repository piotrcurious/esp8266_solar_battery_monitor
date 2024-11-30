float evaluateCompressedData(Polynomial* storageBuffer, int storageCount, float timestamp) {
  float baseTimestamp = 0.0;
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

      // Evaluate the cubic polynomial relative to the base value
      return baseValue +
             (a3 * relativeTime * relativeTime * relativeTime) +
             (a2 * relativeTime * relativeTime) +
             (a1 * relativeTime) +
             (a0);
    }

    // Update the base timestamp and base value
    baseTimestamp += tDelta;

    // Compute the final value at the end of this segment
    float tEnd = tDelta;
    baseValue += (a3 * tEnd * tEnd * tEnd) +
                 (a2 * tEnd * tEnd) +
                 (a1 * tEnd) +
                 (a0);
  }

  // If the timestamp is outside the range, return NAN
  return NAN;  // Not a Number, indicating out of bounds
}
