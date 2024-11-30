float evaluateCompressedData(Polynomial* storageBuffer, int storageCount, float timestamp) {
  float baseTimestamp = 0.0;  // Tracks the start of the current segment
  float baseValue = 0.0;      // Tracks the cumulative value across segments

  for (int i = 0; i < storageCount; i++) {
    Polynomial poly = storageBuffer[i];
    float tDelta = poly.tDelta / 1000.0;  // Convert milliseconds to seconds

    if (timestamp >= baseTimestamp && timestamp <= baseTimestamp + tDelta) {
      // Timestamp falls within this polynomial's range
      float relativeTime = timestamp - baseTimestamp;

      // Dequantize coefficients
      float a3 = dequantizeCoefficient(poly.a3);
      float a2 = dequantizeCoefficient(poly.a2);
      float a1 = dequantizeCoefficient(poly.a1);
      float a0 = dequantizeCoefficient(poly.a0);

      // Evaluate the cubic polynomial relative to the base value
      float value = (a3 * pow(relativeTime, 3)) +
                    (a2 * pow(relativeTime, 2)) +
                    (a1 * relativeTime) +
                    (a0);
      return baseValue + value;
    }

    // Update base timestamp and base value for the next segment
    baseTimestamp += tDelta;

    // Evaluate the cumulative baseValue for the end of this segment
    baseValue += (dequantizeCoefficient(poly.a3) * pow(tDelta, 3)) +
                 (dequantizeCoefficient(poly.a2) * pow(tDelta, 2)) +
                 (dequantizeCoefficient(poly.a1) * tDelta) +
                 (dequantizeCoefficient(poly.a0));
  }

  // If timestamp is out of range, return NAN
  return NAN;
}
