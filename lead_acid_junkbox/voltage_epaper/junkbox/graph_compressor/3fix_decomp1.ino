void decompress(Polynomial* storageBuffer, int storageCount, DataPoint* outputBuffer, int& outputCount) {
  if (storageCount == 0) {
    outputCount = 0;
    return;
  }

  float prevTimestamp = 0.0;  // The timestamp of the last decompressed point
  outputCount = 0;

  for (int i = 0; i < storageCount; i++) {
    Polynomial poly = storageBuffer[i];

    // Dequantize coefficients
    float a3 = dequantizeCoefficient(poly.a3);
    float a2 = dequantizeCoefficient(poly.a2);
    float a1 = dequantizeCoefficient(poly.a1);
    float a0 = dequantizeCoefficient(poly.a0);

    // Compute the time range for this polynomial
    float tDelta = poly.tDelta / 1000.0;
    int numSteps = 50;  // Number of points to interpolate (adjust as needed)
    float timeStep = tDelta / numSteps;

    for (int j = 0; j <= numSteps; j++) {
      float t = j * timeStep;
      float timestamp = prevTimestamp + t;

      // Compute the value using the polynomial equation
      float value = a3 * t * t * t + a2 * t * t + a1 * t + a0;

      // Store the reconstructed data point
      if (outputCount < MAX_TEMPORARY) {  // Ensure the output buffer doesn't overflow
        outputBuffer[outputCount++] = {timestamp, value};
      }
    }

    prevTimestamp += tDelta;  // Update the last timestamp
  }
}
