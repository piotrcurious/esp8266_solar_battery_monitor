// Configuration for compression tuning
float spatialErrorThreshold = 1.0;  // Maximum allowed time gap per segment
float magnitudeErrorThreshold = 0.1;  // Maximum allowed value error per segment

// Compute the error of a polynomial fit over a segment
float computeFitError(Polynomial& poly, DataPoint* segment, int count) {
  float totalError = 0.0;
  for (int i = 0; i < count; i++) {
    float t = segment[i].timestamp - poly.tStart;  // Normalize timestamp
    float predicted = poly.a3 * t * t * t +
                      poly.a2 * t * t +
                      poly.a1 * t +
                      poly.a0;
    float error = abs(predicted - segment[i].value);
    totalError += error;
  }
  return totalError / count;  // Average error
}

// Adaptive compressor with spatial and magnitude error thresholds
void compressTemporaryBuffer() {
  int start = 0;

  while (start < tempBufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int end = start + 1;  // Start with a minimal segment
    Polynomial bestFit;
    float bestError = FLT_MAX;

    while (end < tempBufferCount) {
      // Attempt to fit a polynomial to the current segment
      Polynomial currentFit;
      fitPolynomial(&temporaryBuffer[start], end - start + 1, currentFit);

      // Compute fit error
      float fitError = computeFitError(currentFit, &temporaryBuffer[start], end - start + 1);

      // Check spatial and magnitude constraints
      bool withinSpatialError = (temporaryBuffer[end].timestamp - temporaryBuffer[start].timestamp) <= spatialErrorThreshold;
      bool withinMagnitudeError = fitError <= magnitudeErrorThreshold;

      if (withinSpatialError && withinMagnitudeError) {
        // Accept this segment if it improves the fit
        if (fitError < bestError) {
          bestFit = currentFit;
          bestError = fitError;
        }
        end++;
      } else {
        // Segment is too large; stop extending it
        break;
      }
    }

    // Store the best fit for this segment
    storageBuffer[storageCount++] = bestFit;

    // Move to the next segment
    start = end;
  }

  // Clear the temporary buffer
  resetTemporaryBuffer();
}
