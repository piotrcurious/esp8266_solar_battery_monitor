void setup() {
  Serial.begin(115200);

  // Adjust thresholds for your application
  spatialErrorThreshold = 2.0;  // Increase allowed time gap
  magnitudeErrorThreshold = 0.05;  // Decrease allowed fit error

  // Simulate adding data points
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    addDataPoint(i * 0.5, sin(i * 0.1));
  }

  // Compress data
  compressTemporaryBuffer();

  // Print storage buffer
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tStart=%.2f, tEnd=%.2f\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tStart, poly.tEnd);
  }
}
