void setup() {
  Serial.begin(115200);

  // Simulate adding data points
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    addDataPoint(i * 0.5, sin(i * 0.1) + random(-100, 100) / 1000.0);  // Add some noise for realism
  }

  // Compress with a maximum error of 0.01 and a segment size of up to 16 points
  compressTemporaryBuffer(0.01, 16);

  // Print compressed storage
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tDelta=%d\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tDelta);
  }

  // Decompress a value
  float timestamp = 1.5;
  float value = decompressValue(timestamp);
  if (!isnan(value)) {
    Serial.printf("Decompressed value at %.2f: %.2f\n", timestamp, value);
  }
}

void loop() {}
