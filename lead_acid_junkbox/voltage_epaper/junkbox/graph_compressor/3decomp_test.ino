void testDecompressionAccuracy() {
  // Example input data buffer (replace with actual input data)
  DataPoint temporaryBuffer[MAX_TEMPORARY];
  int tempBufferCount = 100; // Example count

  // Fill the temporary buffer with test data
  for (int i = 0; i < tempBufferCount; i++) {
    temporaryBuffer[i].timestamp = i * 0.1;  // Example timestamps (0.1s intervals)
    temporaryBuffer[i].value = sin(i * 0.1); // Example values (sine wave)
  }

  // Compress the data
  CompressionSettings settings = {0.01, 10}; // Example settings
  LeastSquaresFitting fittingStrategy;
  Compressor compressor(settings, &fittingStrategy);
  compressor.compress(temporaryBuffer, tempBufferCount);

  // Test decompression
  bool allPassed = true;
  for (int i = 0; i < tempBufferCount; i++) {
    float originalTimestamp = temporaryBuffer[i].timestamp;
    float originalValue = temporaryBuffer[i].value;

    // Evaluate the compressed data at the same timestamp
    float decompressedValue = evaluateCompressedData(storageBuffer, storageCount, originalTimestamp);

    // Compare with original value
    float error = fabs(originalValue - decompressedValue);
    if (error > settings.maxError) {
      Serial.printf("Error at timestamp %.3f: Original=%.3f, Decompressed=%.3f, Error=%.3f\n",
                    originalTimestamp, originalValue, decompressedValue, error);
      allPassed = false;
    }
  }

  if (allPassed) {
    Serial.println("Decompression test passed: All values within error bounds.");
  } else {
    Serial.println("Decompression test failed: Some values exceeded error bounds.");
  }
}
