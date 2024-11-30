void testDecompressionAccuracy() {
  // Generate synthetic test data
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    temporaryBuffer[i].timestamp = i * 0.1f;    // Example: 0.1s intervals
    temporaryBuffer[i].value = sin(i * 0.1f);   // Example: Sine wave
  }
  tempBufferCount = 100;  // Example: 100 data points

  // Compression
  CompressionSettings settings = {0.01f, 10};  // Max error: 0.01, max segment size: 10
  LeastSquaresFitting fittingStrategy;
  Compressor compressor(settings, &fittingStrategy);
  compressor.compress(temporaryBuffer, tempBufferCount);

  // Validate decompression
  bool allPassed = true;
  for (int i = 0; i < tempBufferCount; i++) {
    float originalTimestamp = temporaryBuffer[i].timestamp;
    float originalValue = temporaryBuffer[i].value;

    float decompressedValue = evaluateCompressedData(storageBuffer, storageCount, originalTimestamp);
    float error = fabs(originalValue - decompressedValue);

    if (error > settings.maxError) {
      Serial.printf("ERROR: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
                    originalTimestamp, originalValue, decompressedValue, error);
      allPassed = false;
    } else {
      Serial.printf("PASSED: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
                    originalTimestamp, originalValue, decompressedValue, error);
    }
  }

  if (allPassed) {
    Serial.println("All decompression tests passed!");
  } else {
    Serial.println("Decompression test failed!");
  }
}
