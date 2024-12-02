void testKernelEnhancedPipeline() {
  // Generate test data
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    temporaryBuffer[i].timestamp = i * 0.1f;
    temporaryBuffer[i].value = sin(i * 0.1f) + 0.1f * cos(i * 0.2f);
  }
  tempBufferCount = MAX_TEMPORARY;

  // Compress data
  CompressionSettings settings = {0.05, 10};
  compressKernelEnhanced(temporaryBuffer, tempBufferCount, settings);

  // Decompress and verify
  for (int i = 0; i < tempBufferCount; i++) {
    float decompressedValue = evaluateCompressedData(temporaryBuffer[i].timestamp);
    Serial.print("Original: ");
    Serial.print(temporaryBuffer[i].value);
    Serial.print(", Decompressed: ");
    Serial.println(decompressedValue);
  }
}
