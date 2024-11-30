void testDecompression() {
  // Example data (assumes `storageBuffer` has been populated by compression)
  Polynomial storageBuffer[MAX_STORAGE];
  int storageCount = 0;

  // Simulate compressed data (replace with real compression logic)
  // Add your compression code here

  // Decompress into a new buffer
  DataPoint decompressedData[MAX_TEMPORARY];
  int decompressedCount = 0;

  decompress(storageBuffer, storageCount, decompressedData, decompressedCount);

  // Print the decompressed data
  for (int i = 0; i < decompressedCount; i++) {
    Serial.printf("Timestamp: %.3f, Value: %.3f\n", decompressedData[i].timestamp, decompressedData[i].value);
  }
}
