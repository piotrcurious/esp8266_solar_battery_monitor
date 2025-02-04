#include <Arduino.h>
#include "miniz.h"  // Ensure you have miniz.h in your project

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Sample data to compress
    const char* input = "Hello, ESP32! This is a test for Deflate compression.";
    size_t input_size = strlen(input) + 1; // Include null terminator

    // Allocate memory for compressed data
    void* compressed_data = tdefl_compress_mem_to_heap(input, input_size, nullptr);
    
    if (!compressed_data) {
        Serial.println("Compression failed!");
        return;
    }

    // Print compressed size
    size_t compressed_size = mz_compressBound(input_size);
    Serial.printf("Original Size: %d, Compressed Size: %d\n", input_size, compressed_size);

    // Decompress data
    size_t decompressed_size = 0;
    void* decompressed_data = tinfl_decompress_mem_to_heap(compressed_data, compressed_size, &decompressed_size, 0);
    
    if (!decompressed_data) {
        Serial.println("Decompression failed!");
        mz_free(compressed_data); // Free memory
        return;
    }

    // Print decompressed data
    Serial.printf("Decompressed Data: %s\n", (char*)decompressed_data);

    // Free allocated memory
    mz_free(compressed_data);
    mz_free(decompressed_data);
}

void loop() {
    // No loop logic required
}
