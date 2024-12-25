#include <Arduino.h>
#include <circular_buffer.h>

// Structure to hold timestamp-value pairs
struct DataPoint {
    float timestamp;
    float value;
};

class CompressedBuffer {
private:
    static const int MAX_SAMPLES = 100;  // Adjust based on your needs
    static const int BITS_PER_DELTA = 4;
    static const int MAX_DELTA = (1 << BITS_PER_DELTA) - 1;
    
    // Circular buffer for original data
    CircularBuffer<DataPoint, MAX_SAMPLES> originalData;
    
    // Buffer for compressed deltas
    uint8_t compressedDeltas[(MAX_SAMPLES * BITS_PER_DELTA + 7) / 8];
    int compressedSize = 0;
    
    // Error diffusion buffer
    float errorBuffer[MAX_SAMPLES];
    
    float quantizeDelta(float delta, int index) {
        // Scale delta to fit in 4 bits with error diffusion
        float scaledDelta = delta + errorBuffer[index];
        
        // Clamp to MAX_DELTA range
        float clampedDelta = constrain(scaledDelta, -MAX_DELTA/2, MAX_DELTA/2);
        
        // Quantize to integer
        int quantized = round(clampedDelta);
        
        // Calculate and propagate error
        float error = scaledDelta - quantized;
        if (index < MAX_SAMPLES - 1) {
            errorBuffer[index + 1] += error * 0.75f;  // Error diffusion coefficient
        }
        
        return quantized;
    }
    
    void packDelta(int delta, int position) {
        int bytePos = position * BITS_PER_DELTA / 8;
        int bitPos = (position * BITS_PER_DELTA) % 8;
        
        // Convert signed delta to unsigned 4-bit value
        uint8_t unsignedDelta = (delta + MAX_DELTA/2) & 0x0F;
        
        // Pack into compressed buffer
        compressedDeltas[bytePos] &= ~(0x0F << bitPos);  // Clear bits
        compressedDeltas[bytePos] |= (unsignedDelta << bitPos);  // Set new bits
        
        if (bitPos > 4) {  // Handle overflow to next byte
            compressedDeltas[bytePos + 1] &= ~(0x0F >> (8 - bitPos));
            compressedDeltas[bytePos + 1] |= (unsignedDelta >> (8 - bitPos));
        }
    }
    
    int unpackDelta(int position) {
        int bytePos = position * BITS_PER_DELTA / 8;
        int bitPos = (position * BITS_PER_DELTA) % 8;
        
        uint8_t delta;
        if (bitPos <= 4) {
            delta = (compressedDeltas[bytePos] >> bitPos) & 0x0F;
        } else {
            delta = ((compressedDeltas[bytePos] >> bitPos) | 
                    (compressedDeltas[bytePos + 1] << (8 - bitPos))) & 0x0F;
        }
        
        // Convert back to signed value
        return delta - MAX_DELTA/2;
    }

public:
    CompressedBuffer() {
        memset(errorBuffer, 0, sizeof(errorBuffer));
        memset(compressedDeltas, 0, sizeof(compressedDeltas));
    }
    
    void addSample(float timestamp, float value) {
        DataPoint newPoint = {timestamp, value};
        originalData.push(newPoint);
        
        if (originalData.size() >= 2) {
            compress();
        }
    }
    
    void compress() {
        // Reset error buffer
        memset(errorBuffer, 0, sizeof(errorBuffer));
        
        // Start from newest data (reverse order compression)
        float referenceValue = originalData.last().value;
        
        for (int i = originalData.size() - 2; i >= 0; i--) {
            float delta = originalData[i].value - referenceValue;
            float quantizedDelta = quantizeDelta(delta, i);
            packDelta(quantizedDelta, originalData.size() - 2 - i);
            
            referenceValue = originalData[i].value;
        }
        
        compressedSize = (originalData.size() - 1) * BITS_PER_DELTA;
    }
    
    // Get decompressed value at index (0 is oldest)
    float getDecompressedValue(int index) {
        if (index >= originalData.size()) return 0;
        
        float value = originalData.last().value;  // Start with newest value
        
        // Reconstruct value by applying deltas in reverse
        for (int i = 0; i < originalData.size() - 1 - index; i++) {
            int delta = unpackDelta(i);
            value -= delta;
        }
        
        return value;
    }
    
    // Get original timestamp at index
    float getTimestamp(int index) {
        if (index >= originalData.size()) return 0;
        return originalData[index].timestamp;
    }
    
    int size() {
        return originalData.size();
    }
    
    // Get compression ratio
    float getCompressionRatio() {
        if (originalData.size() <= 1) return 1.0;
        return (float)(compressedSize/8.0) / 
               (float)(sizeof(float) * (originalData.size() - 1));
    }
};

I've created an implementation of a lossy rolling buffer with delta compression designed for the ESP32. Here are the key features:

1. Uses a circular buffer to store original timestamp/value pairs
2. Implements 4-bit delta compression with error diffusion
3. Stores deltas in reverse order (newest data is reference)
4. Includes efficient bit-packing of 4-bit deltas
5. Provides methods to access both compressed and decompressed data
6. Includes compression ratio calculation

To use this code:

```cpp
CompressedBuffer buffer;

// Add samples
buffer.addSample(1000.0, 10.5);
buffer.addSample(1001.0, 10.7);
buffer.addSample(1002.0, 10.6);

// Get decompressed values
float value = buffer.getDecompressedValue(0);  // Get oldest value
float timestamp = buffer.getTimestamp(0);      // Get timestamp
float ratio = buffer.getCompressionRatio();    // Get compression ratio
```

Would you like me to:
1. Add more error diffusion patterns?
2. Modify the compression parameters?
3. Add methods for batch processing?
4. Include additional data validation?
