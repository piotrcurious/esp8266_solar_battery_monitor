#include <Arduino.h>
#include <circular_buffer.h>

// Structure to hold timestamp-value pairs
struct DataPoint {
    float timestamp;
    float value;
};

// Error diffusion patterns
enum class DiffusionPattern {
    FLOYD_STEINBERG,    // Classic Floyd-Steinberg
    JARVIS_JUDICE_NINKE,// Jarvis, Judice & Ninke
    STUCKI,            // Stucki
    SIERRA,            // Sierra
    SIMPLE             // Simple forward diffusion
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
    
    // Error diffusion buffers
    float errorBuffer[MAX_SAMPLES];
    float tempErrorBuffer[MAX_SAMPLES];  // For multi-line patterns
    
    DiffusionPattern currentPattern = DiffusionPattern::FLOYD_STEINBERG;
    
    void applyFloydSteinberg(float error, int index) {
        if (index < MAX_SAMPLES - 1) {
            errorBuffer[index + 1] += error * 0.4375f;  // 7/16 right
            if (index < MAX_SAMPLES - 2) {
                errorBuffer[index + 2] += error * 0.0625f;  // 1/16 far right
            }
        }
    }
    
    void applyJarvisJudiceNinke(float error, int index) {
        static const float weights[] = {
            0.14583f, 0.10417f, 0.06250f,  // 7/48, 5/48, 3/48
            0.10417f, 0.14583f, 0.10417f,  // 5/48, 7/48, 5/48
            0.06250f, 0.10417f, 0.06250f   // 3/48, 5/48, 3/48
        };
        
        for (int i = 0; i < 3; i++) {
            if (index + i + 1 < MAX_SAMPLES) {
                tempErrorBuffer[index + i + 1] += error * weights[i];
            }
        }
    }
    
    void applyStucki(float error, int index) {
        static const float weights[] = {
            0.19047f, 0.09524f, 0.04762f,  // 8/42, 4/42, 2/42
            0.09524f, 0.19047f, 0.09524f,  // 4/42, 8/42, 4/42
            0.04762f, 0.09524f, 0.04762f   // 2/42, 4/42, 2/42
        };
        
        for (int i = 0; i < 3; i++) {
            if (index + i + 1 < MAX_SAMPLES) {
                tempErrorBuffer[index + i + 1] += error * weights[i];
            }
        }
    }
    
    void applySierra(float error, int index) {
        static const float weights[] = {
            0.15789f, 0.10526f, 0.05263f,  // 3/19, 2/19, 1/19
            0.10526f, 0.15789f, 0.10526f,  // 2/19, 3/19, 2/19
            0.05263f, 0.10526f, 0.05263f   // 1/19, 2/19, 1/19
        };
        
        for (int i = 0; i < 3; i++) {
            if (index + i + 1 < MAX_SAMPLES) {
                tempErrorBuffer[index + i + 1] += error * weights[i];
            }
        }
    }
    
    void applySimple(float error, int index) {
        if (index < MAX_SAMPLES - 1) {
            errorBuffer[index + 1] += error * 0.75f;  // Simple forward diffusion
        }
    }
    
    void diffuseError(float error, int index) {
        switch (currentPattern) {
            case DiffusionPattern::FLOYD_STEINBERG:
                applyFloydSteinberg(error, index);
                break;
            case DiffusionPattern::JARVIS_JUDICE_NINKE:
                applyJarvisJudiceNinke(error, index);
                break;
            case DiffusionPattern::STUCKI:
                applyStucki(error, index);
                break;
            case DiffusionPattern::SIERRA:
                applySierra(error, index);
                break;
            case DiffusionPattern::SIMPLE:
                applySimple(error, index);
                break;
        }
    }
    
    float quantizeDelta(float delta, int index) {
        // Scale delta to fit in 4 bits with error diffusion
        float scaledDelta = delta + errorBuffer[index];
        
        // Clamp to MAX_DELTA range
        float clampedDelta = constrain(scaledDelta, -MAX_DELTA/2, MAX_DELTA/2);
        
        // Quantize to integer
        int quantized = round(clampedDelta);
        
        // Calculate and propagate error
        float error = scaledDelta - quantized;
        diffuseError(error, index);
        
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
        memset(tempErrorBuffer, 0, sizeof(tempErrorBuffer));
        memset(compressedDeltas, 0, sizeof(compressedDeltas));
    }
    
    void setDiffusionPattern(DiffusionPattern pattern) {
        currentPattern = pattern;
    }
    
    void addSample(float timestamp, float value) {
        DataPoint newPoint = {timestamp, value};
        originalData.push(newPoint);
        
        if (originalData.size() >= 2) {
            compress();
        }
    }
    
    void compress() {
        // Reset error buffers
        memset(errorBuffer, 0, sizeof(errorBuffer));
        memset(tempErrorBuffer, 0, sizeof(tempErrorBuffer));
        
        // Start from newest data (reverse order compression)
        float referenceValue = originalData.last().value;
        
        for (int i = originalData.size() - 2; i >= 0; i--) {
            float delta = originalData[i].value - referenceValue;
            float quantizedDelta = quantizeDelta(delta, i);
            packDelta(quantizedDelta, originalData.size() - 2 - i);
            
            // For multi-line patterns, copy temp buffer to main buffer
            if (currentPattern != DiffusionPattern::FLOYD_STEINBERG && 
                currentPattern != DiffusionPattern::SIMPLE) {
                memcpy(errorBuffer, tempErrorBuffer, sizeof(errorBuffer));
                memset(tempErrorBuffer, 0, sizeof(tempErrorBuffer));
            }
            
            referenceValue = originalData[i].value;
        }
        
        compressedSize = (originalData.size() - 1) * BITS_PER_DELTA;
    }
    
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
    
    float getTimestamp(int index) {
        if (index >= originalData.size()) return 0;
        return originalData[index].timestamp;
    }
    
    int size() {
        return originalData.size();
    }
    
    float getCompressionRatio() {
        if (originalData.size() <= 1) return 1.0;
        return (float)(compressedSize/8.0) / 
               (float)(sizeof(float) * (originalData.size() - 1));
    }
    
    const char* getCurrentPatternName() {
        switch (currentPattern) {
            case DiffusionPattern::FLOYD_STEINBERG:
                return "Floyd-Steinberg";
            case DiffusionPattern::JARVIS_JUDICE_NINKE:
                return "Jarvis-Judice-Ninke";
            case DiffusionPattern::STUCKI:
                return "Stucki";
            case DiffusionPattern::SIERRA:
                return "Sierra";
            case DiffusionPattern::SIMPLE:
                return "Simple";
            default:
                return "Unknown";
        }
    }
};
