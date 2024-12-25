#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class AdaptiveCompressor {
private:
    // Sliding window statistics for adaptive scaling
    static const int STAT_WINDOW = 32;
    CircularBuffer<float, STAT_WINDOW> recentDeltas;
    
    float errorAccumulator = 0;
    float adaptiveScale = 1.0f;
    float lastCompressionError = 0;
    
    // Exponential moving average for scale adaptation
    float scaleEMA = 1.0f;
    const float scaleAlpha = 0.1f;
    
    // Error diffusion parameters
    const float errorDiffusionRate = 0.7f;
    const float maxErrorAccumulation = 2.0f;
    
    void updateAdaptiveScale(float delta) {
        recentDeltas.push(abs(delta));
        
        if (recentDeltas.isFull()) {
            // Calculate median absolute delta for robust scaling
            std::vector<float> sortedDeltas;
            sortedDeltas.reserve(STAT_WINDOW);
            
            for (int i = 0; i < STAT_WINDOW; i++) {
                sortedDeltas.push_back(recentDeltas[i]);
            }
            
            std::sort(sortedDeltas.begin(), sortedDeltas.end());
            float medianDelta = sortedDeltas[STAT_WINDOW / 2];
            
            // Update scale using EMA to smooth changes
            float targetScale = medianDelta / 32.0f;  // Aim to use full 7-bit range
            scaleEMA = scaleAlpha * targetScale + (1 - scaleAlpha) * scaleEMA;
            adaptiveScale = max(scaleEMA, 0.0001f);  // Prevent zero scale
        }
    }
    
public:
    uint8_t compress(float delta, float baseScale) {
        updateAdaptiveScale(delta);
        
        // Apply error diffusion from previous compression
        float adjustedDelta = delta + errorAccumulator * errorDiffusionRate;
        
        // Compress with adaptive scaling
        float scale = baseScale * adaptiveScale;
        float scaled = adjustedDelta / scale;
        
        // Symmetric logarithmic compression
        float sign = scaled >= 0 ? 1.0f : -1.0f;
        float absScaled = abs(scaled);
        float compressed = sign * log2f(1 + absScaled);
        
        // Quantize to 7 bits
        int quantized = constrain(round(compressed * 64), -127, 127);
        
        // Calculate new compression error
        float reconstructed = sign * (pow(2, abs(quantized) / 64.0f) - 1) * scale;
        lastCompressionError = delta - reconstructed;
        
        // Update error accumulator with limiting
        errorAccumulator = lastCompressionError + 
                          errorAccumulator * (1 - errorDiffusionRate);
        errorAccumulator = constrain(errorAccumulator, 
                                   -maxErrorAccumulation, 
                                   maxErrorAccumulation);
        
        // Pack into 8 bits with sign
        uint8_t result = abs(quantized) & 0x7F;
        if (quantized < 0) result |= 0x80;
        
        return result;
    }
    
    float expand(uint8_t compressed, float baseScale) {
        // Unpack from 8 bits
        bool negative = (compressed & 0x80) != 0;
        float magnitude = (compressed & 0x7F) / 64.0f;
        
        // Reverse logarithmic compression
        float expanded = (pow(2, magnitude) - 1) * baseScale * adaptiveScale;
        return negative ? -expanded : expanded;
    }
    
    float getScale() const { return adaptiveScale; }
    float getError() const { return lastCompressionError; }
    void resetError() { errorAccumulator = 0; }
};

class CompressedBuffer {
private:
    static const int BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE];
    int head = 0;
    int tail = 0;
    int count = 0;
    
    float currentTimestamp = 0;
    float currentValue = 0;
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    float timeWindow = 60.0f;
    
    // Separate compressors for time and value to handle different scales
    AdaptiveCompressor timeCompressor;
    AdaptiveCompressor valueCompressor;
    
    // Base compression scales
    const float TIME_SCALE = 0.1f;    // 100ms resolution
    const float VALUE_SCALE = 0.01f;  // 1% resolution
    
    // Statistical tracking for adaptive recompression
    float sumSquaredError = 0;
    int errorCount = 0;
    const float ERROR_THRESHOLD = 0.1f;  // Trigger recompression when average error too high
    
    int getPrevIndex(int index) { return (index - 2 + BUFFER_SIZE) % BUFFER_SIZE; }
    int getNextIndex(int index) { return (index + 2) % BUFFER_SIZE; }
    
    void updateMinMax(float value) {
        minValue = min(minValue, value);
        maxValue = max(maxValue, value);
    }
    
    void recompressBuffer() {
        if (count <= 1) return;
        
        std::vector<std::pair<float, float>> entries;
        entries.reserve(count);
        
        // Decompress all entries
        int readIdx = head;
        entries.push_back({currentTimestamp, currentValue});
        
        // Reset error accumulators before recompression
        timeCompressor.resetError();
        valueCompressor.resetError();
        
        for (int i = 1; i < count; i++) {
            readIdx = getPrevIndex(readIdx);
            float timeDelta = timeCompressor.expand(buffer[readIdx], TIME_SCALE);
            float valueDelta = valueCompressor.expand(buffer[readIdx + 1], VALUE_SCALE);
            
            float timestamp = entries.back().first + timeDelta;
            float value = entries.back().second + valueDelta;
            entries.push_back({timestamp, value});
        }
        
        // Recompress relative to current values with fresh error accumulation
        int writeIdx = head;
        for (int i = 1; i < count; i++) {
            writeIdx = getPrevIndex(writeIdx);
            float timeDelta = entries[i].first - currentTimestamp;
            float valueDelta = entries[i].second - currentValue;
            
            buffer[writeIdx] = timeCompressor.compress(timeDelta, TIME_SCALE);
            buffer[writeIdx + 1] = valueCompressor.compress(valueDelta, VALUE_SCALE);
        }
        
        // Reset error tracking after recompression
        sumSquaredError = 0;
        errorCount = 0;
    }
    
    void trackError() {
        float timeError = timeCompressor.getError();
        float valueError = valueCompressor.getError();
        
        sumSquaredError += timeError * timeError + valueError * valueError;
        errorCount++;
        
        // Check if recompression is needed
        if (errorCount >= 50) {  // Check every 50 samples
            float averageError = sqrt(sumSquaredError / (2 * errorCount));
            if (averageError > ERROR_THRESHOLD) {
                recompressBuffer();
            }
            sumSquaredError = 0;
            errorCount = 0;
        }
    }

public:
    CompressedBuffer(float timeWindowSeconds = 60.0f) : timeWindow(timeWindowSeconds) {}
    
    bool push(float timestamp, float value) {
        if (count >= BUFFER_SIZE/2) {
            tail = getNextIndex(tail);
            count--;
        }
        
        head = getPrevIndex(head);
        
        if (count == 0) {
            currentTimestamp = timestamp;
            currentValue = value;
            buffer[head] = 0;
            buffer[head + 1] = 0;
            updateMinMax(value);
            count++;
            return true;
        }
        
        float timeDelta = currentTimestamp - timestamp;
        float valueDelta = currentValue - value;
        
        // Compress with error diffusion
        buffer[head] = timeCompressor.compress(timeDelta, TIME_SCALE);
        buffer[head + 1] = valueCompressor.compress(valueDelta, VALUE_SCALE);
        
        currentTimestamp = timestamp;
        currentValue = value;
        updateMinMax(value);
        
        count++;
        trackError();
        
        // Remove old data outside time window
        while (count > 1) {
            float oldestTime;
            float oldestValue;
            if (!peek(oldestTime, oldestValue)) break;
            
            if (currentTimestamp - oldestTime > timeWindow) {
                tail = getNextIndex(tail);
                count--;
            } else {
                break;
            }
        }
        
        return true;
    }

    // ... (rest of the class methods remain the same, but use timeCompressor.expand() 
    // and valueCompressor.expand() for decompression operations)

    // Add debug information
    void getCompressionStats(float& timeScale, float& valueScale, float& avgError) {
        timeScale = timeCompressor.getScale();
        valueScale = valueCompressor.getScale();
        avgError = errorCount > 0 ? sqrt(sumSquaredError / (2 * errorCount)) : 0;
    }
};

// ... (rest of the code including TestDataGenerator, setup(), and loop() remains the same)
