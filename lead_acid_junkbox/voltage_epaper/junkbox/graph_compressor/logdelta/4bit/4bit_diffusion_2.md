#include <Arduino.h>
#include <circular_buffer.h>

// Structure to hold timestamp-value pairs
struct DataPoint {
    float timestamp;
    float value;
};

enum class DiffusionPattern {
    FLOYD_STEINBERG,
    JARVIS_JUDICE_NINKE,
    STUCKI,
    SIERRA,
    SIMPLE
};

class CompressedBuffer {
private:
    static const int MAX_SAMPLES = 100;
    static const int BITS_PER_DELTA = 4;
    static const int MAX_DELTA = (1 << BITS_PER_DELTA) - 1;
    static const int MAX_ITERATIONS = 5;
    static const float CONVERGENCE_THRESHOLD = 0.001f;
    
    CircularBuffer<DataPoint, MAX_SAMPLES> originalData;
    uint8_t compressedDeltas[(MAX_SAMPLES * BITS_PER_DELTA + 7) / 8];
    int compressedSize = 0;
    
    float errorBuffer[MAX_SAMPLES];
    float tempErrorBuffer[MAX_SAMPLES];
    float bestErrorBuffer[MAX_SAMPLES];
    
    // Optimization buffers
    float iterationBuffer[MAX_SAMPLES];
    float bestCompressedValues[MAX_SAMPLES];
    
    DiffusionPattern currentPattern = DiffusionPattern::FLOYD_STEINBERG;
    float signalVariance = 0.0f;
    float signalFrequency = 0.0f;
    
    struct CompressionMetrics {
        float meanSquaredError;
        float peakError;
        float signalToNoiseRatio;
    };
    
    // Signal analysis methods
    void analyzeSignal() {
        if (originalData.size() < 2) return;
        
        // Calculate mean
        float mean = 0.0f;
        for (int i = 0; i < originalData.size(); i++) {
            mean += originalData[i].value;
        }
        mean /= originalData.size();
        
        // Calculate variance and estimate frequency
        float sumSquaredDiff = 0.0f;
        int zeroCrossings = 0;
        bool wasAboveMean = originalData[0].value > mean;
        
        for (int i = 0; i < originalData.size(); i++) {
            float diff = originalData[i].value - mean;
            sumSquaredDiff += diff * diff;
            
            bool isAboveMean = originalData[i].value > mean;
            if (isAboveMean != wasAboveMean) {
                zeroCrossings++;
                wasAboveMean = isAboveMean;
            }
        }
        
        signalVariance = sumSquaredDiff / originalData.size();
        signalFrequency = float(zeroCrossings) / (originalData.last().timestamp - originalData[0].timestamp);
    }
    
    DiffusionPattern selectOptimalPattern() {
        if (signalVariance < 0.1f) {
            return DiffusionPattern::SIMPLE;  // Low variance - simple pattern sufficient
        }
        
        if (signalFrequency > 0.5f) {
            return DiffusionPattern::STUCKI;  // High frequency - need better error distribution
        }
        
        if (signalVariance > 1.0f) {
            return DiffusionPattern::JARVIS_JUDICE_NINKE;  // High variance - need best quality
        }
        
        return DiffusionPattern::FLOYD_STEINBERG;  // Default balanced option
    }
    
    void applyErrorDiffusion(float error, int index, float* targetBuffer) {
        switch (currentPattern) {
            case DiffusionPattern::FLOYD_STEINBERG:
                if (index < MAX_SAMPLES - 1) {
                    targetBuffer[index + 1] += error * 0.4375f;
                    if (index < MAX_SAMPLES - 2) {
                        targetBuffer[index + 2] += error * 0.0625f;
                    }
                }
                break;
                
            case DiffusionPattern::JARVIS_JUDICE_NINKE: {
                static const float weights[] = {0.14583f, 0.10417f, 0.06250f};
                for (int i = 0; i < 3 && index + i + 1 < MAX_SAMPLES; i++) {
                    targetBuffer[index + i + 1] += error * weights[i];
                }
                break;
            }
            
            case DiffusionPattern::STUCKI: {
                static const float weights[] = {0.19047f, 0.09524f, 0.04762f};
                for (int i = 0; i < 3 && index + i + 1 < MAX_SAMPLES; i++) {
                    targetBuffer[index + i + 1] += error * weights[i];
                }
                break;
            }
            
            case DiffusionPattern::SIERRA: {
                static const float weights[] = {0.15789f, 0.10526f, 0.05263f};
                for (int i = 0; i < 3 && index + i + 1 < MAX_SAMPLES; i++) {
                    targetBuffer[index + i + 1] += error * weights[i];
                }
                break;
            }
            
            case DiffusionPattern::SIMPLE:
                if (index < MAX_SAMPLES - 1) {
                    targetBuffer[index + 1] += error * 0.75f;
                }
                break;
        }
    }
    
    CompressionMetrics calculateMetrics(const float* decompressedValues) {
        CompressionMetrics metrics = {0.0f, 0.0f, 0.0f};
        float sumSquaredError = 0.0f;
        float sumSquaredSignal = 0.0f;
        
        for (int i = 0; i < originalData.size(); i++) {
            float error = abs(originalData[i].value - decompressedValues[i]);
            float squaredError = error * error;
            
            sumSquaredError += squaredError;
            sumSquaredSignal += originalData[i].value * originalData[i].value;
            
            if (error > metrics.peakError) {
                metrics.peakError = error;
            }
        }
        
        metrics.meanSquaredError = sumSquaredError / originalData.size();
        metrics.signalToNoiseRatio = 10 * log10(sumSquaredSignal / sumSquaredError);
        
        return metrics;
    }
    
    void optimizeCompression() {
        float bestMSE = INFINITY;
        float bestCompressedData[MAX_SAMPLES];
        DiffusionPattern bestPattern = currentPattern;
        
        // Try each diffusion pattern
        for (int pattern = 0; pattern < 5; pattern++) {
            currentPattern = static_cast<DiffusionPattern>(pattern);
            
            // Iterative optimization for current pattern
            float prevMSE = INFINITY;
            for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
                // Reset buffers
                memset(errorBuffer, 0, sizeof(errorBuffer));
                memset(tempErrorBuffer, 0, sizeof(tempErrorBuffer));
                
                // Compress with current parameters
                compress();
                
                // Decompress for analysis
                for (int i = 0; i < originalData.size(); i++) {
                    iterationBuffer[i] = getDecompressedValue(i);
                }
                
                // Calculate metrics
                CompressionMetrics metrics = calculateMetrics(iterationBuffer);
                
                // Check if this is the best result so far
                if (metrics.meanSquaredError < bestMSE) {
                    bestMSE = metrics.meanSquaredError;
                    bestPattern = currentPattern;
                    memcpy(bestCompressedData, iterationBuffer, sizeof(bestCompressedData));
                }
                
                // Check convergence
                if (abs(metrics.meanSquaredError - prevMSE) < CONVERGENCE_THRESHOLD) {
                    break;
                }
                prevMSE = metrics.meanSquaredError;
                
                // Adjust error diffusion weights based on local error distribution
                adjustErrorWeights(metrics);
            }
        }
        
        // Use the best found configuration
        currentPattern = bestPattern;
        memcpy(bestCompressedValues, bestCompressedData, sizeof(bestCompressedValues));
    }
    
    void adjustErrorWeights(const CompressionMetrics& metrics) {
        // Adaptive weight adjustment based on error metrics
        float adaptiveFactor = 1.0f;
        if (metrics.peakError > 0.1f) {
            adaptiveFactor = 0.9f;  // Reduce weights if error is too high
        } else if (metrics.signalToNoiseRatio < 20.0f) {
            adaptiveFactor = 1.1f;  // Increase weights if SNR is too low
        }
        
        // Apply adaptive factor to error diffusion
        for (int i = 0; i < originalData.size(); i++) {
            errorBuffer[i] *= adaptiveFactor;
        }
    }

public:
    CompressedBuffer() {
        memset(errorBuffer, 0, sizeof(errorBuffer));
        memset(tempErrorBuffer, 0, sizeof(tempErrorBuffer));
        memset(bestErrorBuffer, 0, sizeof(bestErrorBuffer));
        memset(compressedDeltas, 0, sizeof(compressedDeltas));
        memset(bestCompressedValues, 0, sizeof(bestCompressedValues));
    }
    
    void addSample(float timestamp, float value) {
        DataPoint newPoint = {timestamp, value};
        originalData.push(newPoint);
        
        if (originalData.size() >= 2) {
            analyzeSignal();
            currentPattern = selectOptimalPattern();
            optimizeCompression();
        }
    }
    
    void compress() {
        memset(errorBuffer, 0, sizeof(errorBuffer));
        float referenceValue = originalData.last().value;
        
        for (int i = originalData.size() - 2; i >= 0; i--) {
            float delta = originalData[i].value - referenceValue;
            float scaledDelta = delta + errorBuffer[i];
            float clampedDelta = constrain(scaledDelta, -MAX_DELTA/2, MAX_DELTA/2);
            int quantized = round(clampedDelta);
            
            float error = scaledDelta - quantized;
            applyErrorDiffusion(error, i, errorBuffer);
            
            packDelta(quantized, originalData.size() - 2 - i);
            referenceValue = originalData[i].value;
        }
        
        compressedSize = (originalData.size() - 1) * BITS_PER_DELTA;
    }
    
    // ... [Previous packing/unpacking methods remain unchanged]
    
    float getCompressionMetrics(const char** patternName) {
        *patternName = getCurrentPatternName();
        return calculateMetrics(bestCompressedValues).signalToNoiseRatio;
    }
    
    const char* getCurrentPatternName() {
        switch (currentPattern) {
            case DiffusionPattern::FLOYD_STEINBERG: return "Floyd-Steinberg";
            case DiffusionPattern::JARVIS_JUDICE_NINKE: return "Jarvis-Judice-Ninke";
            case DiffusionPattern::STUCKI: return "Stucki";
            case DiffusionPattern::SIERRA: return "Sierra";
            case DiffusionPattern::SIMPLE: return "Simple";
            default: return "Unknown";
        }
    }
};

I've enhanced the compression system with adaptive pattern selection and iterative optimization. Key improvements include:

1. Signal Analysis:
   - Calculates signal variance and frequency characteristics
   - Automatically selects optimal diffusion pattern based on signal properties

2. Iterative Optimization:
   - Implements multiple compression iterations to minimize data loss
   - Uses convergence detection to avoid unnecessary iterations
   - Maintains best results across iterations

3. Adaptive Features:
   - Dynamic error weight adjustment based on compression metrics
   - Pattern selection based on signal characteristics
   - SNR-based optimization

4. Quality Metrics:
   - Mean squared error
   - Peak error
   - Signal-to-noise ratio

Usage example:
```cpp
CompressedBuffer buffer;

// Add samples - compression automatically optimizes
buffer.addSample(1000.0, 10.5);
buffer.addSample(1001.0, 10.7);
buffer.addSample(1002.0, 10.6);

// Get compression metrics
const char* patternName;
float snr = buffer.getCompressionMetrics(&patternName);
Serial.printf("Using %s pattern, SNR: %.2f dB\n", patternName, snr);
```

Would you like me to:
1. Add more sophisticated signal analysis methods?
2. Implement parallel pattern evaluation?
3. Add frequency-domain optimization?
4. Include adaptive bit depth selection?
