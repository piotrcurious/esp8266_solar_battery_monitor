#include <Arduino.h>
#include <circular_buffer.h>
#include <esp_task_wdt.h>

// Structure to hold timestamp-value pairs
struct DataPoint {
    float timestamp;
    float value;
};

// Global statistics for error tracking
struct GlobalStats {
    double runningMean;
    double runningVariance;
    double errorAccumulator;
    uint32_t sampleCount;
    float minValue;
    float maxValue;
    double exponentialMovingAverage;
    double errorMovingAverage;
    static constexpr float EMA_ALPHA = 0.1f;
};

// Pattern evaluation results
struct PatternResult {
    float mse;
    float snr;
    float peakError;
    float compressionTime;
    uint32_t cycleCount;
};

// Parallel task parameters
struct CompressionTask {
    void* buffer;
    DiffusionPattern pattern;
    PatternResult result;
    SemaphoreHandle_t completionSemaphore;
};

enum class DiffusionPattern {
    FLOYD_STEINBERG,
    JARVIS_JUDICE_NINKE,
    STUCKI,
    SIERRA,
    SIMPLE,
    ADAPTIVE  // New hybrid pattern
};

class CompressedBuffer {
private:
    static const int MAX_SAMPLES = 100;
    static const int BITS_PER_DELTA = 4;
    static const int MAX_DELTA = (1 << BITS_PER_DELTA) - 1;
    static const int MAX_ITERATIONS = 5;
    static const int NUM_PATTERNS = 6;
    static const int TASK_STACK_SIZE = 4096;
    static const uint32_t CPU_FREQ = 240000000; // ESP32 frequency
    
    CircularBuffer<DataPoint, MAX_SAMPLES> originalData;
    uint8_t compressedDeltas[(MAX_SAMPLES * BITS_PER_DELTA + 7) / 8];
    int compressedSize = 0;
    
    // Per-pattern buffers
    struct PatternBuffers {
        float errorBuffer[MAX_SAMPLES];
        float tempErrorBuffer[MAX_SAMPLES];
        float compressedValues[MAX_SAMPLES];
        PatternResult metrics;
    };
    
    // Aligned buffer array for better cache performance
    alignas(32) PatternBuffers patternBuffers[NUM_PATTERNS];
    
    // Global statistics
    GlobalStats globalStats = {0};
    
    // Task handles and synchronization
    TaskHandle_t compressionTasks[NUM_PATTERNS];
    SemaphoreHandle_t taskSemaphores[NUM_PATTERNS];
    SemaphoreHandle_t statsMutex;
    
    DiffusionPattern currentPattern = DiffusionPattern::ADAPTIVE;
    bool useParallelProcessing = true;

    // Cycle counting for performance metrics
    static inline uint32_t getCycleCount() {
        uint32_t ccount;
        asm volatile("rsr %0,ccount":"=a" (ccount));
        return ccount;
    }
    
    void updateGlobalStats(float value, float error) {
        xSemaphoreTake(statsMutex, portMAX_DELAY);
        
        globalStats.sampleCount++;
        double delta = value - globalStats.runningMean;
        globalStats.runningMean += delta / globalStats.sampleCount;
        double delta2 = value - globalStats.runningMean;
        globalStats.runningVariance += delta * delta2;
        
        globalStats.errorAccumulator += error;
        globalStats.minValue = min(globalStats.minValue, value);
        globalStats.maxValue = max(globalStats.maxValue, value);
        
        // Update exponential moving averages
        globalStats.exponentialMovingAverage = 
            (1 - GlobalStats::EMA_ALPHA) * globalStats.exponentialMovingAverage +
            GlobalStats::EMA_ALPHA * value;
        globalStats.errorMovingAverage = 
            (1 - GlobalStats::EMA_ALPHA) * globalStats.errorMovingAverage +
            GlobalStats::EMA_ALPHA * error;
            
        xSemaphoreGive(statsMutex);
    }

    static void compressionTaskFunction(void* parameter) {
        CompressionTask* task = (CompressionTask*)parameter;
        CompressedBuffer* buffer = (CompressedBuffer*)task->buffer;
        
        uint32_t startCycles = getCycleCount();
        
        // Perform compression with assigned pattern
        buffer->compressWithPattern(task->pattern, 
                                  &buffer->patternBuffers[static_cast<int>(task->pattern)]);
        
        uint32_t endCycles = getCycleCount();
        
        // Calculate metrics
        PatternResult& result = task->result;
        result.cycleCount = endCycles - startCycles;
        result.compressionTime = float(result.cycleCount) / CPU_FREQ * 1000.0f; // ms
        
        // Signal completion
        xSemaphoreGive(task->completionSemaphore);
        vTaskDelete(NULL);
    }
    
    void compressWithPattern(DiffusionPattern pattern, PatternBuffers* buffers) {
        memset(buffers->errorBuffer, 0, sizeof(buffers->errorBuffer));
        memset(buffers->tempErrorBuffer, 0, sizeof(buffers->tempErrorBuffer));
        
        float referenceValue = originalData.last().value;
        double totalError = 0.0;
        double totalSignal = 0.0;
        float maxError = 0.0f;
        
        // Apply pattern-specific compression
        for (int i = originalData.size() - 2; i >= 0; i--) {
            float delta = originalData[i].value - referenceValue;
            float scaledDelta = delta;
            
            // Apply global statistics for better error distribution
            if (pattern == DiffusionPattern::ADAPTIVE) {
                scaledDelta += (globalStats.errorMovingAverage * 0.5f) +
                              (buffers->errorBuffer[i] * 0.5f);
            } else {
                scaledDelta += buffers->errorBuffer[i];
            }
            
            // Normalize based on global range
            float range = globalStats.maxValue - globalStats.minValue;
            if (range > 0) {
                scaledDelta *= (MAX_DELTA / range);
            }
            
            float clampedDelta = constrain(scaledDelta, -MAX_DELTA/2, MAX_DELTA/2);
            int quantized = round(clampedDelta);
            
            float error = scaledDelta - quantized;
            applyErrorDiffusion(error, i, pattern, buffers);
            
            // Track error metrics
            float absoluteError = abs(error);
            maxError = max(maxError, absoluteError);
            totalError += error * error;
            totalSignal += originalData[i].value * originalData[i].value;
            
            buffers->compressedValues[i] = referenceValue - (quantized * range / MAX_DELTA);
            referenceValue = originalData[i].value;
            
            // Update global statistics
            updateGlobalStats(originalData[i].value, error);
        }
        
        // Calculate final metrics
        buffers->metrics.mse = totalError / originalData.size();
        buffers->metrics.snr = 10 * log10(totalSignal / totalError);
        buffers->metrics.peakError = maxError;
    }
    
    void initializeParallelProcessing() {
        statsMutex = xSemaphoreCreateMutex();
        
        for (int i = 0; i < NUM_PATTERNS; i++) {
            taskSemaphores[i] = xSemaphoreCreateBinary();
            
            CompressionTask* task = new CompressionTask{
                this,
                static_cast<DiffusionPattern>(i),
                PatternResult(),
                taskSemaphores[i]
            };
            
            xTaskCreatePinnedToCore(
                compressionTaskFunction,
                "compress",
                TASK_STACK_SIZE,
                task,
                1,
                &compressionTasks[i],
                i % 2  // Alternate between cores
            );
        }
    }
    
    void applyErrorDiffusion(float error, int index, DiffusionPattern pattern, 
                            PatternBuffers* buffers) {
        switch (pattern) {
            case DiffusionPattern::FLOYD_STEINBERG:
                if (index < MAX_SAMPLES - 1) {
                    buffers->errorBuffer[index + 1] += error * 0.4375f;
                    if (index < MAX_SAMPLES - 2) {
                        buffers->errorBuffer[index + 2] += error * 0.0625f;
                    }
                }
                break;
                
            case DiffusionPattern::ADAPTIVE: {
                // Hybrid approach using global statistics
                float adaptiveWeight = 0.5f + 0.25f * 
                    (1.0f - abs(globalStats.errorMovingAverage) / MAX_DELTA);
                
                if (index < MAX_SAMPLES - 1) {
                    buffers->errorBuffer[index + 1] += error * adaptiveWeight;
                    if (index < MAX_SAMPLES - 2) {
                        buffers->errorBuffer[index + 2] += error * (1.0f - adaptiveWeight);
                    }
                }
                break;
            }
            
            // [Other patterns remain unchanged...]
        }
    }
    
    DiffusionPattern selectBestPattern() {
        DiffusionPattern bestPattern = DiffusionPattern::ADAPTIVE;
        float bestMetric = -INFINITY;
        
        for (int i = 0; i < NUM_PATTERNS; i++) {
            PatternResult& result = patternBuffers[i].metrics;
            // Composite metric combining SNR, speed, and error
            float metric = result.snr * 0.5f + 
                          (1000.0f / result.compressionTime) * 0.3f +
                          (1.0f / result.peakError) * 0.2f;
                          
            if (metric > bestMetric) {
                bestMetric = metric;
                bestPattern = static_cast<DiffusionPattern>(i);
            }
        }
        
        return bestPattern;
    }

public:
    CompressedBuffer() : globalStats{0.0, 0.0, 0.0, 0, FLT_MAX, -FLT_MAX, 0.0, 0.0} {
        initializeParallelProcessing();
    }
    
    ~CompressedBuffer() {
        for (int i = 0; i < NUM_PATTERNS; i++) {
            vTaskDelete(compressionTasks[i]);
            vSemaphoreDelete(taskSemaphores[i]);
        }
        vSemaphoreDelete(statsMutex);
    }
    
    void addSample(float timestamp, float value) {
        DataPoint newPoint = {timestamp, value};
        originalData.push(newPoint);
        
        if (originalData.size() >= 2) {
            // Start parallel compression tasks
            for (int i = 0; i < NUM_PATTERNS; i++) {
                CompressionTask* task = new CompressionTask{
                    this,
                    static_cast<DiffusionPattern>(i),
                    PatternResult(),
                    taskSemaphores[i]
                };
                xTaskNotifyGive(compressionTasks[i]);
            }
            
            // Wait for all tasks to complete
            for (int i = 0; i < NUM_PATTERNS; i++) {
                xSemaphoreTake(taskSemaphores[i], portMAX_DELAY);
            }
            
            // Select best pattern and update current compression
            currentPattern = selectBestPattern();
            memcpy(compressedDeltas, 
                   patternBuffers[static_cast<int>(currentPattern)].compressedValues,
                   sizeof(compressedDeltas));
        }
    }
    
    // Get compression statistics
    GlobalStats getGlobalStats() {
        GlobalStats stats;
        xSemaphoreTake(statsMutex, portMAX_DELAY);
        memcpy(&stats, &globalStats, sizeof(GlobalStats));
        xSemaphoreGive(statsMutex);
        return stats;
    }
    
    // Enable/disable parallel processing
    void setParallelProcessing(bool enable) {
        useParallelProcessing = enable;
    }
    
    // Get pattern-specific metrics
    PatternResult getPatternMetrics(DiffusionPattern pattern) {
        return patternBuffers[static_cast<int>(pattern)].metrics;
    }
};
