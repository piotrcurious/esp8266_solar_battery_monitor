// CompressedBufferSystem.h
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>
#include <vector>
#include <algorithm>

namespace CompressedBuffer {

// Forward declarations
class IDisplay;
class ICompressor;
class DataPoint;

// Configuration structure
struct Config {
    float timeScale = 0.1f;         // Base time scale (100ms)
    float valueScale = 0.01f;       // Base value scale (1%)
    float timeWindow = 60.0f;       // Display window in seconds
    size_t bufferSize = 1024;       // Buffer size in bytes
    size_t statsWindow = 32;        // Window for statistics
    float errorThreshold = 0.1f;    // Error threshold for recompression
};

// Data structures
struct DataPoint {
    float timestamp;
    float value;
    
    DataPoint(float t = 0, float v = 0) : timestamp(t), value(v) {}
};

struct CompressionStats {
    float timeScale;
    float valueScale;
    float averageError;
    float compressionRatio;
};

// Error diffusion compressor interface
class ICompressor {
public:
    virtual ~ICompressor() = default;
    virtual uint8_t compress(float delta, float baseScale) = 0;
    virtual float expand(uint8_t compressed, float baseScale) = 0;
    virtual float getScale() const = 0;
    virtual float getError() const = 0;
    virtual void resetError() = 0;
};

// Adaptive compressor implementation
class AdaptiveCompressor : public ICompressor {
private:
    struct Parameters {
        float errorDiffusionRate = 0.7f;
        float maxErrorAccumulation = 2.0f;
        float scaleAlpha = 0.1f;
    } params;

    CircularBuffer<float, 32> recentDeltas;
    float errorAccumulator = 0;
    float adaptiveScale = 1.0f;
    float lastCompressionError = 0;
    float scaleEMA = 1.0f;

    void updateAdaptiveScale(float delta);
    float computeQuantizationError(float original, float reconstructed) const;
    
public:
    AdaptiveCompressor() = default;
    uint8_t compress(float delta, float baseScale) override;
    float expand(uint8_t compressed, float baseScale) override;
    float getScale() const override { return adaptiveScale; }
    float getError() const override { return lastCompressionError; }
    void resetError() override { errorAccumulator = 0; }
};

// Display interface
class IDisplay {
public:
    virtual ~IDisplay() = default;
    virtual void init() = 0;
    virtual void clear() = 0;
    virtual void update() = 0;
    virtual void drawGraph(const std::vector<DataPoint>& points, 
                         float minValue, float maxValue,
                         float timeWindow) = 0;
};

// OLED Display implementation
class OLEDDisplay : public IDisplay {
private:
    Adafruit_SSD1306& display;
    const int width;
    const int height;
    
    void drawAxes();
    void drawValue(float value);
    
public:
    OLEDDisplay(Adafruit_SSD1306& d, int w, int h) 
        : display(d), width(w), height(h) {}
        
    void init() override;
    void clear() override;
    void update() override;
    void drawGraph(const std::vector<DataPoint>& points,
                  float minValue, float maxValue,
                  float timeWindow) override;
};

// Main compressed buffer class
class TimeSeriesBuffer {
private:
    struct BufferState {
        std::vector<uint8_t> data;
        int head = 0;
        int tail = 0;
        int count = 0;
        float minValue = INFINITY;
        float maxValue = -INFINITY;
        float currentTimestamp = 0;
        float currentValue = 0;
    } state;
    
    Config config;
    std::unique_ptr<ICompressor> timeCompressor;
    std::unique_ptr<ICompressor> valueCompressor;
    std::unique_ptr<IDisplay> display;
    
    struct ErrorTracking {
        float sumSquaredError = 0;
        int errorCount = 0;
    } errorTracking;
    
    // Private helper methods
    int getPrevIndex(int index) const;
    int getNextIndex(int index) const;
    void updateMinMax(float value);
    void trackError();
    void recompressBuffer();
    std::vector<DataPoint> decompressAll() const;
    
public:
    TimeSeriesBuffer(const Config& cfg,
                    std::unique_ptr<ICompressor> tComp,
                    std::unique_ptr<ICompressor> vComp,
                    std::unique_ptr<IDisplay> disp);
    
    // Core operations
    bool push(const DataPoint& point);
    bool pop(DataPoint& point);
    bool peek(DataPoint& point) const;
    
    // Buffer management
    void clear();
    size_t available() const { return state.count; }
    
    // Display and statistics
    void updateDisplay();
    CompressionStats getStats() const;
    
    // Accessors
    float getCurrentValue() const { return state.currentValue; }
    float getCurrentTimestamp() const { return state.currentTimestamp; }
    std::pair<float, float> getValueRange() const {
        return {state.minValue, state.maxValue};
    }
};

} // namespace CompressedBuffer

// Implementation file (CompressedBufferSystem.cpp)
namespace CompressedBuffer {

uint8_t AdaptiveCompressor::compress(float delta, float baseScale) {
    updateAdaptiveScale(delta);
    
    float adjustedDelta = delta + errorAccumulator * params.errorDiffusionRate;
    float scale = baseScale * adaptiveScale;
    float scaled = adjustedDelta / scale;
    
    float sign = scaled >= 0 ? 1.0f : -1.0f;
    float absScaled = abs(scaled);
    float compressed = sign * log2f(1 + absScaled);
    
    int quantized = constrain(round(compressed * 64), -127, 127);
    float reconstructed = sign * (pow(2, abs(quantized) / 64.0f) - 1) * scale;
    
    lastCompressionError = delta - reconstructed;
    errorAccumulator = constrain(
        lastCompressionError + errorAccumulator * (1 - params.errorDiffusionRate),
        -params.maxErrorAccumulation,
        params.maxErrorAccumulation
    );
    
    return (abs(quantized) & 0x7F) | (quantized < 0 ? 0x80 : 0);
}

float AdaptiveCompressor::expand(uint8_t compressed, float baseScale) {
    bool negative = (compressed & 0x80) != 0;
    float magnitude = (compressed & 0x7F) / 64.0f;
    float expanded = (pow(2, magnitude) - 1) * baseScale * adaptiveScale;
    return negative ? -expanded : expanded;
}

void OLEDDisplay::init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }
    display.setRotation(0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
}

void OLEDDisplay::drawGraph(const std::vector<DataPoint>& points,
                          float minValue, float maxValue,
                          float timeWindow) {
    clear();
    
    const int GRAPH_HEIGHT = height * 3/4;
    const int GRAPH_OFFSET_Y = height - 8;
    
    // Draw current value
    drawValue(points.front().value);
    
    // Calculate scaling
    float valueRange = maxValue - minValue;
    if (valueRange < 0.1f) valueRange = 0.1f;
    
    float lastX = -1, lastY = -1;
    float currentTime = points.front().timestamp;
    
    for (const auto& point : points) {
        float timeOffset = currentTime - point.timestamp;
        int x = width - (timeOffset / timeWindow) * width;
        
        if (x < 0 || x >= width) continue;
        
        float normalizedValue = (point.value - minValue) / valueRange;
        int y = GRAPH_OFFSET_Y - normalizedValue * GRAPH_HEIGHT;
        
        if (lastX >= 0) {
            display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
        }
        
        lastX = x;
        lastY = y;
    }
    
    drawAxes();
    update();
}

// Test data generator class
class TestDataGenerator {
private:
    float time = 0;
    const float frequency = 0.1f;
    const float amplitude = 5.0f;
    const float noise = 0.5f;
    
public:
    DataPoint getNext() {
        float value = amplitude * sin(2 * PI * frequency * time) +
                     random(-noise, noise) +
                     2 * sin(2 * PI * 0.05f * time);
        
        time += 0.1f;
        return DataPoint(time, value);
    }
};

} // namespace CompressedBuffer

// Main application (main.cpp)
#include "CompressedBufferSystem.h"

using namespace CompressedBuffer;

// Global display instance
Adafruit_SSD1306 oledDisplay(128, 64, &Wire, -1);

// Global buffer instance
std::unique_ptr<TimeSeriesBuffer> buffer;
std::unique_ptr<TestDataGenerator> testGen;

void setup() {
    Serial.begin(115200);
    
    // Configure the system
    Config config;
    config.timeWindow = 30.0f;  // 30-second window
    
    // Create system components
    auto timeComp = std::make_unique<AdaptiveCompressor>();
    auto valueComp = std::make_unique<AdaptiveCompressor>();
    auto display = std::make_unique<OLEDDisplay>(oledDisplay, 128, 64);
    
    // Initialize buffer
    buffer = std::make_unique<TimeSeriesBuffer>(
        config,
        std::move(timeComp),
        std::move(valueComp),
        std::move(display)
    );
    
    // Initialize test generator
    testGen = std::make_unique<TestDataGenerator>();
}

void loop() {
    static unsigned long lastUpdate = 0;
    const unsigned long UPDATE_INTERVAL = 100;
    
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = currentMillis;
        
        // Get and push new data point
        DataPoint point = testGen->getNext();
        buffer->push(point);
        
        // Update display
        buffer->updateDisplay();
        
        // Print debug info
        auto stats = buffer->getStats();
        Serial.printf("T=%.3f V=%.3f Err=%.3f Ratio=%.2f\n",
                     stats.timeScale,
                     stats.valueScale,
                     stats.averageError,
                     stats.compressionRatio);
    }
}
