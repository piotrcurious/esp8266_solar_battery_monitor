// SystemTypes.h
#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <optional>
#include <functional>

namespace tsb {  // time series buffer

// Core data structures
struct Point {
    float timestamp;
    float value;
    
    Point(float t = 0, float v = 0) : timestamp(t), value(v) {}
    bool operator==(const Point& other) const {
        return timestamp == other.timestamp && value == other.value;
    }
};

struct Range {
    float min;
    float max;
    Range(float mn = 0, float mx = 0) : min(mn), max(mx) {}
};

struct Stats {
    float scale;
    float error;
    size_t samples;
};

// Configuration structures
struct CompressionConfig {
    float baseScale;
    float maxError;
    float adaptationRate;
    size_t windowSize;
};

struct BufferConfig {
    size_t capacity;
    float timeWindow;
    CompressionConfig timeCompression;
    CompressionConfig valueCompression;
};

struct DisplayConfig {
    uint16_t width;
    uint16_t height;
    uint8_t rotation;
    uint8_t address;
};

// Result types
template<typename T>
struct Result {
    std::optional<T> value;
    std::string error;
    
    bool isOk() const { return value.has_value(); }
    bool isError() const { return !value.has_value(); }
    
    static Result<T> ok(T val) {
        return Result<T>{std::optional<T>(std::move(val)), ""};
    }
    
    static Result<T> err(std::string e) {
        return Result<T>{std::nullopt, std::move(e)};
    }
};

using ErrorOr = Result<bool>;

} // namespace tsb

// Compression.h
namespace tsb {

class ICompressor {
public:
    virtual ~ICompressor() = default;
    virtual Result<uint8_t> compress(float delta) = 0;
    virtual Result<float> expand(uint8_t compressed) = 0;
    virtual Stats getStats() const = 0;
    virtual void reset() = 0;
};

class AdaptiveCompressor : public ICompressor {
private:
    class ErrorDiffusion {
        float accumulator = 0;
        float rate;
        float maxAccumulation;
        
    public:
        ErrorDiffusion(float r, float max) : rate(r), maxAccumulation(max) {}
        float process(float error);
        void reset() { accumulator = 0; }
    };
    
    class ScaleAdapter {
        float scale = 1.0f;
        float alpha;
        std::vector<float> window;
        size_t pos = 0;
        
    public:
        ScaleAdapter(float a, size_t size) : alpha(a), window(size) {}
        float adapt(float value);
        float getScale() const { return scale; }
    };
    
    CompressionConfig config;
    std::unique_ptr<ErrorDiffusion> errorDiffusion;
    std::unique_ptr<ScaleAdapter> scaleAdapter;
    Stats stats{1.0f, 0.0f, 0};
    
public:
    explicit AdaptiveCompressor(const CompressionConfig& cfg);
    Result<uint8_t> compress(float delta) override;
    Result<float> expand(uint8_t compressed) override;
    Stats getStats() const override { return stats; }
    void reset() override;
};

} // namespace tsb

// Display.h
namespace tsb {

class IDisplay {
public:
    virtual ~IDisplay() = default;
    virtual ErrorOr initialize(const DisplayConfig& config) = 0;
    virtual void clear() = 0;
    virtual void update() = 0;
    virtual void drawTimeSeries(const std::vector<Point>& points,
                              const Range& valueRange,
                              float timeWindow) = 0;
};

class SSD1306Display : public IDisplay {
private:
    class DisplayBuffer {
        std::vector<uint8_t> buffer;
        uint16_t width;
        uint16_t height;
        
    public:
        DisplayBuffer(uint16_t w, uint16_t h);
        void clear();
        void setPixel(uint16_t x, uint16_t y, bool value);
        void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
        const uint8_t* data() const { return buffer.data(); }
    };
    
    std::unique_ptr<DisplayBuffer> buffer;
    I2CInterface& i2c;
    DisplayConfig config;
    
public:
    explicit SSD1306Display(I2CInterface& i2c);
    ErrorOr initialize(const DisplayConfig& config) override;
    void clear() override;
    void update() override;
    void drawTimeSeries(const std::vector<Point>& points,
                       const Range& valueRange,
                       float timeWindow) override;
};

} // namespace tsb

// Buffer.h
namespace tsb {

class CircularBuffer {
private:
    std::vector<uint8_t> data;
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;
    
public:
    explicit CircularBuffer(size_t capacity);
    bool push(uint8_t value);
    std::optional<uint8_t> pop();
    size_t size() const { return count; }
    bool empty() const { return count == 0; }
    bool full() const { return count == data.size(); }
    void clear();
};

class CompressedTimeSeriesBuffer {
private:
    struct State {
        Point current;
        Range valueRange{INFINITY, -INFINITY};
        size_t count = 0;
    };
    
    BufferConfig config;
    State state;
    std::unique_ptr<CircularBuffer> buffer;
    std::unique_ptr<ICompressor> timeCompressor;
    std::unique_ptr<ICompressor> valueCompressor;
    std::unique_ptr<IDisplay> display;
    
    class ErrorTracker {
        float sumSquaredError = 0;
        size_t count = 0;
        float threshold;
        
    public:
        explicit ErrorTracker(float th) : threshold(th) {}
        void track(float error);
        bool needsRecompression() const;
        void reset();
    };
    
    std::unique_ptr<ErrorTracker> errorTracker;
    
    Result<std::vector<Point>> decompressAll() const;
    ErrorOr recompressBuffer();
    void updateValueRange(float value);
    void removeExpiredPoints();
    
public:
    CompressedTimeSeriesBuffer(const BufferConfig& cfg,
                              std::unique_ptr<ICompressor> tComp,
                              std::unique_ptr<ICompressor> vComp,
                              std::unique_ptr<IDisplay> disp);
    
    ErrorOr push(const Point& point);
    Result<Point> pop();
    Result<Point> peek() const;
    
    void clear();
    size_t size() const { return state.count; }
    
    const Point& getCurrentPoint() const { return state.current; }
    Range getValueRange() const { return state.valueRange; }
    
    ErrorOr updateDisplay();
    std::pair<Stats, Stats> getCompressionStats() const;
};

} // namespace tsb

// Main application file (main.cpp)
#include "SystemTypes.h"
#include "Compression.h"
#include "Display.h"
#include "Buffer.h"

namespace {

class TestDataGenerator {
private:
    float time = 0;
    struct {
        float mainFreq = 0.1f;
        float subFreq = 0.05f;
        float amplitude = 5.0f;
        float noiseLevel = 0.5f;
        float timeStep = 0.1f;
    } params;
    
public:
    tsb::Point getNext() {
        float value = params.amplitude * sin(2 * PI * params.mainFreq * time) +
                     random(-params.noiseLevel, params.noiseLevel) +
                     2 * sin(2 * PI * params.subFreq * time);
        
        time += params.timeStep;
        return tsb::Point(time, value);
    }
};

class Application {
private:
    std::unique_ptr<tsb::CompressedTimeSeriesBuffer> buffer;
    std::unique_ptr<TestDataGenerator> generator;
    unsigned long lastUpdate = 0;
    const unsigned long updateInterval = 100;
    
    void setupDisplay() {
        auto& i2c = I2C::getInstance();
        i2c.begin(21, 22);  // SDA, SCL pins
    }
    
    tsb::BufferConfig createConfig() {
        return {
            .capacity = 1024,
            .timeWindow = 30.0f,
            .timeCompression = {
                .baseScale = 0.1f,
                .maxError = 0.05f,
                .adaptationRate = 0.1f,
                .windowSize = 32
            },
            .valueCompression = {
                .baseScale = 0.01f,
                .maxError = 0.1f,
                .adaptationRate = 0.1f,
                .windowSize = 32
            }
        };
    }
    
    void createBuffer() {
        auto config = createConfig();
        auto timeComp = std::make_unique<tsb::AdaptiveCompressor>(
            config.timeCompression);
        auto valueComp = std::make_unique<tsb::AdaptiveCompressor>(
            config.valueCompression);
        auto display = std::make_unique<tsb::SSD1306Display>(
            I2C::getInstance());
            
        buffer = std::make_unique<tsb::CompressedTimeSeriesBuffer>(
            config,
            std::move(timeComp),
            std::move(valueComp),
            std::move(display));
    }
    
public:
    ErrorOr initialize() {
        setupDisplay();
        createBuffer();
        generator = std::make_unique<TestDataGenerator>();
        return ErrorOr::ok(true);
    }
    
    void update() {
        auto currentMillis = millis();
        if (currentMillis - lastUpdate >= updateInterval) {
            lastUpdate = currentMillis;
            
            // Get and push new data
            auto point = generator->getNext();
            if (auto result = buffer->push(point); result.isError()) {
                Serial.println(result.error.c_str());
                return;
            }
            
            // Update display
            if (auto result = buffer->updateDisplay(); result.isError()) {
                Serial.println(result.error.c_str());
                return;
            }
            
            // Print stats
            auto [timeStats, valueStats] = buffer->getCompressionStats();
            Serial.printf("Time: scale=%.3f err=%.3f n=%zu\n",
                        timeStats.scale, timeStats.error, timeStats.samples);
            Serial.printf("Value: scale=%.3f err=%.3f n=%zu\n",
                        valueStats.scale, valueStats.error, valueStats.samples);
        }
    }
};

} // anonymous namespace

std::unique_ptr<Application> app;

void setup() {
    Serial.begin(115200);
    app = std::make_unique<Application>();
    
    if (auto result = app->initialize(); result.isError()) {
        Serial.println(result.error.c_str());
        return;
    }
}

void loop() {
    app->update();
}
