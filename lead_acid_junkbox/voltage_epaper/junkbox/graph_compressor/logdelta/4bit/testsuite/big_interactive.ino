#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <circular_buffer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Configuration structure for pattern generation
struct PatternConfig {
    float frequency;      // Base frequency for periodic patterns
    float amplitude;      // Amplitude of the pattern
    float noiseLevel;     // Amount of noise to add
    float phaseShift;     // Phase shift for periodic patterns
    float offset;         // DC offset
    float burstProb;      // Probability of burst events
    float burstMagnitude; // Magnitude of burst events
};

// Configuration structure for visualization
struct DisplayConfig {
    enum class ViewMode {
        OVERLAY,           // Original and compressed signals overlaid
        SPLIT_SCREEN,      // Original top, compressed bottom
        ERROR_DISPLAY,     // Show error magnitude
        WATERFALL         // Scrolling history view
    };
    
    enum class MetricsDisplay {
        MINIMAL,          // Just basic SNR
        DETAILED,         // All compression metrics
        PATTERN_FOCUS,    // Pattern-specific stats
        ERROR_STATS      // Error distribution stats
    };
    
    ViewMode viewMode;
    MetricsDisplay metricsMode;
    bool showGrid;
    bool showLegend;
    uint8_t brightness;
    bool invertDisplay;
    bool autoScale;
    uint8_t plotThickness;
};

class CompressionDemo {
private:
    Adafruit_SSD1306 display;
    CompressedBuffer compressor;
    
    // Configuration objects
    PatternConfig patternConfig;
    DisplayConfig displayConfig;
    
    // Waterfall buffer
    static const int WATERFALL_HISTORY = 32;
    uint8_t waterfallBuffer[WATERFALL_HISTORY][SCREEN_WIDTH];
    int waterfallIndex = 0;
    
    // Auto-scaling parameters
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    float autoScaleAlpha = 0.1f; // EMA factor for auto-scaling
    
    // Test parameters
    TestPattern currentPattern = SINE_WAVE;
    float timeStep = 0;
    float testValue = 0;
    int sampleCount = 0;
    
    // Advanced pattern generation methods
    float generateCompositeSignal() {
        float value = patternConfig.offset;
        
        switch (currentPattern) {
            case SINE_WAVE:
                value += patternConfig.amplitude * sin(
                    timeStep * patternConfig.frequency + patternConfig.phaseShift);
                // Add harmonics
                value += patternConfig.amplitude * 0.3f * sin(
                    timeStep * patternConfig.frequency * 2.0f);
                value += patternConfig.amplitude * 0.15f * sin(
                    timeStep * patternConfig.frequency * 3.0f);
                break;
                
            case SQUARE_WAVE:
                value += patternConfig.amplitude * 
                    (sin(timeStep * patternConfig.frequency + patternConfig.phaseShift) > 0 ? 1 : -1);
                // Add transition smoothing
                float transition = sin(timeStep * patternConfig.frequency + patternConfig.phaseShift);
                if (abs(transition) < 0.1) {
                    value *= (abs(transition) / 0.1);
                }
                break;
                
            case RANDOM_WALK:
                static float walkValue = 0;
                walkValue += random(-100, 101) / 100.0f * patternConfig.noiseLevel;
                walkValue = constrain(walkValue, -patternConfig.amplitude, patternConfig.amplitude);
                value += walkValue;
                break;
                
            case STEP_FUNCTION:
                static float stepValue = 0;
                if (int(timeStep * patternConfig.frequency) % 20 == 0) {
                    stepValue = random(-100, 101) / 100.0f * patternConfig.amplitude;
                }
                value += stepValue;
                break;
                
            case BURST_NOISE:
                static float burstValue = 0;
                if (random(100) < patternConfig.burstProb * 100) {
                    burstValue = random(-100, 101) / 100.0f * patternConfig.burstMagnitude;
                } else {
                    burstValue *= 0.95; // Decay
                }
                value += burstValue;
                break;
        }
        
        // Add noise
        value += random(-100, 101) / 100.0f * patternConfig.noiseLevel;
        
        return value;
    }
    
    void updateAutoScale(float value) {
        if (displayConfig.autoScale) {
            minValue = min(minValue, value);
            maxValue = max(maxValue, value);
            
            // Exponential moving average for smooth scaling
            minValue += autoScaleAlpha * (value - minValue);
            maxValue += autoScaleAlpha * (value - maxValue);
        }
    }
    
    int mapValueToDisplay(float value) {
        if (displayConfig.autoScale) {
            return map(value * 100, minValue * 100, maxValue * 100, 
                      GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
        } else {
            return map(value * 100, -patternConfig.amplitude * 100, 
                      patternConfig.amplitude * 100, 
                      GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
        }
    }
    
    void drawOverlayView() {
        // Draw grid if enabled
        if (displayConfig.showGrid) {
            drawGrid();
        }
        
        // Draw signals with configured thickness
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            int idx = i % compressor.size();
            if (idx < compressor.size()) {
                // Original signal
                int y1 = mapValueToDisplay(compressor.getOriginalValue(idx));
                for (int t = 0; t < displayConfig.plotThickness; t++) {
                    display.drawPixel(i, y1 + t, SSD1306_WHITE);
                }
                
                // Compressed signal (dashed)
                int y2 = mapValueToDisplay(compressor.getDecompressedValue(idx));
                if (i % 2 == 0) { // Dashed line effect
                    for (int t = 0; t < displayConfig.plotThickness; t++) {
                        display.drawPixel(i, y2 + t, SSD1306_WHITE);
                    }
                }
            }
        }
    }
    
    void drawSplitScreenView() {
        int splitPoint = SCREEN_HEIGHT / 2;
        
        // Draw dividing line
        display.drawFastHLine(0, splitPoint, SCREEN_WIDTH, SSD1306_WHITE);
        
        // Draw signals in split view
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            int idx = i % compressor.size();
            if (idx < compressor.size()) {
                // Original signal (top)
                int y1 = mapValueToDisplay(compressor.getOriginalValue(idx)) / 2;
                display.drawPixel(i, y1, SSD1306_WHITE);
                
                // Compressed signal (bottom)
                int y2 = splitPoint + 
                        mapValueToDisplay(compressor.getDecompressedValue(idx)) / 2;
                display.drawPixel(i, y2, SSD1306_WHITE);
            }
        }
    }
    
    void drawErrorDisplay() {
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            int idx = i % compressor.size();
            if (idx < compressor.size()) {
                float error = abs(compressor.getDecompressedValue(idx) - 
                                compressor.getOriginalValue(idx));
                int errorHeight = map(error * 100, 0, 
                                    patternConfig.amplitude * 50, 0, GRAPH_HEIGHT);
                display.drawFastVLine(i, SCREEN_HEIGHT - errorHeight, 
                                    errorHeight, SSD1306_WHITE);
            }
        }
    }
    
    void updateWaterfallBuffer() {
        // Shift existing data
        memmove(&waterfallBuffer[1], &waterfallBuffer[0], 
                (WATERFALL_HISTORY - 1) * SCREEN_WIDTH);
        
        // Add new line
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            int idx = i % compressor.size();
            if (idx < compressor.size()) {
                float error = abs(compressor.getDecompressedValue(idx) - 
                                compressor.getOriginalValue(idx));
                waterfallBuffer[0][i] = map(error * 100, 0, 
                                          patternConfig.amplitude * 50, 0, 15);
            } else {
                waterfallBuffer[0][i] = 0;
            }
        }
    }
    
    void drawWaterfallView() {
        for (int y = 0; y < WATERFALL_HISTORY; y++) {
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                if (waterfallBuffer[y][x] > 0) {
                    // Create dithered effect for grayscale simulation
                    if (y % 2 == 0 && waterfallBuffer[y][x] > 7) {
                        display.drawPixel(x, SCREEN_HEIGHT - y - 1, SSD1306_WHITE);
                    } else if (y % 2 == 1 && waterfallBuffer[y][x] > 3) {
                        display.drawPixel(x, SCREEN_HEIGHT - y - 1, SSD1306_WHITE);
                    }
                }
            }
        }
    }
    
    void drawGrid() {
        // Vertical grid lines
        for (int x = 0; x < SCREEN_WIDTH; x += 16) {
            for (int y = GRAPH_Y_OFFSET; y < GRAPH_Y_OFFSET + GRAPH_HEIGHT; y += 2) {
                display.drawPixel(x, y, SSD1306_WHITE);
            }
        }
        
        // Horizontal grid lines
        for (int y = GRAPH_Y_OFFSET; y < GRAPH_Y_OFFSET + GRAPH_HEIGHT; y += 8) {
            for (int x = 0; x < SCREEN_WIDTH; x += 2) {
                display.drawPixel(x, y, SSD1306_WHITE);
            }
        }
    }
    
    void drawMetrics() {
        display.setTextSize(1);
        display.setCursor(0, TEXT_Y_OFFSET);
        
        PatternResult metrics = compressor.getPatternMetrics(
            compressor.getCurrentPattern());
        
        switch (displayConfig.metricsMode) {
            case DisplayConfig::MetricsDisplay::MINIMAL:
                display.printf("SNR:%.1fdB", metrics.snr);
                break;
                
            case DisplayConfig::MetricsDisplay::DETAILED:
                display.printf("SNR:%.1fdB Err:%.1f%% T:%.1fms", 
                    metrics.snr, 
                    (metrics.peakError / patternConfig.amplitude) * 100.0f,
                    metrics.compressionTime);
                break;
                
            case DisplayConfig::MetricsDisplay::PATTERN_FOCUS:
                display.printf("%s F:%.1f A:%.1f", 
                    getPatternName(currentPattern),
                    patternConfig.frequency,
                    patternConfig.amplitude);
                break;
                
            case DisplayConfig::MetricsDisplay::ERROR_STATS:
                GlobalStats stats = compressor.getGlobalStats();
                display.printf("ErrAvg:%.2f Max:%.2f", 
                    stats.errorMovingAverage,
                    stats.maxValue - stats.minValue);
                break;
        }
    }

public:
    CompressionDemo() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
        // Initialize default pattern configuration
        patternConfig = {
            .frequency = 0.1f,
            .amplitude = 30.0f,
            .noiseLevel = 1.0f,
            .phaseShift = 0.0f,
            .offset = 32.0f,
            .burstProb = 0.1f,
            .burstMagnitude = 20.0f
        };
        
        // Initialize default display configuration
        displayConfig = {
            .viewMode = DisplayConfig::ViewMode::OVERLAY,
            .metricsMode = DisplayConfig::MetricsDisplay::DETAILED,
            .showGrid = true,
            .showLegend = true,
            .brightness = 128,
            .invertDisplay = false,
            .autoScale = true,
            .plotThickness = 1
        };
    }
    
    void begin() {
        // Initialize display
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            return;
        }
        
        // Apply initial display settings
        display.setContrast(displayConfig.brightness);
        display.invertDisplay(displayConfig.invertDisplay);
        display.clearDisplay();
        display.display();
        
        printHelp();
    }
    
    void handleSerialCommand() {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            
            // Pattern configuration commands
            if (cmd.startsWith("f ")) {
                patternConfig.frequency = cmd.substring(2).toFloat();
            }
            else if (cmd.startsWith("a ")) {
                patternConfig.amplitude = cmd.substring(2).toFloat();
            }
            else if (cmd.startsWith("n ")) {
                patternConfig.noiseLevel = cmd.substring(2).toFloat();
            }
            else if (cmd.startsWith("p ")) {
                patternConfig.phaseShift = cmd.substring(2).toFloat();
            }
            // Display configuration commands
            else if (cmd == "view") {
                displayConfig.viewMode = static_cast<DisplayConfig::ViewMode>(
                    (static_cast<int>(displayConfig.viewMode) + 1) % 4);
            }
            else if (cmd == "metrics") {
                displayConfig.metricsMode = static_cast<DisplayConfig::MetricsDisplay>(
                    (static_cast<int>(displayConfig.metricsMode) + 1) % 4);
            }
            else if (cmd == "grid") {
                displayConfig.showGrid = !displayConfig.showGrid;
            }
            else if (cmd == "auto") {
                displayConfig.autoScale = !displayConfig.autoScale;
            }
            else if (cmd.startsWith("thick ")) {
                displayConfig.plotThickness = cmd.substring(6).toInt();
            }
            else if (cmd == "help") {
                printHelp();
            }
            
            Serial.println("OK");
        }
    }
    
    void printHelp() {
        Serial.println("\nCompression Demo Commands:");
        Serial.println("Pattern Configuration
          
