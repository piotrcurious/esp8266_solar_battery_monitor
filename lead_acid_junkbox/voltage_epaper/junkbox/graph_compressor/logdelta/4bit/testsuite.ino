#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <circular_buffer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Test patterns for compression evaluation
enum TestPattern {
    SINE_WAVE,
    SQUARE_WAVE,
    RANDOM_WALK,
    STEP_FUNCTION,
    BURST_NOISE
};

class CompressionDemo {
private:
    Adafruit_SSD1306 display;
    CompressedBuffer compressor;
    
    // Display layout constants
    static const int GRAPH_HEIGHT = 40;
    static const int GRAPH_Y_OFFSET = 12;
    static const int TEXT_Y_OFFSET = 55;
    static const int NUM_TEST_PATTERNS = 5;
    
    // Test parameters
    TestPattern currentPattern = SINE_WAVE;
    float timeStep = 0;
    float testValue = 0;
    int sampleCount = 0;
    
    // Statistics tracking
    struct TestStats {
        float avgSNR;
        float avgCompressTime;
        float peakError;
        int sampleCount;
    };
    TestStats patternStats[NUM_TEST_PATTERNS] = {0};
    
    void generateTestValue() {
        switch (currentPattern) {
            case SINE_WAVE:
                testValue = 32 + 30 * sin(timeStep * 0.1);
                break;
                
            case SQUARE_WAVE:
                testValue = (sin(timeStep * 0.05) > 0) ? 62 : 2;
                break;
                
            case RANDOM_WALK:
                testValue = constrain(testValue + random(-4, 5), 2, 62);
                break;
                
            case STEP_FUNCTION:
                if (int(timeStep) % 40 == 0) {
                    testValue = random(2, 62);
                }
                break;
                
            case BURST_NOISE:
                if (random(100) < 10) {  // 10% chance of burst
                    testValue = random(2, 62);
                } else {
                    testValue += random(-2, 3);
                    testValue = constrain(testValue, 2, 62);
                }
                break;
        }
        timeStep += 1.0;
    }
    
    void updateDisplay() {
        display.clearDisplay();
        
        // Draw title
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Pattern: ");
        display.println(getPatternName(currentPattern));
        
        // Draw original and compressed signals
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            int idx = i % compressor.size();
            if (idx < compressor.size()) {
                // Original signal
                int y1 = GRAPH_Y_OFFSET + SCREEN_HEIGHT - 
                        int(compressor.getOriginalValue(idx));
                display.drawPixel(i, y1, SSD1306_WHITE);
                
                // Compressed signal
                int y2 = GRAPH_Y_OFFSET + SCREEN_HEIGHT - 
                        int(compressor.getDecompressedValue(idx));
                display.drawPixel(i, y2, SSD1306_WHITE);
            }
        }
        
        // Draw statistics
        display.setCursor(0, TEXT_Y_OFFSET);
        GlobalStats stats = compressor.getGlobalStats();
        PatternResult metrics = compressor.getPatternMetrics(
            compressor.getCurrentPattern());
        
        display.printf("SNR:%.1fdB Err:%.1f%%", 
                      metrics.snr, 
                      (metrics.peakError / 64.0f) * 100.0f);
        
        display.display();
    }
    
    const char* getPatternName(TestPattern pattern) {
        switch (pattern) {
            case SINE_WAVE: return "Sine";
            case SQUARE_WAVE: return "Square";
            case RANDOM_WALK: return "Random Walk";
            case STEP_FUNCTION: return "Step";
            case BURST_NOISE: return "Burst";
            default: return "Unknown";
        }
    }
    
    void updateStats(TestPattern pattern, const PatternResult& metrics) {
        TestStats& stats = patternStats[pattern];
        stats.avgSNR = (stats.avgSNR * stats.sampleCount + metrics.snr) / 
                       (stats.sampleCount + 1);
        stats.avgCompressTime = (stats.avgCompressTime * stats.sampleCount + 
                                metrics.compressionTime) / (stats.sampleCount + 1);
        stats.peakError = max(stats.peakError, metrics.peakError);
        stats.sampleCount++;
    }

public:
    CompressionDemo() : 
        display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
    }
    
    void begin() {
        // Initialize OLED
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            return;
        }
        display.clearDisplay();
        display.display();
        
        // Initialize random seed
        randomSeed(analogRead(0));
        
        // Enable parallel processing in compressor
        compressor.setParallelProcessing(true);
        
        Serial.println("Compression Demo Started");
        Serial.println("Commands:");
        Serial.println("p: Change pattern");
        Serial.println("s: Show statistics");
        Serial.println("r: Reset statistics");
    }
    
    void update() {
        generateTestValue();
        
        // Add sample to compressor
        compressor.addSample(timeStep, testValue);
        
        // Update statistics
        if (sampleCount % 10 == 0) {  // Update stats every 10 samples
            PatternResult metrics = compressor.getPatternMetrics(
                compressor.getCurrentPattern());
            updateStats(currentPattern, metrics);
        }
        
        updateDisplay();
        sampleCount++;
    }
    
    void handleSerial() {
        if (Serial.available()) {
            char cmd = Serial.read();
            switch (cmd) {
                case 'p':
                    currentPattern = static_cast<TestPattern>(
                        (currentPattern + 1) % NUM_TEST_PATTERNS);
                    Serial.printf("Pattern: %s\n", 
                                getPatternName(currentPattern));
                    break;
                    
                case 's':
                    printStatistics();
                    break;
                    
                case 'r':
                    resetStatistics();
                    Serial.println("Statistics reset");
                    break;
            }
        }
    }
    
    void printStatistics() {
        Serial.println("\nCompression Statistics:");
        Serial.println("Pattern      SNR(dB)  Time(ms)  Peak Err");
        Serial.println("----------------------------------------");
        
        for (int i = 0; i < NUM_TEST_PATTERNS; i++) {
            TestStats& stats = patternStats[i];
            if (stats.sampleCount > 0) {
                Serial.printf("%-12s %6.1f   %6.2f   %6.1f%%\n",
                    getPatternName(static_cast<TestPattern>(i)),
                    stats.avgSNR,
                    stats.avgCompressTime,
                    (stats.peakError / 64.0f) * 100.0f
                );
            }
        }
        Serial.println();
    }
    
    void resetStatistics() {
        memset(patternStats, 0, sizeof(patternStats));
    }
};

// Main program
CompressionDemo demo;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Wire.begin();
    demo.begin();
}

void loop() {
    demo.update();
    demo.handleSerial();
    delay(50);  // 20 Hz update rate
}
