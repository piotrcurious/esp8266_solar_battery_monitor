#include <TFT_eSPI.h>
#include "AdvancedPolynomialFitter.hpp"
#include <vector>
#include <Arduino.h>
#include <algorithm>
#include <cmath>

// Configuration constants
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define MAX_RAW_DATA 1440  // 24h of minute-by-minute data
#define LOG_BUFFER_POINTS_PER_POLY 60
#define POLY_COUNT 8
#define SEGMENTS 2
#define POLY_DEGREE 5
#define SUB_FIT_POLY_DEGREE 3
#define BOUNDARY_MARGIN 16
#define BOUNDARY_DELTA 100
#define MEMORY_LIMIT 1000

// Display color definitions
#define COLOR_BACKGROUND TFT_BLACK
#define COLOR_GRID 0x2104      // Dark gray
#define COLOR_RAW_DATA TFT_BLUE
#define COLOR_COMPRESSED TFT_YELLOW
#define COLOR_BOUNDARY TFT_RED

struct PolynomialSegment {
    float coefficients[POLY_COUNT][POLY_DEGREE + 1];
    uint32_t timeDeltas[POLY_COUNT];
    
    void clearTimeDeltas() {
        memset(timeDeltas, 0, sizeof(timeDeltas));
    }
};

class DataCompressor {
private:
    TFT_eSPI& tft;
    PolynomialSegment segmentBuffer[SEGMENTS];
    uint8_t segmentCount = 0;
    uint16_t currentPolyIndex = 0;
    uint32_t lastTimestamp = 0;
    
    // Raw data storage
    float rawData[MAX_RAW_DATA];
    uint32_t rawTimestamps[MAX_RAW_DATA];
    uint16_t rawDataIndex = 0;
    float rawGraphMinY = 0, rawGraphMaxY = 0;
    uint32_t rawLogDelta = 0;
    
    // Visualization parameters
    float minValue = INFINITY, maxValue = -INFINITY;
    
public:
    DataCompressor(TFT_eSPI& display) : tft(display) {}
    
    void init() {
        tft.fillScreen(COLOR_BACKGROUND);
        drawGrid();
    }
    
    void drawGrid() {
        // Draw horizontal grid lines
        for (int y = 0; y < SCREEN_HEIGHT; y += 40) {
            tft.drawFastHLine(0, y, SCREEN_WIDTH, COLOR_GRID);
        }
        // Draw vertical grid lines
        for (int x = 0; x < SCREEN_WIDTH; x += 40) {
            tft.drawFastVLine(x, 0, SCREEN_HEIGHT, COLOR_GRID);
        }
    }
    
    void logData(float data, uint32_t timestamp) {
        static float buffer[LOG_BUFFER_POINTS_PER_POLY];
        static uint32_t timestamps[LOG_BUFFER_POINTS_PER_POLY];
        static uint16_t bufferIndex = 0;
        
        uint32_t timeDelta = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        
        buffer[bufferIndex] = data;
        timestamps[bufferIndex] = timeDelta;
        bufferIndex++;
        rawLogDelta += timeDelta;
        
        if (bufferIndex >= LOG_BUFFER_POINTS_PER_POLY) {
            processBuffer(buffer, timestamps, bufferIndex);
            bufferIndex = 0;
        }
        
        updateRawData(data, timestamp);
    }
    
    void updateDisplay() {
        tft.startWrite(); // Batch write operations for better performance
        
        // Clear only the necessary portion of the screen
        uint16_t clearWidth = mapFloat(rawLogDelta, 0, rawTimestamps[rawDataIndex-1] - rawTimestamps[0], 0, SCREEN_WIDTH);
        if (clearWidth > 0) {
            tft.fillRect(SCREEN_WIDTH - clearWidth, 0, clearWidth, SCREEN_HEIGHT, COLOR_BACKGROUND);
        }
        
        drawGrid();
        drawRawData();
        drawCompressedData();
        
        tft.endWrite();
    }
    
private:
    void processBuffer(float* buffer, uint32_t* timestamps, uint16_t size) {
        if (segmentCount == 0) {
            initializeNewSegment();
        }
        
        if (currentPolyIndex >= POLY_COUNT) {
            handleSegmentOverflow();
        }
        
        fitPolynomial(buffer, timestamps, size);
    }
    
    void fitPolynomial(float* data, uint32_t* timestamps, uint16_t size) {
        AdvancedPolynomialFitter fitter;
        std::vector<float> x(size), y(size);
        
        // Prepare data for fitting
        float timeAccum = 0;
        for (uint16_t i = 0; i < size; i++) {
            x[i] = timeAccum;
            y[i] = data[i];
            timeAccum += timestamps[i];
        }
        
        // Fit polynomial with boundary conditions
        auto coefficients = fitter.fitPolynomial(x, y, POLY_DEGREE, AdvancedPolynomialFitter::NONE);
        
        // Store results
        for (uint8_t i = 0; i < coefficients.size() && i < POLY_DEGREE + 1; i++) {
            segmentBuffer[segmentCount-1].coefficients[currentPolyIndex][i] = coefficients[i];
        }
        segmentBuffer[segmentCount-1].timeDeltas[currentPolyIndex] = timeAccum;
        
        currentPolyIndex++;
    }
    
    // Additional helper methods for initialization, overflow handling, 
    // and visualization would go here...
};

// Main program
TFT_eSPI tft;
DataCompressor compressor(tft);

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(1);
    compressor.init();
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (now - lastUpdate >= 100) {  // 10Hz update rate
        float data = sampleData(now);
        compressor.logData(data, now);
        compressor.updateDisplay();
        lastUpdate = now;
    }
}
