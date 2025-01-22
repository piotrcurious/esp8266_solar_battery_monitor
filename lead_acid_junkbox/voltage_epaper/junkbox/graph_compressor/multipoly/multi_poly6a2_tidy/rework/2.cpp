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

// Helper functions
static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static double evaluatePolynomial(const float* coefficients, uint8_t degree, double t) {
    double result = 0.0;
    double tPower = 1.0;
    for (int i = 0; i < degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= t;
    }
    return result;
}

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
        tft.startWrite();
        
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
    void initializeNewSegment() {
        segmentCount++;
        currentPolyIndex = 0;
        segmentBuffer[segmentCount-1].clearTimeDeltas();
    }
    
    void handleSegmentOverflow() {
        if (segmentCount < SEGMENTS) {
            initializeNewSegment();
        } else {
            recompressSegments();
            currentPolyIndex = 0;
        }
    }
    
    void recompressSegments() {
        if (segmentCount < 2) return;
        
        PolynomialSegment recompressed;
        recompressed.clearTimeDeltas();
        
        // Combine polynomials from oldest two segments
        combinePolynomials(segmentBuffer[0], segmentBuffer[1], recompressed);
        
        // Shift segments and insert recompressed
        for (uint8_t i = 0; i < segmentCount-2; i++) {
            segmentBuffer[i] = segmentBuffer[i+2];
        }
        segmentBuffer[segmentCount-2] = recompressed;
        segmentCount--;
    }
    
    void combinePolynomials(const PolynomialSegment &first, const PolynomialSegment &second, PolynomialSegment &result) {
        AdvancedPolynomialFitter fitter;
        uint16_t currentPoly = 0;
        
        for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
            if (first.timeDeltas[i] == 0 || first.timeDeltas[i+1] == 0) break;
            
            std::vector<float> timestamps;
            std::vector<float> values;
            float resolution = (first.timeDeltas[i] + first.timeDeltas[i+1]) / MEMORY_LIMIT;
            
            // Sample points from both polynomials
            samplePolynomialPoints(first.coefficients[i], first.timeDeltas[i], resolution, timestamps, values);
            samplePolynomialPoints(first.coefficients[i+1], first.timeDeltas[i+1], resolution, timestamps, values);
            
            // Fit new polynomial
            auto newCoeffs = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);
            
            // Store results
            for (uint8_t j = 0; j < newCoeffs.size() && j < POLY_DEGREE+1; j++) {
                result.coefficients[currentPoly][j] = newCoeffs[j];
            }
            result.timeDeltas[currentPoly] = first.timeDeltas[i] + first.timeDeltas[i+1];
            currentPoly++;
        }
    }
    
    void samplePolynomialPoints(const float* coeffs, uint32_t timeDelta, float resolution, 
                               std::vector<float>& timestamps, std::vector<float>& values) {
        for (float t = 0; t <= timeDelta; t += resolution) {
            timestamps.push_back(t);
            values.push_back(evaluatePolynomial(coeffs, POLY_DEGREE+1, t));
        }
    }
    
    void updateRawData(float data, uint32_t timestamp) {
        if (rawDataIndex >= MAX_RAW_DATA) {
            // Shift data when buffer is full
            memmove(rawData, rawData + 1, (MAX_RAW_DATA - 1) * sizeof(float));
            memmove(rawTimestamps, rawTimestamps + 1, (MAX_RAW_DATA - 1) * sizeof(uint32_t));
            rawDataIndex--;
        }
        
        rawData[rawDataIndex] = data;
        rawTimestamps[rawDataIndex] = timestamp;
        rawDataIndex++;
        
        // Update min/max values
        updateMinMax();
    }
    
    void updateMinMax() {
        rawGraphMinY = rawData[0];
        rawGraphMaxY = rawData[0];
        for (uint16_t i = 1; i < rawDataIndex; i++) {
            rawGraphMinY = min(rawGraphMinY, rawData[i]);
            rawGraphMaxY = max(rawGraphMaxY, rawData[i]);
        }
    }
    
    void drawRawData() {
        if (rawDataIndex < 2) return;
        
        uint32_t timeStart = rawTimestamps[0];
        uint32_t timeEnd = rawTimestamps[rawDataIndex - 1];
        
        for (uint16_t i = 1; i < rawDataIndex; i++) {
            uint16_t x1 = mapFloat(rawTimestamps[i-1], timeStart, timeEnd, 0, SCREEN_WIDTH - 1);
            uint16_t x2 = mapFloat(rawTimestamps[i], timeStart, timeEnd, 0, SCREEN_WIDTH - 1);
            uint16_t y1 = mapFloat(rawData[i-1], rawGraphMinY, rawGraphMaxY, SCREEN_HEIGHT - 1, 0);
            uint16_t y2 = mapFloat(rawData[i], rawGraphMinY, rawGraphMaxY, SCREEN_HEIGHT - 1, 0);
            
            tft.drawLine(x1, y1, x2, y2, COLOR_RAW_DATA);
        }
    }
    
    void drawCompressedData() {
        if (segmentCount == 0) return;
        
        uint32_t timeStart = rawTimestamps[0];
        uint32_t timeEnd = rawTimestamps[rawDataIndex - 1] - rawLogDelta;
        uint16_t lastX = -1, lastY = -1;
        
        for (uint16_t x = 0; x < SCREEN_WIDTH; x++) {
            uint32_t t = mapFloat(x, 0, SCREEN_WIDTH - 1, timeStart, timeEnd);
            float value = evaluateCompressedValue(t);
            
            if (!isnan(value)) {
                uint16_t y = mapFloat(value, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
                if (lastX >= 0) {
                    tft.drawLine(lastX, lastY, x, y, COLOR_COMPRESSED);
                }
                lastX = x;
                lastY = y;
            }
        }
    }
    
    float evaluateCompressedValue(uint32_t timestamp) {
        // Find the appropriate segment and polynomial
        uint32_t tCurrent = timestamp;
        for (uint8_t i = 0; i < segmentCount; i++) {
            for (uint8_t j = 0; j < POLY_COUNT; j++) {
                if (segmentBuffer[i].timeDeltas[j] == 0) continue;
                
                if (tCurrent <= segmentBuffer[i].timeDeltas[j]) {
                    return evaluatePolynomial(segmentBuffer[i].coefficients[j], POLY_DEGREE+1, tCurrent);
                }
                tCurrent -= segmentBuffer[i].timeDeltas[j];
            }
        }
        return NAN;
    }
};

// Sample data generation function
float sampleData(uint32_t timestamp) {
    float scalar = random(0, 1000 * sin((float)timestamp * 0.00002)) / 100.0;
    return scalar + 10 * sin((float)timestamp * 0.0001) + 20 * sin((float)timestamp * 0.00001);
}

// Main program
TFT_eSPI tft;
DataCompressor compressor(tft);

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));
    
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
