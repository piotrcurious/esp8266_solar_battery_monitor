#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(); // Create TFT instance
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_HEIGHT 240
#define RAW_GRAPH_Y 0
#define COMPRESSED_GRAPH_Y 0

#include "AdvancedPolynomialFitter.hpp" // Include the advanced fitter
#include <vector>
#include <Arduino.h>

//for debug
#define MAX_RAW_DATA 1024
//#define MAX_RAW_DATA_GRAIN 32 // after how many points added to the raw data buffer data is recompressed. 
#define LOG_BUFFER_POINTS_PER_POLY 64 // replaced by this

static float    raw_Data[MAX_RAW_DATA];
static uint32_t raw_timestamps[MAX_RAW_DATA];
float raw_graphMinY = 0;
float raw_graphMaxY = 0;
static uint16_t dataIndex = 0;

#include <stdint.h>

#define POLY_COUNT 8 // Number of polynomials in each segment
#define SEGMENTS 4    // Total number of segments

// Storage structure
struct PolynomialSegment {
    float coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), full resolution
    uint16_t timeDeltas[POLY_COUNT];   // 16-bit time deltas
};

// Data buffer
PolynomialSegment segmentBuffer[SEGMENTS];
uint8_t segmentCount = 0;
uint32_t lastTimestamp = 0;

// Rolling buffer indices
uint8_t head = 0; // Points to the oldest segment
uint8_t tail = 0; // Points to the newest segment
uint8_t count = 0; // Number of valid segments in buffer

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Add a new segment to the buffer
void addSegment(const PolynomialSegment &newSegment) {
    segmentBuffer[tail] = newSegment;
    tail = (tail + 1) % SEGMENTS;

    if (count < SEGMENTS) {
        count++;
    } else {
        // Buffer full, advance head to discard the oldest segment
        head = (head + 1) % SEGMENTS;
    }
}

// Check if the buffer is full
bool isBufferFull() {
    return count == SEGMENTS;
}

// Retrieve the oldest and second-oldest segments
void getOldestSegments(PolynomialSegment &oldest, PolynomialSegment &secondOldest) {
    oldest = segmentBuffer[head];
    secondOldest = segmentBuffer[(head + 1) % SEGMENTS];
}

// Remove the oldest two segments
void removeOldestTwo() {
    head = (head + 2) % SEGMENTS;
    count -= 2;
}

#include <math.h>

void compressDataToSegment(const float *rawData, const uint16_t *timestamps, uint16_t dataSize, float *coefficients, uint16_t &timeDelta) {
    AdvancedPolynomialFitter fitter;

    std::vector<float> x(dataSize);
    std::vector<float> y(dataSize);

    float timestamp_absolute = 0;
    // Accumulate timestamps and collect data points
    for (uint16_t j = 0; j < dataSize; j++) {
        x[j] = timestamp_absolute;
        y[j] = rawData[j];
        timestamp_absolute += timestamps[j];
    }

    // Fit polynomial to this chunk
    std::vector<float> fitted_coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::NONE);

    // Store coefficients
    for (uint8_t j = 0; j < fitted_coefficients.size() && j < 6; j++) {
        coefficients[j] = fitted_coefficients[j];
    }

    // Store the time delta
    timeDelta = timestamp_absolute;
}



// Evaluate a polynomial at a given timestamp
float evaluatePolynomial(const float *coefficients, float tNormalized) {
    float result = 0.0;
    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * pow(tNormalized, i);
    }
    return result;
}

// Combine two segments by evaluating their polynomials at normalized timestamps

void generateCombinedData(PolynomialSegment &seg1, uint8_t poly1_start, uint8_t poly1_end,
                         PolynomialSegment &seg2, uint8_t poly2_start, uint8_t poly2_end,
                         float *combinedData, uint16_t *combinedTimestamps, uint16_t &combinedSize) {
    combinedSize = 0;
    const uint16_t SAMPLES_PER_POLY = 8;  // Number of samples per polynomial interval

    // Process polynomials from first segment
    uint32_t tAccum = 0;
    for (uint8_t i = poly1_start; i <= poly1_end; i++) {
        if (seg1.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg1.timeDeltas[i];
        for (uint16_t j = 0; j < SAMPLES_PER_POLY; j++) {
            float tNorm = (float)j / (SAMPLES_PER_POLY - 1);
            float value = evaluatePolynomial(seg1.coefficients[i], tNorm);
            
            combinedData[combinedSize] = value;
            combinedTimestamps[combinedSize] = tDelta / SAMPLES_PER_POLY;
            combinedSize++;
        }
        tAccum += tDelta;
    }

    // Process polynomials from second segment
    for (uint8_t i = poly2_start; i <= poly2_end; i++) {
        if (seg2.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg2.timeDeltas[i];
        for (uint16_t j = 0; j < SAMPLES_PER_POLY; j++) {
            float tNorm = (float)j / (SAMPLES_PER_POLY - 1);
            float value = evaluatePolynomial(seg2.coefficients[i], tNorm);
            
            combinedData[combinedSize] = value;
            combinedTimestamps[combinedSize] = tDelta / SAMPLES_PER_POLY;
            combinedSize++;
        }
        tAccum += tDelta;
    }
}

// Stack two segments and fit new polynomials to the combined data

void stackPolynomials(const PolynomialSegment &seg1, const PolynomialSegment &seg2, PolynomialSegment &result) {
    // Initialize result
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        result.timeDeltas[i] = 0;
    }

    const uint8_t POLYS_TO_COMBINE = 4;  // Number of polynomials to combine into one
    uint8_t resultPolyIndex = 0;
    
    // Process first segment
    for (uint8_t i = 0; i < POLY_COUNT && resultPolyIndex < POLY_COUNT; i += POLYS_TO_COMBINE) {
        // Check if we have valid polynomials
        if (seg1.timeDeltas[i] == 0) break;
        
        // Determine end index for this group
        uint8_t compare1 = i + POLYS_TO_COMBINE - 1;
        uint8_t compare2 = (POLY_COUNT - 1);
        uint8_t endIdx = min(compare1, compare2);
        while (endIdx > i && seg1.timeDeltas[endIdx] == 0) endIdx--;
        
        // Generate combined data points
        float combinedData[POLY_COUNT * 8];  // Plenty of space for samples
        uint16_t combinedTimestamps[POLY_COUNT * 8];
        uint16_t combinedSize = 0;
        
        PolynomialSegment& seg1_nonconst = const_cast<PolynomialSegment&>(seg1);
        PolynomialSegment& seg2_nonconst = const_cast<PolynomialSegment&>(seg2);
        
        generateCombinedData(seg1_nonconst, i, endIdx,
                           seg2_nonconst, 0, -1,  // Don't include seg2 yet
                           combinedData, combinedTimestamps, combinedSize);
        
        if (combinedSize >= 4) {  // Need at least 4 points for fitting
            float coefficients[6];
            uint16_t timeDelta;
            
            compressDataToSegment(combinedData, combinedTimestamps, combinedSize,
                                coefficients, timeDelta);
            
            // Store the result
            for (uint8_t j = 0; j < 6; j++) {
                result.coefficients[resultPolyIndex][j] = coefficients[j];
            }
            result.timeDeltas[resultPolyIndex] = timeDelta;
            
            Serial.print("Combined polynomials ");
            Serial.print(i);
            Serial.print(" to ");
            Serial.print(endIdx);
            Serial.print(" into result polynomial ");
            Serial.println(resultPolyIndex);
            
            resultPolyIndex++;
        }
    }
    
    // Process second segment similarly
    uint8_t startFromPoly = 0;
    for (uint8_t i = 0; i < POLY_COUNT && resultPolyIndex < POLY_COUNT; i += POLYS_TO_COMBINE) {
        if (seg2.timeDeltas[i] == 0) break;

        uint8_t compare1 = i + POLYS_TO_COMBINE - 1;
        uint8_t compare2 = (POLY_COUNT - 1);
        
        uint8_t endIdx = min(compare1, compare2);
        while (endIdx > i && seg2.timeDeltas[endIdx] == 0) endIdx--;
        
        float combinedData[POLY_COUNT * 8];
        uint16_t combinedTimestamps[POLY_COUNT * 8];
        uint16_t combinedSize = 0;
        
        PolynomialSegment& seg1_nonconst = const_cast<PolynomialSegment&>(seg1);
        PolynomialSegment& seg2_nonconst = const_cast<PolynomialSegment&>(seg2);
        
        generateCombinedData(seg2_nonconst, i, endIdx,
                           seg1_nonconst, 0, -1,  // Don't include seg1
                           combinedData, combinedTimestamps, combinedSize);
        
        if (combinedSize >= 4) {
            float coefficients[6];
            uint16_t timeDelta;
            
            compressDataToSegment(combinedData, combinedTimestamps, combinedSize,
                                coefficients, timeDelta);
            
            for (uint8_t j = 0; j < 6; j++) {
                result.coefficients[resultPolyIndex][j] = coefficients[j];
            }
            result.timeDeltas[resultPolyIndex] = timeDelta;
            
            Serial.print("Combined polynomials from seg2 ");
            Serial.print(i);
            Serial.print(" to ");
            Serial.print(endIdx);
            Serial.print(" into result polynomial ");
            Serial.println(resultPolyIndex);
            
            resultPolyIndex++;
        }
    }

    Serial.print("Final polynomial count after stacking: ");
    Serial.println(resultPolyIndex);
}

void recompressSegments() {
    if (segmentCount < 2) return;

    // Take the oldest two segments and recompress them
    PolynomialSegment recompressedSegment;
    
    // Initialize timeDeltas for recompressed segment
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        recompressedSegment.timeDeltas[i] = 0;
    }

    stackPolynomials(segmentBuffer[0], segmentBuffer[1], recompressedSegment);

    // Shift all segments down
    for (uint8_t i = 0; i < segmentCount - 2; i++) {
        segmentBuffer[i] = segmentBuffer[i + 2];
    }
    
    // Place recompressed segment
    segmentBuffer[segmentCount - 2] = recompressedSegment;
    segmentCount--;

    Serial.print("Recompressed. New segment count: ");
    Serial.println(segmentCount);
}


// Sample scalar data (simulated random data for now)
float sampleScalarData(uint32_t timestamp) {
    float scalar = random(0, 1000) / 100.0; // background noise
    scalar = scalar + 20 * sin((float)timestamp * 0.0001);

    return scalar; // Random data in range [0, 10.0]
}


void logSampledData(float data, uint32_t currentTimestamp) {
 //   static float rawData[POLY_COUNT];
 //   static uint16_t timestamps[POLY_COUNT];
    static float rawData[LOG_BUFFER_POINTS_PER_POLY];
    static uint16_t timestamps[LOG_BUFFER_POINTS_PER_POLY];
 
    static uint16_t dataIndex = 0;
    static uint16_t currentPolyIndex = 0;  // Track position within current segment

    // Calculate time delta
    uint16_t timeDelta = (uint16_t)(currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;

    // Store the data and timestamp
    rawData[dataIndex] = data;
    timestamps[dataIndex] = timeDelta;
    dataIndex++;

    // Check if we have enough data for a polynomial (using smaller chunks)
    const uint16_t POINTS_PER_POLY = LOG_BUFFER_POINTS_PER_POLY;  // Adjust this value as needed
    if (dataIndex >= POINTS_PER_POLY) {
        // If we don't have any segments yet, create the first one
        if (segmentCount == 0) {
            segmentCount++;
            currentPolyIndex = 0;
            // Initialize timeDeltas to 0 for new segment
            for (uint16_t i = 0; i < POLY_COUNT; i++) {
                segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
            }
        }

        // Fit polynomial to current data chunk
        float new_coefficients[6];
        uint16_t new_timeDelta;
        compressDataToSegment(rawData, timestamps, POINTS_PER_POLY, new_coefficients, new_timeDelta);

        // Store the polynomial in current segment
        for (uint8_t i = 0; i < 6; i++) {
            segmentBuffer[segmentCount-1].coefficients[currentPolyIndex][i] = new_coefficients[i];
        }
        segmentBuffer[segmentCount-1].timeDeltas[currentPolyIndex] = new_timeDelta;

        Serial.print("Added polynomial ");
        Serial.print(currentPolyIndex);
        Serial.print(" to segment ");
        Serial.println(segmentCount-1);

        currentPolyIndex++;

        // If current segment is full, prepare for next segment
        if (currentPolyIndex >= POLY_COUNT) {
            if (segmentCount < SEGMENTS) {
                // Create new segment
                segmentCount++;
                currentPolyIndex = 0;
                // Initialize timeDeltas for new segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                }
                Serial.print("Created new segment ");
                Serial.println(segmentCount-1);
            } else {
                // Need to recompress
                Serial.println("Triggering recompression...");
                recompressSegments();
                currentPolyIndex = 0;  // Start filling the freed segment
                // Initialize timeDeltas for freed segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                }
            }
        }

        // Shift remaining data to start of buffer
        uint16_t remaining = dataIndex - POINTS_PER_POLY;
        for (uint16_t i = 0; i < remaining; i++) {
            rawData[i] = rawData[POINTS_PER_POLY + i];
            timestamps[i] = timestamps[POINTS_PER_POLY + i];
        }
        dataIndex = remaining;
    }
}


// Log sampled data into the current segment
void raw_logSampledData(float data, uint32_t currentTimestamp) {
    // Check if the current segment is full
    if (dataIndex >= MAX_RAW_DATA - 1) {
        raw_graphMinY = raw_graphMaxY;
        raw_graphMaxY = 0;
        for (uint16_t i = 0; i < dataIndex; i++) {
            raw_Data[i] = raw_Data[i + 1];
            raw_timestamps[i] = raw_timestamps[i + 1];
            if (raw_Data[i] > raw_graphMaxY) {
                raw_graphMaxY = raw_Data[i];
            }
            if (raw_Data[i] < raw_graphMinY) {
                raw_graphMinY = raw_Data[i];
            }
        }
        raw_Data[dataIndex] = data;
        raw_timestamps[dataIndex] = currentTimestamp;
    } else {
        // Store the data and timestamp
        raw_Data[dataIndex] = data;
        raw_timestamps[dataIndex] = currentTimestamp;
        raw_graphMinY = raw_graphMaxY;
        raw_graphMaxY = 0;
        for (uint16_t i = 0; i < dataIndex; i++) {
            if (raw_Data[i] > raw_graphMaxY) {
                raw_graphMaxY = raw_Data[i];
            }
            if (raw_Data[i] < raw_graphMinY) {
                raw_graphMinY = raw_Data[i];
            }
        }
        dataIndex++;
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));

    // TFT initialization
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);

    // Draw static labels
    tft.drawString("Raw Data", 10, 5, 2);
    tft.drawString("Compressed Data", 10, COMPRESSED_GRAPH_Y - 15, 2);
}

void drawRawGraph() {
    tft.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT - 1, TFT_RED);

    // Draw the new point
    for (uint16_t i = 0; i < dataIndex; i++) {
        uint16_t y = mapFloat(raw_Data[i], raw_graphMinY, raw_graphMaxY, SCREEN_HEIGHT - 1, 0);
        uint16_t x = mapFloat(raw_timestamps[i], raw_timestamps[0], raw_timestamps[dataIndex - 1], 0, SCREEN_WIDTH);
        tft.drawPixel(x, y, TFT_GREEN);
    }
}


void updateCompressedGraph(const PolynomialSegment *segments, uint8_t segmentCount) {
    if (segmentCount == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // First pass: find value range across all segments
    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];
        
        uint32_t tCurrent = 0;
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            // Sample polynomial at regular intervals within its tDelta range
            for (uint32_t t = 0; t <= tDelta; t += tDelta/10) {
               // float tNormalized = (float)t / tDelta; // Normalize time to [0,1]
//                Serial.println(t);
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
            }
            tCurrent += tDelta;
        }
    }

    // Ensure we have valid min/max values
    if (isinf(minValue) || isinf(maxValue)) {
        minValue = raw_graphMinY;
        maxValue = raw_graphMaxY;
    }

    // Add small margin to prevent values from touching screen edges
    float valueRange = maxValue - minValue;
    minValue -= valueRange * 0.05;
    maxValue += valueRange * 0.05;

    // Second pass: plot the data
    uint32_t tCurrent = 0;
    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tStart = tCurrent;
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t tEnd = tStart + tDelta;
            
            // Skip polynomials outside the visible time window
            if (tStart > windowEnd || tEnd < windowStart) {
                tCurrent = tEnd;
                continue;
            }

            // Plot with higher resolution for smoother curves
            uint32_t numSteps = min(50UL, tDelta); // Limit maximum number of points per polynomial
            uint32_t stepSize = tDelta / numSteps;
            float lastX = -1, lastY = -1;
            
            for (uint32_t t = 0; t <= tDelta; t += stepSize) {
                uint32_t tAbsolute = tStart + t;
                
                // Skip points outside the visible window
                if (tAbsolute < windowStart || tAbsolute > windowEnd) continue;
                
                // Normalize t to [0,1] for polynomial evaluation
  //              float tNormalized = (float)t / tDelta;
 //               float value = evaluatePolynomial(segment.coefficients[polyIndex], tNormalized);
               float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                 
                // Map to screen coordinates
                uint16_t x = mapFloat(tAbsolute, windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
                uint16_t y = mapFloat(value, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
                
                // Ensure coordinates are within screen bounds
                x = constrain(x, 0, SCREEN_WIDTH - 1);
                y = constrain(y, 0, SCREEN_HEIGHT - 1);
                
                // Draw line to current point if we have a previous point
                if (lastX >= 0 && lastY >= 0) {
                    tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
                } else {
                    tft.drawPixel(x, y, TFT_YELLOW);
                }
                
                lastX = x;
                lastY = y;
            }
            
            tCurrent = tEnd;
        }
    }
}


void loop() {
    // Simulate sampling at random intervals
    delay(random(10, 100)); // Random delay between 50 ms to 500 ms
    uint32_t currentTimestamp = millis();

    // Sample scalar data
    float sampledData = sampleScalarData(currentTimestamp);

    // Log the sampled data
    logSampledData(sampledData, currentTimestamp);

    // Log in the raw form (for debug purposes)
    raw_logSampledData(sampledData, currentTimestamp);

    tft.fillScreen(TFT_BLACK);
    // Update the raw data graph
    drawRawGraph();

    // Update the compressed data graph
    updateCompressedGraph(segmentBuffer, segmentCount);
}
