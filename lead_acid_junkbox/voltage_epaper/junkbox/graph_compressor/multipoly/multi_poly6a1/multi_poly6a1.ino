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
#define MAX_RAW_DATA 2048
//#define MAX_RAW_DATA_GRAIN 32 // after how many points added to the raw data buffer data is recompressed. 
#define LOG_BUFFER_POINTS_PER_POLY 16 // replaced by this

static float    raw_Data[MAX_RAW_DATA];
static uint32_t raw_timestamps[MAX_RAW_DATA];
static uint16_t raw_dataIndex = 0; 
float raw_graphMinY = 0;
float raw_graphMaxY = 0;
//static uint16_t dataIndex = 0;

#include <stdint.h>

#define POLY_COUNT 4 // Number of polynomials in each segment
#define SEGMENTS 4    // Total number of segments
const uint8_t POLYS_TO_COMBINE = 2;  // Number of polynomials to combine into one when recompression is triggered


// Storage structure
struct PolynomialSegment {
    float coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), full resolution
    uint32_t timeDeltas[POLY_COUNT];   // 32-bit time deltas
};

// Data buffer
PolynomialSegment segmentBuffer[SEGMENTS];
uint8_t segmentCount = 0;
uint32_t lastTimestamp = 0;

// Rolling buffer indices
uint8_t head = 0; // Points to the oldest segment
uint8_t tail = 0; // Points to the newest segment
//uint8_t segmentCount = 0; // Number of valid segments in buffer

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Add a new segment to the buffer
void addSegment(const PolynomialSegment &newSegment) {
    segmentBuffer[segmentCount] = newSegment;
//    tail = (tail + 1) % SEGMENTS;
      tail = tail+1; 
      segmentCount++;
    if (segmentCount <= SEGMENTS) {
    } else {
        // Buffer full, advance head to discard the oldest segment
//        head = (head + 1) % SEGMENTS;

    }
}

// Check if the buffer is full
bool isBufferFull() {
    return segmentCount == SEGMENTS;
}

// Retrieve the oldest and second-oldest segments
void getOldestSegments(PolynomialSegment &oldest, PolynomialSegment &secondOldest) {
    oldest = segmentBuffer[head];
    secondOldest = segmentBuffer[(head + 1) % SEGMENTS];
}

// Remove the oldest two segments
void removeOldestTwo() {
    //head = (head + 2) % SEGMENTS;
    segmentCount -= 2;
}

#include <math.h>

void compressDataToSegment(const float *rawData, const uint32_t *timestamps, uint16_t dataSize, float *coefficients, uint32_t &timeDelta) {
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
/* // on some systems this is faster
float evaluatePolynomial(const float *coefficients, float t) {
    // t is already in milliseconds within the segment's time delta range
    float result = 0.0;
    float tPower = 1.0;  // t^0 = 1
    
    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * tPower;
        tPower *= t;  // More efficient than pow()
    }
    return result;
}
*/




void combinePolynomials(const PolynomialSegment &oldest, const PolynomialSegment &secondOldest, PolynomialSegment &recompressedSegment) {
    AdvancedPolynomialFitter fitter;

    uint16_t currentPolyIndex = 0;

    for (uint16_t i = 0; i < POLY_COUNT; i=i+2) {
        // Stop if no more valid time deltas
        if (oldest.timeDeltas[i] == 0 || oldest.timeDeltas[i+1] == 0) break;
        // Reconstruct data from both polynomials
        std::vector<float> timestamps;
        std::vector<float> values;

        // Reconstruct data points from the first polynomial
        uint32_t tStart = 0;
        for (float t = 0; t <= oldest.timeDeltas[i]; t += oldest.timeDeltas[i] / 50.0) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i], t));
        }

        // Adjust second polynomial's timestamps to continue from the end of the first
        tStart += oldest.timeDeltas[i];
        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += oldest.timeDeltas[i+1] / 50.0) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i+1], t));
        }

        // Fit a new polynomial to the combined data
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::NONE);

        // Store the new polynomial in the recompressed segment
        for (uint8_t j = 0; j < newCoefficients.size() && j < 6; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        // Store the combined time delta
        recompressedSegment.timeDeltas[currentPolyIndex] = oldest.timeDeltas[i] + oldest.timeDeltas[i+1];
        //Serial.println(recompressedSegment.timeDeltas[currentPolyIndex]);
        currentPolyIndex++;
    }

// add secondOldest segment
//        currentPolyIndex =0 ; 
        for (uint16_t i = 0; i < POLY_COUNT; i=i+2) {
        // Stop if no more valid time deltas
        if (secondOldest.timeDeltas[i] == 0 || secondOldest.timeDeltas[i+1] == 0) break;
        // Reconstruct data from both polynomials
        std::vector<float> timestamps;
        std::vector<float> values;

        // Reconstruct data points from the first polynomial
        uint32_t tStart = 0;
        for (float t = 0; t <= secondOldest.timeDeltas[i]; t += secondOldest.timeDeltas[i] / 50.0) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i], t));
        }

        // Adjust second polynomial's timestamps to continue from the end of the first
        tStart += secondOldest.timeDeltas[i];
        for (float t = 0; t <= secondOldest.timeDeltas[i+1]; t += secondOldest.timeDeltas[i+1] / 50.0) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i+1], t));
        }

        // Fit a new polynomial to the combined data
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::NONE);

        // Store the new polynomial in the recompressed segment
        for (uint8_t j = 0; j < newCoefficients.size() && j < 6; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        // Store the combined time delta
        recompressedSegment.timeDeltas[currentPolyIndex] = secondOldest.timeDeltas[i] + secondOldest.timeDeltas[i+1];
        currentPolyIndex++;
    }

    
}

void recompressSegments() {
    if (segmentCount < 2) return;

    PolynomialSegment oldest, secondOldest;
    getOldestSegments(oldest, secondOldest);
    
    // Create recompressed segment
    PolynomialSegment recompressedSegment;
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        recompressedSegment.timeDeltas[i] = 0;
    }

    combinePolynomials(oldest, secondOldest, recompressedSegment);
  
    // Add recompressed segment at the correct position (head)
    uint8_t insertPos = head;
    //head = (head + 1) % SEGMENTS;  // Update head to next position
    segmentBuffer[insertPos] = recompressedSegment;
    // Insert recompressed segment


    // Shift existing segments if needed
    for (uint8_t i = insertPos+1 ; i < segmentCount-1; i++) {
        segmentBuffer[i] = segmentBuffer[i+1];
        Serial.println(i);
    }
//only shift for debug
//    for (uint8_t i = insertPos ; i < segmentCount-1; i++) {
//        segmentBuffer[i] = segmentBuffer[i+1];
//        Serial.println(i);
//    }
    
    Serial.print("Recompressed. New segment count: ");
    Serial.println(segmentCount-1);
    Serial.print("poly size: ");
    Serial.print(sizeof(segmentBuffer));
    Serial.print(" raw size: ");
    Serial.println(sizeof(raw_Data)+sizeof(raw_timestamps));


}


// Sample scalar data (simulated random data for now)
float sampleScalarData(uint32_t timestamp) {
    float scalar = random(0, 1000) / 100.0; // background noise
    scalar = scalar + 20 * sin((float)timestamp * 0.0001);

    return scalar; // Random data in range [0, 10.0]
}


void logSampledData(float data, uint32_t currentTimestamp) {
    static float rawData[LOG_BUFFER_POINTS_PER_POLY];
    static uint32_t timestamps[LOG_BUFFER_POINTS_PER_POLY];
    static uint16_t dataIndex = 0;
    static uint16_t currentPolyIndex = 0;

    // Calculate time delta
    uint32_t timeDelta = (currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;

    // Store the data and timestamp
    rawData[dataIndex] = data;
    timestamps[dataIndex] = timeDelta;
    dataIndex++;

    // Check if we have enough data for a polynomial
    if (dataIndex >= LOG_BUFFER_POINTS_PER_POLY) {
        // Initialize first segment if needed
        if (segmentCount == 0) {
            addSegment(PolynomialSegment());
            currentPolyIndex = 0 ; 
            // Initialize new segment's timeDeltas
            for (uint16_t i = 0; i < POLY_COUNT; i++) {
                segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
            }
        }

        // Fit polynomial to current data chunk
        float new_coefficients[6];
        uint32_t new_timeDelta;
        compressDataToSegment(rawData, timestamps, LOG_BUFFER_POINTS_PER_POLY, new_coefficients, new_timeDelta);

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
            currentPolyIndex = 0;
            
            if (segmentCount < SEGMENTS) {
                // Create new segment
                addSegment(PolynomialSegment());
                // Initialize timeDeltas for new segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                }
                Serial.print("Created new segment ");
                Serial.println(segmentCount-1);
            } else {
                // Trigger recompression when buffer is full
                recompressSegments();
                // initalize time deltas for freshly cleared segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                }
            }
        }

        // Reset data buffer
        dataIndex = 0;
    }
}



// Log sampled data into the current segment
void raw_logSampledData(float data, uint32_t currentTimestamp) {
    // Check if the current segment is full
    if (raw_dataIndex >= MAX_RAW_DATA - 1) {
        raw_graphMinY = raw_graphMaxY;
        raw_graphMaxY = 0;
        for (uint16_t i = 0; i < raw_dataIndex; i++) {
            raw_Data[i] = raw_Data[i + 1];
            raw_timestamps[i] = raw_timestamps[i + 1];
            if (raw_Data[i] > raw_graphMaxY) {
                raw_graphMaxY = raw_Data[i];
            }
            if (raw_Data[i] < raw_graphMinY) {
                raw_graphMinY = raw_Data[i];
            }
        }
        raw_Data[raw_dataIndex] = data;
        raw_timestamps[raw_dataIndex] = currentTimestamp;
    } else {
        // Store the data and timestamp
        raw_Data[raw_dataIndex] = data;
        raw_timestamps[raw_dataIndex] = currentTimestamp;
        raw_graphMinY = raw_graphMaxY;
        raw_graphMaxY = 0;
        for (uint16_t i = 0; i < raw_dataIndex; i++) {
            if (raw_Data[i] > raw_graphMaxY) {
                raw_graphMaxY = raw_Data[i];
            }
            if (raw_Data[i] < raw_graphMinY) {
                raw_graphMinY = raw_Data[i];
            }
        }
        raw_dataIndex++;
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
    for (uint16_t i = 0; i < raw_dataIndex; i++) {
        uint16_t y = mapFloat(raw_Data[i], raw_graphMinY, raw_graphMaxY, SCREEN_HEIGHT - 1, 0);
        uint16_t x = mapFloat(raw_timestamps[i], raw_timestamps[0], raw_timestamps[raw_dataIndex - 1], 0, SCREEN_WIDTH);
        tft.drawPixel(x, y, TFT_GREEN);
    }
}

void updateCompressedGraph(const PolynomialSegment *segments, uint8_t count) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;
    //tft.setCursor(10,10);
    //tft.setTextColor(TFT_WHITE);
    //tft,printf("span: %.2f",timeSpan);
    //Serial.println(timeSpan);
    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // First pass: find value range across all valid segments
    uint8_t currentSegment = head;
    for (uint8_t i = 0; i < count; i++) {
        const PolynomialSegment &segment = segments[currentSegment];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            //Serial.println(polyIndex);
            //Serial.println(segment.timeDeltas[polyIndex]);
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            for (uint32_t t = 0; t <= tDelta; t += tDelta/10) {
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
            }
        }
        currentSegment = (currentSegment + 1) % SEGMENTS;
    }

    // Ensure valid min/max values
    if (isinf(minValue) || isinf(maxValue)) {
        minValue = raw_graphMinY;
        maxValue = raw_graphMaxY;
    }

    // Add margin to prevent edge touching
    float valueRange = maxValue - minValue;
    minValue -= valueRange * 0.05;
    maxValue += valueRange * 0.05;

    // Second pass: plot the data
    uint32_t tCurrent = windowStart;
    currentSegment = head;
    for (uint8_t i = 0; i < count; i++) {
        const PolynomialSegment &segment = segments[currentSegment];    
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t numSteps = min(100UL, tDelta);
            uint32_t stepSize = tDelta / numSteps;
            float lastX = -1, lastY = -1;
            
            for (uint32_t t = 0; t <= tDelta; t += stepSize) {
             // Serial.println(t);
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                uint16_t x = mapFloat(tCurrent + t, windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
                uint16_t y = mapFloat(value, minValue, maxValue, SCREEN_HEIGHT - 1, 0);

                x = constrain(x, 0, SCREEN_WIDTH - 1);
                y = constrain(y, 0, SCREEN_HEIGHT - 1);
                
                if (lastX >= 0 && lastY >= 0) {
                    tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
                } else {
                    tft.drawPixel(x, y, TFT_YELLOW);
                }

                lastX = x;
                lastY = y;
            }
            
            tCurrent += tDelta;
        }
        currentSegment = (currentSegment + 1) % SEGMENTS;
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
