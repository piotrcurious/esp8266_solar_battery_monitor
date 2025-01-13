#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(); // Create TFT instance
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_HEIGHT 240
#define RAW_GRAPH_Y 0
#define COMPRESSED_GRAPH_Y 0

#include "AdvancedPolynomialFitter.hpp" // Include the advanced fitter
#include <vector>


//for debug
#define MAX_RAW_DATA 1024
static float    raw_Data[MAX_RAW_DATA];
static uint32_t raw_timestamps[MAX_RAW_DATA];
float raw_graphMinY = 0;
float raw_graphMaxY = 0;
static uint16_t dataIndex = 0;

//uint16_t compressedGraphBuffer[SCREEN_WIDTH]; // Scrolling buffer for compressed data visualization
//uint16_t graphIndex = 0; // Tracks the current position in the graph buffer

#include <stdint.h>

#define POLY_COUNT 64 // Number of polynomials in each segment
#define SEGMENTS 8    // Total number of segments
#define MAX_SEGMENTS 8    // Total number of segments again , so it is more confusing 


// Storage structure
struct PolynomialSegment {
//    int8_t coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), quantized to 8-bit
    float coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), full resolution
    uint16_t timeDeltas[POLY_COUNT];   // 16-bit time deltas
};

// Data buffer
PolynomialSegment segmentBuffer[SEGMENTS];
uint8_t segmentCount = 0 ;
uint32_t lastTimestamp = 0 ; 



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


void compressDataToSegment(const float *rawData, const uint16_t *timestamps, uint16_t dataSize, PolynomialSegment &segment) {
    AdvancedPolynomialFitter fitter;
    uint16_t segmentIndex = 0;

    for (uint16_t i = 0; i < dataSize; i += POLY_COUNT) {
        uint16_t chunkSize = (i + POLY_COUNT < dataSize) ? POLY_COUNT : (dataSize - i);

        std::vector<float> x(chunkSize);
        std::vector<float> y(chunkSize);

        float timestamp_absolute = 0 ; 
        for (uint16_t j = 0; j < chunkSize; j++) {

 //           x[j] = timestamps[i + j];
            x[j] = timestamp_absolute;
            y[j] = rawData[i + j];
            timestamp_absolute=timestamp_absolute+timestamps[i+j];    // must derelativise deltas.        
        }


        std::vector<float> coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::NONE);

        for (uint8_t j = 0; j < coefficients.size(); j++) {
            segment.coefficients[segmentIndex][j] = coefficients[j];
        }

        //segment.timeDeltas[segmentIndex] = timestamps[i + chunkSize - 1] - timestamps[i];
        //Serial.println(segment.timeDeltas[segmentIndex]);
        segment.timeDeltas[segmentIndex] = x[i + chunkSize -1];
        Serial.println(x[i+chunkSize-1]);
        segmentIndex++;
        Serial.println(segmentCount);
        if (segmentIndex >= POLY_COUNT) break;
    }
}

// Evaluate a polynomial at a given timestamp
//float evaluatePolynomial(const int8_t *coefficients, float tNormalized) {
float evaluatePolynomial(const float *coefficients, float tNormalized) {

    float tPower = 1.0;
    float result = 0.0;

 //   for (int i = 0; i < 6; i++) {
 //       result += coefficients[i] * tPower / 256.0; // Scale back from quantized 8-bit format
 //       tPower *= tNormalized;
 //   }
    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * pow(tNormalized,i); // Scale back from quantized 8-bit format
    }
 
    return result;
}

// Combine two segments by evaluating their polynomials at normalized timestamps
void generateCombinedData(const PolynomialSegment &seg1, const PolynomialSegment &seg2,
                          float *combinedData, uint16_t *combinedTimestamps, uint16_t &combinedSize) {
    combinedSize = 0;

    // Step 1: Evaluate and append data from the first segment
    uint16_t tBase = 0;
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        if (seg1.timeDeltas[i] == 0) break; // End of valid data
        uint16_t tStart = tBase;
        uint16_t tEnd = tBase + seg1.timeDeltas[i];
        tBase = tEnd;

        // Sample data at regular intervals within this segment
        for (uint16_t j = 0; j <= 8; j++) { // Sample 8 points per polynomial
            float tNormalized = (float)j / 8.0; // Normalize within [0, 1]
            uint16_t tActual = tStart + (uint16_t)(tNormalized * (tEnd - tStart));
            combinedTimestamps[combinedSize] = tActual;
            combinedData[combinedSize] = evaluatePolynomial(seg1.coefficients[i], tNormalized);
            combinedSize++;
        }
    }

    // Step 2: Evaluate and append data from the second segment
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        if (seg2.timeDeltas[i] == 0) break; // End of valid data
        uint16_t tStart = tBase;
        uint16_t tEnd = tBase + seg2.timeDeltas[i];
        tBase = tEnd;

        // Sample data at regular intervals within this segment
        for (uint16_t j = 0; j <= 8; j++) { // Sample 8 points per polynomial
            float tNormalized = (float)j / 8.0; // Normalize within [0, 1]
            uint16_t tActual = tStart + (uint16_t)(tNormalized * (tEnd - tStart));
            combinedTimestamps[combinedSize] = tActual;
            combinedData[combinedSize] = evaluatePolynomial(seg2.coefficients[i], tNormalized);
            combinedSize++;
        }
    }
}

// Stack two segments and fit new polynomials to the combined data
void stackPolynomials(const PolynomialSegment &seg1, const PolynomialSegment &seg2, PolynomialSegment &result) {
    float combinedData[POLY_COUNT * 2 * 9];  // 9 samples per polynomial
    uint16_t combinedTimestamps[POLY_COUNT * 2 * 9];
    uint16_t combinedSize = 0;

    // Generate combined timestamp/value data
    generateCombinedData(seg1, seg2, combinedData, combinedTimestamps, combinedSize);

    // Compress the combined data into a new segment
    compressDataToSegment(combinedData, combinedTimestamps, combinedSize, result);
}


void recompressSegments() {
    if (count < 2) {
        // Not enough segments to recompress
        return;
    }

    PolynomialSegment oldest, secondOldest, recompressedSegment;

    // Retrieve the oldest and second-oldest segments
    getOldestSegments(oldest, secondOldest);

    // Stack and recompress the two segments
    stackPolynomials(oldest, secondOldest, recompressedSegment);

    // Remove the two oldest segments
    removeOldestTwo();

    // Add the recompressed segment back into the buffer
    addSegment(recompressedSegment);
}

  
// Sample scalar data (simulated random data for now)
float sampleScalarData(uint32_t timestamp) {
    float scalar = random(0,1000)/100.0; // background noise
    scalar = scalar + 20* sin ((float)timestamp*0.0001);
    
    return (float)scalar; // Random data in range [0, 10.0]
}

// Log sampled data into the current segment
void logSampledData(float data, uint32_t currentTimestamp) {
    static float rawData[POLY_COUNT];
    static uint16_t timestamps[POLY_COUNT];
    static uint16_t dataIndex = 0;

    // Calculate time delta
    uint16_t timeDelta = (uint16_t)(currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;

    // Store the data and timestamp
    rawData[dataIndex] = data;
    timestamps[dataIndex] = timeDelta;
    dataIndex++;

    // Check if the current segment is full
    if (dataIndex >= POLY_COUNT) {
        // Create a new segment from the sampled data
        PolynomialSegment newSegment;
        compressDataToSegment(rawData, timestamps, dataIndex, newSegment);

        // Add the new segment to the buffer
        if (segmentCount < MAX_SEGMENTS) {
            segmentBuffer[segmentCount++] = newSegment;
        } else {
            // Recompress the buffer if full
            recompressSegments();
            segmentBuffer[segmentCount - 1] = newSegment; // Replace the oldest after recompression
        }

        // Reset the data index for the next segment
        dataIndex = 0;
    }
}

// Log sampled data into the current segment
void raw_logSampledData(float data, uint32_t currentTimestamp) {

    // Check if the current segment is full
    if (dataIndex >= MAX_RAW_DATA-1) {

    raw_graphMinY = raw_graphMaxY ; 
    raw_graphMaxY = 0 ; 
    for (uint16_t i = 0 ; i< dataIndex ; i++){
      raw_Data[i] = raw_Data[i+1];
      raw_timestamps[i] = raw_timestamps[i+1];
      if (raw_Data[i]> raw_graphMaxY) {
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
          raw_graphMinY = raw_graphMaxY ; 
          raw_graphMaxY = 0 ; 
            for (uint16_t i = 0 ; i< dataIndex ; i++){
            if (raw_Data[i]> raw_graphMaxY) {
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
//    tft.fillRect(SCREEN_WIDTH-MAX_RAW_DATA, 0, SCREEN_WIDTH-MAX_RAW_DATA-1, SCREEN_HEIGHT-1, TFT_BLACK);
    tft.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT-1, TFT_RED);
    
    // Draw the new point
    for (uint16_t i = 0; i < dataIndex; i++ ){
          uint16_t y = mapFloat(raw_Data[i], raw_graphMinY, raw_graphMaxY, SCREEN_HEIGHT-1, 0);
          uint16_t x = mapFloat(raw_timestamps[i], raw_timestamps[0], raw_timestamps[dataIndex-1], 0, SCREEN_WIDTH);
          tft.drawPixel(x,y,TFT_GREEN);
    }
    
    //tft.drawPixel(graphIndex, RAW_GRAPH_Y + graphValue, TFT_GREEN);

}


void updateCompressedGraph(const PolynomialSegment *segments, uint8_t segmentCount) {
    if (segmentCount == 0) {
        // No segments to visualize
        return;
    }
//    Serial.println(segmentCount);
    // Clear the previous compressed graph
    //tft.fillRect(0, COMPRESSED_GRAPH_Y, SCREEN_WIDTH, GRAPH_HEIGHT, TFT_BLACK);

    // Initialize graphing parameters
    uint32_t tCurrent = 0; // Absolute time tracker for continuity
    uint16_t x = 0;        // Screen x-coordinate for plotting

    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];

        // Process each polynomial in the segment
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            uint16_t tDelta = segment.timeDeltas[polyIndex];
            Serial.println(polyIndex);
            Serial.println(tDelta);
            
            if (tDelta == 0) {
                // End of valid data in this segment
                break;
            }

            // Define the start and end times of the polynomial
            uint32_t tStart = tCurrent;
            uint32_t tEnd = tStart + tDelta;

            // Plot the polynomial across its duration
            for (float tNorm = 0; tNorm <= 1.0 && x < SCREEN_WIDTH; tNorm += 1.0 / (tDelta * 4)) {
                uint32_t tAbsolute = tStart + (uint32_t)(tNorm * (tEnd - tStart));
//                Serial.println(tNorm);
                float reconstructedValue = evaluatePolynomial(segment.coefficients[polyIndex], tNorm);

                // Scale the data to fit the graph
                int graphValue = GRAPH_HEIGHT - (int)(reconstructedValue * (GRAPH_HEIGHT / 10.0));
                if (graphValue < 0) graphValue = 0;
                if (graphValue >= GRAPH_HEIGHT) graphValue = GRAPH_HEIGHT - 1;

                // Draw the point on the graph
                tft.drawPixel(x, COMPRESSED_GRAPH_Y + graphValue, TFT_YELLOW);
                x++;
            }

            // Update the current time
            tCurrent = tEnd;
        }
    }
}


void loop() {
    // Simulate sampling at random intervals
    delay(random(100, 200)); // Random delay between 50 ms to 500 ms
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
