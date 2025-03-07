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
#include <algorithm>
#include <cmath>

//for debug
#define MAX_RAW_DATA 2048
//#define MAX_RAW_DATA_GRAIN 32 // after how many points added to the raw data buffer data is recompressed. 
#define LOG_BUFFER_POINTS_PER_POLY 5*10 // replaced by this

static float    raw_Data[MAX_RAW_DATA];
static uint32_t raw_timestamps[MAX_RAW_DATA];
static uint16_t raw_dataIndex = 0; 
float raw_graphMinY = 0;
float raw_graphMaxY = 0;
//static uint16_t dataIndex = 0;
uint32_t raw_log_delta = 0 ; // delta to last logged data, for the offset the compressed graph

 // Min/Max values for Y-axis scaling of the polynomial graph
    float minValue = INFINITY, maxValue = -INFINITY;

#include <stdint.h>

#define POLY_COUNT 16 // Number of polynomials in each segment
#define SEGMENTS 2    // Total number of segments
//const uint8_t POLYS_TO_COMBINE = 2;  // Number of polynomials to combine into one when recompression is triggered


// Storage structure
struct PolynomialSegment {
    float coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), full resolution
    uint32_t timeDeltas[POLY_COUNT];   // 32-bit time deltas
};

// Data buffer
PolynomialSegment segmentBuffer[SEGMENTS];
uint8_t segmentCount = 0;
uint32_t lastTimestamp = 0;
static uint16_t currentPolyIndex = 0;

// Rolling buffer indices
uint8_t head = 0; // Points to the oldest segment
//uint8_t tail = 0; // Points to the newest segment
//uint8_t segmentCount = 0; // Number of valid segments in buffer

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t mapUint(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Add a new segment to the buffer
void addSegment(const PolynomialSegment &newSegment) {
    segmentBuffer[segmentCount] = newSegment;
//    tail = (tail + 1) % SEGMENTS;
//      tail = tail+1; 
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

void compressDataToSegment(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex,  const float *rawData, const uint32_t *timestamps, uint16_t dataSize, float *coefficients, uint32_t &timeDelta) {
    AdvancedPolynomialFitter fitter;

    int8_t segmentIndex = count -1;
    int16_t polyIndex = polyindex; 
    
    #define BOUNDARY_MARGIN3 16 // duplicate data across margin for better fit, multiple of 2
    #define BOUNDARY_DELTA3 100 // time window of margin.
    std::vector<float> x3(dataSize+BOUNDARY_MARGIN3); // +2 front and back datapoints
    std::vector<float> y3(dataSize+BOUNDARY_MARGIN3);

    float timestamp_absolute = 0;
    // Accumulate timestamps and collect data points
//    for (uint8_t i = 0 ; i<BOUNDARY_MARGIN3/2; i++){
    for (uint8_t i = 0 ; i<BOUNDARY_MARGIN3; i++){
    if(currentPolyIndex == 0){
      if (segmentIndex>0){
//        x3[i]= -(((BOUNDARY_MARGIN3/2)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
        x3[i]= -(((BOUNDARY_MARGIN3)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
        uint32_t previous_poly_range = segments[segmentIndex-1].timeDeltas[POLY_COUNT-1];
        y3[i]= evaluatePolynomial(segments[segmentIndex-1].coefficients[POLY_COUNT-1],6,(float)previous_poly_range+x3[i]); // data from previous poly    
      } else {
        //x[i]= -(((BOUNDARY_MARGIN3/2)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
        //y[i]= evaluatePolynomial(coefficients,4,x[i]); // data from fitted 3rd order poly
//        x3[i]= -(((BOUNDARY_MARGIN3/2)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
        x3[i]= -(((BOUNDARY_MARGIN3)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
        y3[i]= rawData[0]; // no better data - duplicate data
      }
    }else{
//    x3[i]= -(((BOUNDARY_MARGIN3/2)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
    x3[i]= -(((BOUNDARY_MARGIN3)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
    uint32_t previous_poly_range = segments[segmentIndex].timeDeltas[polyIndex-1];
    y3[i]= evaluatePolynomial(segments[segmentIndex].coefficients[polyIndex-1],6,(float)previous_poly_range+x3[i]); // data from previous poly
    }

 //   x3[i]= -(((BOUNDARY_MARGIN3/2)-i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
 //   y3[i]= rawData[0]; // duplicate data

    }
    for (uint16_t j = 0; j < dataSize; j++) {
//        x3[j+(BOUNDARY_MARGIN3/2)] = timestamp_absolute;
        x3[j+(BOUNDARY_MARGIN3)] = timestamp_absolute;
//        y3[j+(BOUNDARY_MARGIN3/2)] = rawData[j]; // correct to 0
        y3[j+(BOUNDARY_MARGIN3)] = rawData[j]; // correct to 0
        timestamp_absolute += timestamps[j];
    }

// adding data forward is problematic.

 //  for (uint8_t i = 0 ; i<BOUNDARY_MARGIN3/2; i++){
 //   x3[dataSize+(BOUNDARY_MARGIN3/2)+i]= timestamp_absolute+(((BOUNDARY_MARGIN3/2)*i)*BOUNDARY_DELTA3) ; // add data on the boundary to improve edge fitting
 //   y3[dataSize+(BOUNDARY_MARGIN3/2)+i]= rawData[dataSize-1]; // duplicate data
 //   }

//    x[dataSize+1]=timestamp_absolute+10; // add data on the boundary to improve edge fitting
//    y[dataSize+1]=rawData[dataSize-1]; // duplicate data

    // Fit polynomial to this chunk
//    std::vector<float> fitted_coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT);
//    std::vector<float> fitted_coefficients3 = fitter.fitPolynomial(x, y, 2, AdvancedPolynomialFitter::NONE); // good enough
    std::vector<float> fitted_coefficients3 = fitter.fitPolynomial(x3, y3, 3, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT); // good enough

    // fit with degree of 3 to redefine boundaries
// Store coefficients
    for (uint8_t j = 0; j < fitted_coefficients3.size() && j < 4; j++) {
        coefficients[j] = fitted_coefficients3[j];
    }

    #define BOUNDARY_MARGIN 16 // duplicate data across margin for better fit, multiple of 2
    #define BOUNDARY_DELTA 100 // time window of margin.
    std::vector<float> x(dataSize+BOUNDARY_MARGIN); // +2 front and back datapoints
    std::vector<float> y(dataSize+BOUNDARY_MARGIN);

    timestamp_absolute = 0;
    // Accumulate timestamps and collect data points
    for (uint8_t i = 0 ; i<BOUNDARY_MARGIN/2; i++){
    if(currentPolyIndex == 0){
      if (segmentIndex>0){
        x[i]= -(((BOUNDARY_MARGIN/2)-i)*BOUNDARY_DELTA) ; // add data on the boundary to improve edge fitting
        uint32_t previous_poly_range = segments[segmentIndex-1].timeDeltas[POLY_COUNT-1];
        y[i]= evaluatePolynomial(segments[segmentIndex-1].coefficients[POLY_COUNT-1],6,(float)previous_poly_range+x[i]); // data from previous poly    
      } else {
        x[i]= -(((BOUNDARY_MARGIN/2)-i)*BOUNDARY_DELTA) ; // add data on the boundary to improve edge fitting
        y[i]= evaluatePolynomial(coefficients,4,x[i]); // data from fitted 3rd order poly
      }
    }else{
    x[i]= -(((BOUNDARY_MARGIN/2)-i)*BOUNDARY_DELTA) ; // add data on the boundary to improve edge fitting
    uint32_t previous_poly_range = segments[segmentIndex].timeDeltas[polyIndex-1];
    y[i]= evaluatePolynomial(segments[segmentIndex].coefficients[polyIndex-1],6,(float)previous_poly_range+x[i]); // data from previous poly
    }
    
    }
    for (uint16_t j = 0; j < dataSize; j++) {
        x[j+(BOUNDARY_MARGIN/2)] = timestamp_absolute;
        y[j+(BOUNDARY_MARGIN/2)] = rawData[j]; // correct to 0
        timestamp_absolute += timestamps[j];
    }
   for (uint8_t i = 0 ; i<BOUNDARY_MARGIN/2; i++){
    x[dataSize+(BOUNDARY_MARGIN/2)+i]= timestamp_absolute+(((BOUNDARY_MARGIN/2)*i)*BOUNDARY_DELTA) ; // add data on the boundary to improve edge fitting
    y[dataSize+(BOUNDARY_MARGIN/2)+i]= evaluatePolynomial(coefficients,4,x[dataSize+(BOUNDARY_MARGIN/2)+i]); // data from fitted 3rd order poly
    }

//    std::vector<float> fitted_coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::NONE); // good enough
    std::vector<float> fitted_coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT); // good enough

    // Store coefficients
    for (uint8_t j = 0; j < fitted_coefficients.size() && j < 6; j++) {
        coefficients[j] = fitted_coefficients[j];
    }

    // Store the time delta
    timeDelta = timestamp_absolute;
}


/*
// Evaluate a polynomial at a given timestamp
float evaluatePolynomial(const float *coefficients, float tNormalized) {
    float result = 0.0;
    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * pow(tNormalized, i);
    }
    return result;
}
*/

 // on some systems this is faster
float evaluatePolynomial(const float *coefficients, uint8_t degree, float t) {
    // t is already in milliseconds within the segment's time delta range
    float result = 0.0;
    float tPower = 1.0;  // t^0 = 1
    
    for (int i = 0; i < degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= t;  // More efficient than pow()
    }
    return result;
}

void combinePolynomials(const PolynomialSegment &oldest, const PolynomialSegment &secondOldest, PolynomialSegment &recompressedSegment) {
    AdvancedPolynomialFitter fitter;

//#define RECOMPRESS_RESOLUTION  10 // memory limit
    #define MEMORY_LIMIT 1000 // 1000*4*2 =8K *2 16K
    uint16_t currentPolyIndex = 0;

 // (oldest.timeDeltas[i] + oldest.timeDeltas[i+1]) // total step count
 //   float RECOMPRESS_RESOLUTION =  (oldest.timeDeltas[i] + oldest.timeDeltas[i+1])/MEMORY_LIMIT;

    for (uint16_t i = 0; i < POLY_COUNT; i=i+2) {
        // Stop if no more valid time deltas
        if (oldest.timeDeltas[i] == 0 || oldest.timeDeltas[i+1] == 0) break;
        // Reconstruct data from both polynomials
        std::vector<float> timestamps;
        std::vector<float> values;
        float RECOMPRESS_RESOLUTION =  (oldest.timeDeltas[i] + oldest.timeDeltas[i+1])/MEMORY_LIMIT;
        Serial.print("resolution:");
        Serial.println(RECOMPRESS_RESOLUTION);
        // Reconstruct data points from the first polynomial
        uint32_t tStart = 0;
 //       for (float t = 0; t <= oldest.timeDeltas[i]; t += oldest.timeDeltas[i] / 50.0) {
        for (float t = 0; t <= oldest.timeDeltas[i]; t += RECOMPRESS_RESOLUTION) {

            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i],6, t));
        }

        // Adjust second polynomial's timestamps to continue from the end of the first
        tStart += oldest.timeDeltas[i];
//        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += oldest.timeDeltas[i+1] / 50.0) {
        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {

            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i+1],6, t));
        }

        // Fit a new polynomial to the combined data
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::NONE);
//        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT);


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
        float RECOMPRESS_RESOLUTION =  (secondOldest.timeDeltas[i] + secondOldest.timeDeltas[i+1])/MEMORY_LIMIT;

        // Reconstruct data points from the first polynomial
        uint32_t tStart = 0;
        for (float t = 0; t <= secondOldest.timeDeltas[i]; t += secondOldest.timeDeltas[i] / 50.0) {
//        for (float t = 0; t <= secondOldest.timeDeltas[i]; t += RECOMPRESS_RESOLUTION) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i],6, t));
        }

        // Adjust second polynomial's timestamps to continue from the end of the first
        tStart += secondOldest.timeDeltas[i];
//        for (float t = 0; t <= secondOldest.timeDeltas[i+1]; t += secondOldest.timeDeltas[i+1] / 50.0) {
        for (float t = 0; t <= secondOldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i+1],6, t));
        }

        // Fit a new polynomial to the combined data
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::NONE);
//      std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::LEVENBERG_MARQUARDT);

        // Store the new polynomial in the recompressed segment
        for (uint8_t j = 0; j < newCoefficients.size() && j < 6; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        // Store the combined time delta
        recompressedSegment.timeDeltas[currentPolyIndex] = secondOldest.timeDeltas[i] + secondOldest.timeDeltas[i+1];
        currentPolyIndex++;
    }    
//               minValue = 0;
//               maxValue = 0;

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
    float scalar = random(0, 1000*sin((float)timestamp * 0.00002)) / 100.0; // background noise
    scalar = scalar + 10 * sin((float)timestamp * 0.0001)+20*sin((float)timestamp * 0.00001);

    return scalar; // Random data
}

void logSampledData(float data, uint32_t currentTimestamp) {
    static float rawData[LOG_BUFFER_POINTS_PER_POLY];
    static uint32_t timestamps[LOG_BUFFER_POINTS_PER_POLY];
    static uint16_t dataIndex = 0;
//    static uint16_t currentPolyIndex = 0; made global to make stuff easier

    // Calculate time delta
    uint32_t timeDelta = (currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;

    // Store the data and timestamp
    rawData[dataIndex] = data;
    timestamps[dataIndex] = timeDelta;
    dataIndex++;

    raw_log_delta += timeDelta;
  
   if (dataIndex >= LOG_BUFFER_POINTS_PER_POLY) {

         // Initialize first segment if needed
        if (segmentCount == 0) {
           addSegment(PolynomialSegment());
            currentPolyIndex = 0 ;  
            // Initialize new segment's timeDeltas
            for (uint16_t i = 0; i < POLY_COUNT; i++) {
                segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
            }
            segmentBuffer[segmentCount-1].timeDeltas[currentPolyIndex]=1; // initalize first poly. it acts as extra storage for oldest data.       
        } 
     currentPolyIndex++;

        // If current segment is full, prepare for next segment
        if (currentPolyIndex >= POLY_COUNT) {
            
            if (segmentCount < SEGMENTS) {
                // Create new segment
                addSegment(PolynomialSegment());
                // Initialize timeDeltas for new segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][0] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][1] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][2] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][3] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][4] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][5] = 0;
                }
                currentPolyIndex = 0;
                Serial.print("Created new segment ");
                Serial.println(segmentCount-1);
            } else {
                // Trigger recompression when buffer is full
                recompressSegments();
                // initalize time deltas for freshly cleared segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][0] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][1] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][2] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][3] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][4] = 0;
                    segmentBuffer[segmentCount-1].coefficients[i][5] = 0;
                }
                currentPolyIndex = 0;
 
            }
        }

        // Fit polynomial to current data chunk
        float new_coefficients[6];
        uint32_t new_timeDelta;
        compressDataToSegment(segmentBuffer,segmentCount,currentPolyIndex,rawData, timestamps, dataIndex, new_coefficients, new_timeDelta);

//        compressDataToSegment(rawData, timestamps, LOG_BUFFER_POINTS_PER_POLY-2, new_coefficients, new_timeDelta);

        // Store the polynomial in current segment
        for (uint8_t i = 0; i < 6; i++) {
            segmentBuffer[segmentCount-1].coefficients[currentPolyIndex][i] = new_coefficients[i];
        }
        segmentBuffer[segmentCount-1].timeDeltas[currentPolyIndex] = new_timeDelta;

        raw_log_delta = 0; 
        Serial.print("Added polynomial ");
        Serial.print(currentPolyIndex);
        Serial.print(" to segment ");
        Serial.println(segmentCount-1);

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

//        // Initialize first segment if needed
//        if (segmentCount == 0) {
//           addSegment(PolynomialSegment());
//            currentPolyIndex = 0 ;  
//            // Initialize new segment's timeDeltas
//            for (uint16_t i = 0; i < POLY_COUNT; i++) {
//                segmentBuffer[segmentCount-1].timeDeltas[i] = 0;
//            }
//        }

    // Draw static labels
//    tft.drawString("Raw Data", 10, 5, 2);
//    tft.drawString("Compressed Data", 10, COMPRESSED_GRAPH_Y - 15, 2);
}

void drawRawGraph() {
    // Time window for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex -1] ;

      uint32_t xMin = windowStart, xMax = windowEnd - raw_log_delta;    
    uint16_t SWdelta = mapFloat(raw_log_delta+xMin, xMin, windowEnd, 0, SCREEN_WIDTH-1);
    uint16_t Swidth = SCREEN_WIDTH-SWdelta;
    uint32_t lastDataX = xMax;
    
    if(SWdelta){tft.fillRect(SCREEN_WIDTH-SWdelta,0,SWdelta,SCREEN_HEIGHT,0x0821);}
   // if(SWdelta){tft.drawRect(SCREEN_WIDTH-1-SWdelta,0,SWdelta,SCREEN_HEIGHT-1,TFT_RED);}   
    
    // Draw the new point
    for (uint16_t i = 0; i < raw_dataIndex; i++) {
        uint16_t y = mapFloat(raw_Data[i], raw_graphMinY, raw_graphMaxY, SCREEN_HEIGHT - 1, 0);
        uint16_t x = mapFloat(raw_timestamps[i], raw_timestamps[0], raw_timestamps[raw_dataIndex - 1], 0, SCREEN_WIDTH);
        tft.drawPixel(x, y, TFT_BLUE);
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
                float value = evaluatePolynomial(segment.coefficients[polyIndex],6, t);
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
                float value = evaluatePolynomial(segment.coefficients[polyIndex],6, t);
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

void updateCompressedGraphBackwards(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // Calculate total valid time span in polynomial log
  //  uint32_t totalPolyTime = 0;
    uint8_t lastValidSegment = head;
    uint16_t lastValidPoly = 0;
    
/*
    // Find the last valid segment and polynomial
    for (uint8_t i = 0; i < count; i++) {
        uint8_t segmentIndex = (head + i) % SEGMENTS;
        const PolynomialSegment &segment = segments[segmentIndex];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            totalPolyTime += segment.timeDeltas[polyIndex];
            lastValidSegment = segmentIndex;
            lastValidPoly = polyIndex;
        }
    }
*/ // no need as we pass last valid segment as parameter

    lastValidSegment = count-1;
    lastValidPoly = polyindex;

    // First pass: find value range across all valid segments within time window
    uint32_t tCurrent = windowEnd;
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t segmentIndex = (lastValidSegment - i + SEGMENTS) % SEGMENTS;
        const PolynomialSegment &segment = segments[segmentIndex];
        
        for (int16_t polyIndex = (i == 0 ? lastValidPoly : POLY_COUNT - 1); polyIndex >= 0; polyIndex--) {
            if (segment.timeDeltas[polyIndex] == 0) continue;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t numSteps = min(100UL, tDelta);
            int32_t stepSize = tDelta / numSteps;
            int32_t t = tDelta;  // Changed to signed
            
            while (t > 0) {  // Changed to while loop
                uint32_t actualTime = tCurrent - (tDelta - t);
                // Only consider values within the time window
                if (actualTime >= windowStart && actualTime <= windowEnd) {
                    float value = evaluatePolynomial(segment.coefficients[polyIndex],6, t);
                    minValue = min(minValue, value);
                    maxValue = max(maxValue, value);
                }
                t -= stepSize;
            }
            
            tCurrent -= tDelta;
            // If we've gone before the window start, we can stop
            if (tCurrent < windowStart) break;
        }
        if (tCurrent < windowStart) break;
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

    // Second pass: plot the data backwards
    tCurrent = windowEnd;
    float lastX = -1, lastY = -1;
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t segmentIndex = (lastValidSegment - i + SEGMENTS) % SEGMENTS;
        const PolynomialSegment &segment = segments[segmentIndex];
        
        for (int16_t polyIndex = (i == 0 ? lastValidPoly : POLY_COUNT - 1); polyIndex >= 0; polyIndex--) {
            if (segment.timeDeltas[polyIndex] == 0) continue;
            
      //      uint64_t tDelta = segment.timeDeltas[polyIndex];
       //     uint32_t numSteps = min(100UL, tDelta);

           
            float stepSize = segment.timeDeltas[polyIndex] / SCREEN_WIDTH-1;
            double t = segment.timeDeltas[polyIndex];  // Changed to signed
            
            while (t > 0) {  // Changed to while loop
//uint64_t actualTime = tCurrent - (tDelta - t);
                  uint64_t actualTime = tCurrent - (segment.timeDeltas[polyIndex] - t);

                // Only plot points within the time window
                if (actualTime >= windowStart && actualTime <= windowEnd) {
                    float value = evaluatePolynomial(segment.coefficients[polyIndex],6, t);
                    uint16_t x = mapFloat(actualTime, windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
                    uint16_t y = mapFloat(value, minValue, maxValue, SCREEN_HEIGHT - 1, 0);

                    x = constrain(x, 0, SCREEN_WIDTH - 1);
                    y = constrain(y, 0, SCREEN_HEIGHT - 1);
                    

//                    if (lastX >= 0 && lastY >= 0) {
//                       tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
//                    } else {
//                        tft.drawPixel(x, y, TFT_YELLOW);
//                    }
                        tft.drawPixel(x, y, TFT_YELLOW);


                    lastX = x;
                    lastY = y;
                }
                t -= stepSize;
            }
            
            tCurrent -= segment.timeDeltas[polyIndex];
            // If we've gone before the window start, we can stop
            if (tCurrent < windowStart) break;
        }
        if (tCurrent < windowStart) break;
    }
}

void updateCompressedGraphBackwardsFast(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // Calculate total valid time span in polynomial log
  //  uint32_t totalPolyTime = 0;
    uint8_t lastValidSegment = head;
    uint16_t lastValidPoly = 0;
    
/*
    // Find the last valid segment and polynomial
    for (uint8_t i = 0; i < count; i++) {
        uint8_t segmentIndex = (head + i) % SEGMENTS;
        const PolynomialSegment &segment = segments[segmentIndex];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            totalPolyTime += segment.timeDeltas[polyIndex];
            lastValidSegment = segmentIndex;
            lastValidPoly = polyIndex;
        }
    }
*/ // no need as we pass last valid segment as parameter

    lastValidSegment = count-1;
    lastValidPoly = polyindex;

    // First pass: find value range across all valid segments within time window
    uint32_t tCurrent = windowEnd;
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t segmentIndex = (lastValidSegment - i + SEGMENTS) % SEGMENTS;
        const PolynomialSegment &segment = segments[segmentIndex];
        
        for (int16_t polyIndex = (i == 0 ? lastValidPoly : POLY_COUNT - 1); polyIndex >= 0; polyIndex--) {
            if (segment.timeDeltas[polyIndex] == 0) continue;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t numSteps = min(100UL, tDelta);
            int32_t stepSize = tDelta / numSteps;
            int32_t t = tDelta;  // Changed to signed
            
            while (t > 0) {  // Changed to while loop
                uint32_t actualTime = tCurrent - (tDelta - t);
                // Only consider values within the time window
                if (actualTime >= windowStart && actualTime <= windowEnd) {
                    float value = evaluatePolynomial(segment.coefficients[polyIndex],6, t);
                    minValue = min(minValue, value);
                    maxValue = max(maxValue, value);
                }
                t -= stepSize;
            }
            
            tCurrent -= tDelta;
            // If we've gone before the window start, we can stop
            if (tCurrent < windowStart) break;
        }
        if (tCurrent < windowStart) break;
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

// faster plot
      uint32_t xMin = windowStart;
      uint32_t xMax = windowEnd;
      uint32_t lastDataX = windowEnd;  
      int16_t segmentIndex = lastValidSegment;
      int16_t polyIndex = lastValidPoly;
      uint16_t lastY = 0 ; 
        for (int x = SCREEN_WIDTH; x > 0; --x) {
  //          uint32_t dataX = mapUint(x, 0,  SCREEN_WIDTH - 1, xMin, xMax);          
           uint32_t dataX = mapUint(x, 0,  SCREEN_WIDTH - 1, xMin, xMax);          
           //uint32_t dataX = mapUint64(x, 0,  SCREEN_WIDTH - 1, xMin, xMax);          
  
            const PolynomialSegment &segment = segments[segmentIndex];
            uint32_t xDelta = segment.timeDeltas[polyIndex];
            uint32_t curDelta = lastDataX-dataX;
           if (curDelta >= xDelta) {
                tft.drawLine(x,0,x,SCREEN_HEIGHT,TFT_DARKGREEN); // plot segment boundaries 
                lastDataX = dataX; // set the segment boundary tracker
              if (polyIndex-- <= 0) {
                polyIndex = POLY_COUNT-1;
                tft.drawLine(x,0,x,SCREEN_HEIGHT,TFT_RED); // plot segment boundaries 
                  if (segmentIndex-- < 0) {
                    break; // stop drawing
                  }
              }
            }
          // if not continue drawing

            float yFitted = 0.0;
            for (uint8_t j = 0; j < 6; ++j) {
               const PolynomialSegment &segment = segments[segmentIndex];
               uint32_t in_delta = xDelta-(lastDataX-dataX);
               yFitted += segment.coefficients[polyIndex][j] * pow(in_delta, j);
            }
            uint16_t y = 0; 
            if (yFitted != NAN ) {
               y = mapFloat(yFitted, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
              if (y>0 && y< SCREEN_HEIGHT) {
                if (lastY>=0){
                  tft.drawLine(x,y,x+1, lastY, TFT_CYAN);
                } else {
                  tft.drawPixel(x, y, TFT_CYAN);
                } 
                
              }
            }
            lastY = y;  

        }

}

void updateMinMax(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex,uint32_t windowStart, uint32_t windowEnd, bool clear_under, bool draw_lines) {
    // Initialize tracking indices
    int16_t segmentIndex = count - 1;
    int16_t polyIndex = polyindex;
    // First pass: Calculate min/max values
    uint32_t tCurrent = windowEnd;
    minValue = INFINITY;
    maxValue = -INFINITY;
 
    for (int16_t i = 0; i < count; ++i) {
        const PolynomialSegment &segment = segments[segmentIndex];
        for (int16_t j = (i == 0 ? polyIndex : POLY_COUNT - 1); j >= 0; --j) {
            uint32_t tDelta = segment.timeDeltas[j];
            if (tDelta == 0) continue;

            uint32_t numSteps = min(100UL, tDelta);
            uint32_t stepSize = tDelta / numSteps;

            for (uint32_t t = stepSize; t <= tDelta; t += stepSize) {
                uint32_t actualTime = tCurrent - t;
                if (actualTime < windowStart || actualTime > windowEnd) break;

                float value = evaluatePolynomial(segment.coefficients[j],6, t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
            }
            tCurrent -= tDelta;
            if (tCurrent < windowStart) break;
        }
        if (--segmentIndex < 0) break;
    }
 // use min/max value from previous graph pass 
 

    if (isinf(minValue) || isinf(maxValue)) {
        minValue = raw_graphMinY;
        maxValue = raw_graphMaxY;
    }

    // Add margin for aesthetics
 //   float valueRange = maxValue - minValue;
 //   minValue -= valueRange * 0.05f;
 //   maxValue += valueRange * 0.05f;
    
}

void updateCompressedGraphBackwardsFastOpt(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex,bool clear_under, bool draw_lines) {
    if (count == 0) return;

    // Time window for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex -1] ;


    // Min/Max values for Y-axis scaling
    float new_minValue = INFINITY, new_maxValue = -INFINITY;
    
    // Plotting the compressed graph
    uint32_t xMin = windowStart, xMax = windowEnd - raw_log_delta;
//    uint32_t lastDataX = xMax;

int16_t    segmentIndex = count - 1;
int16_t    polyIndex = polyindex;
//    segmentIndex = count - 1;
//    polyIndex = polyindex;

 
    uint32_t lastDataX = xMax;
    uint16_t SWdelta = mapFloat(raw_log_delta+windowStart,windowStart, windowEnd, 0, SCREEN_WIDTH);
    uint16_t Swidth = SCREEN_WIDTH-SWdelta-1;
 
//tft.setCursor(5,40);
//tft.setTextColor(TFT_WHITE);
//tft.setTextSize(2);
//tft.print("SW:");
//tft.print(Swidth);
    if(SWdelta){tft.drawRect(SCREEN_WIDTH-SWdelta,0,SWdelta,SCREEN_HEIGHT-1,TFT_RED);}   
//    if(SWdelta){tft.fillRect(SCREEN_WIDTH-1-SWdelta,0,SWdelta,SCREEN_HEIGHT-1,TFT_DARKGREY);}
   
    int16_t lastY = -1 ;
    for (int x = Swidth; x >= 0; --x) {
        float dataX = mapFloat(x, +0.0, Swidth, xMin, xMax);  
       // const PolynomialSegment &segment = segments[segmentIndex];
        float tDelta = segments[segmentIndex].timeDeltas[polyIndex]-(lastDataX - dataX);   
        if(clear_under){tft.drawFastVLine(x, 0, SCREEN_HEIGHT, TFT_BLACK);}
        
        while (segmentIndex >= 0 && ((lastDataX - dataX) >= segments[segmentIndex].timeDeltas[polyIndex])) {
            tft.drawFastVLine(x, 0, SCREEN_HEIGHT, 0x0821 );
            lastDataX -= segments[segmentIndex].timeDeltas[polyIndex];
            if (--polyIndex < 0) {
                polyIndex = POLY_COUNT - 1;
                if (--segmentIndex < 0) break;
               tft.drawFastVLine(x, 0, SCREEN_HEIGHT, TFT_RED);
            }

            
    //    Serial.println("boundary");
    //    Serial.print(x);Serial.print(":");Serial.print(tDelta);Serial.print("mapped:");Serial.print(lastDataX-dataX);Serial.print("dSeg:");Serial.println(segment.timeDeltas[polyIndex]);
        tDelta = segments[segmentIndex].timeDeltas[polyIndex]-(lastDataX - dataX);   // recompute if changed polynomial or segment
    //    Serial.print(x);Serial.print(":");Serial.print(tDelta);Serial.print("mapped:");Serial.print(lastDataX-dataX);Serial.print("dSeg:");Serial.println(segment.timeDeltas[polyIndex]);
 
        }
         // segments[segmentIndex];
//        tDelta = segment.timeDeltas[polyIndex]-(lastDataX - dataX);   
        
        // Compute the fitted Y value
//        const PolynomialSegment &segment = segments[segmentIndex];
 //       float tDelta = segment.timeDeltas[polyIndex]-(lastDataX - dataX);

        float yFitted = 0.0f;
       // Serial.print(x);Serial.print(":");Serial.print(tDelta);Serial.print("mapped:");Serial.print(lastDataX-dataX);Serial.print("dSeg:");Serial.println(segment.timeDeltas[polyIndex]);
/*
        for (uint8_t j = 0; j < 6; ++j) {
            yFitted += segment.coefficients[polyIndex][j] * powf(tDelta, j);
        }
*/ //on some systems this is faster

    float tPower = 1.0;  // t^0 = 1  
    for (uint8_t i = 0; i < 6; i++) {
        yFitted += segments[segmentIndex].coefficients[polyIndex][i] * tPower;
        tPower *= tDelta;  // More efficient than pow()
    }
                // update the new min-max values
                new_minValue = min(new_minValue, yFitted);
                new_maxValue = max(new_maxValue, yFitted);

        uint16_t y =0;
        if (!isnan(yFitted)) {
            y = mapFloat(yFitted, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
            if (y < SCREEN_HEIGHT) {
                if(draw_lines&&lastY>0){
                tft.drawLine(x,y, x+1,lastY,TFT_YELLOW);  
                }else{
                tft.drawPixel(x, y, TFT_WHITE);  
                }                
            }
        }
        lastY = y;
    }
    minValue=new_minValue;
    maxValue=new_maxValue;
}
//==================================================================

void loop() {
    // Simulate sampling at random intervals
    delay(random(1, 200)); // Random delay between 50 ms to 500 ms
    uint32_t currentTimestamp = millis();

    // Sample scalar data
    float sampledData = sampleScalarData(currentTimestamp);

    // Log the sampled data
    logSampledData(sampledData, currentTimestamp);

    // Log in the raw form (for debug purposes)
    raw_logSampledData(sampledData, currentTimestamp);

     // tft.fillScreen(TFT_BLACK);
     uint32_t windowStart = raw_timestamps[0];
     uint32_t windowEnd = raw_timestamps[raw_dataIndex -1] ;
    // updateMinMax(segmentBuffer, segmentCount,currentPolyIndex,windowStart,windowEnd,false,true);
     // Update the raw data graph
     drawRawGraph();
     
    // Update the compressed data graph
//     updateCompressedGraphBackwards(segmentBuffer, segmentCount,currentPolyIndex);
//     updateCompressedGraphBackwardsFast(segmentBuffer, segmentCount,currentPolyIndex);
     updateCompressedGraphBackwardsFastOpt(segmentBuffer, segmentCount,currentPolyIndex,true,true); 
                                          // buffer_segment[n].coeffs[degree],segment_count,poly_index_in_last_seg, if_clear_under, if_draw_lines 
     drawRawGraph();
     updateCompressedGraphBackwardsFastOpt(segmentBuffer, segmentCount,currentPolyIndex,false,true);

//tft.setCursor(5,20);
//tft.setTextColor(TFT_WHITE);
//tft.setTextSize(2);
//tft.print("d:");
//tft.print(raw_log_delta);
//     updateCompressedGraphBackwardsFast(segmentBuffer, segmentCount,currentPolyIndex);

     //updateCompressedGraph(segmentBuffer, segmentCount);

  // Update the raw data graph
  //  drawRawGraph();

 
}
