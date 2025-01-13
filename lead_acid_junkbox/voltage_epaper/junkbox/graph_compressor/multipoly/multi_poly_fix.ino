#include <TFT_eSPI.h>
#include "AdvancedPolynomialFitter.hpp" // Include the advanced fitter

TFT_eSPI tft = TFT_eSPI(); // Create TFT instance
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_HEIGHT 240
#define RAW_GRAPH_Y 0
#define COMPRESSED_GRAPH_Y 0

#define MAX_RAW_DATA 1024
static float raw_Data[MAX_RAW_DATA];
static uint32_t raw_timestamps[MAX_RAW_DATA];
float raw_graphMinY = 0;
float raw_graphMaxY = 0;
static uint16_t dataIndex = 0;

#define POLY_COUNT 64
#define SEGMENTS 8
#define MAX_SEGMENTS 8

struct PolynomialSegment {
    float coefficients[POLY_COUNT][6];
    uint16_t timeDeltas[POLY_COUNT];
};

PolynomialSegment segmentBuffer[SEGMENTS];
uint8_t segmentCount = 0;
uint32_t lastTimestamp = 0;

uint8_t head = 0;
uint8_t tail = 0;
uint8_t count = 0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void addSegment(const PolynomialSegment &newSegment) {
    segmentBuffer[tail] = newSegment;
    tail = (tail + 1) % SEGMENTS;

    if (count < SEGMENTS) {
        count++;
    } else {
        head = (head + 1) % SEGMENTS;
    }
}

void compressDataToSegment(const float *rawData, const uint16_t *timestamps, uint16_t dataSize, PolynomialSegment &segment) {
    AdvancedPolynomialFitter fitter;
    uint16_t segmentIndex = 0;

    for (uint16_t i = 0; i < dataSize; i += POLY_COUNT) {
        uint16_t chunkSize = (i + POLY_COUNT < dataSize) ? POLY_COUNT : (dataSize - i);

        std::vector<float> x(chunkSize);
        std::vector<float> y(chunkSize);

        for (uint16_t j = 0; j < chunkSize; j++) {
            x[j] = timestamps[i + j];
            y[j] = rawData[i + j];
        }

        std::vector<float> coefficients = fitter.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::GRADIENT_DESCENT);

        for (uint8_t j = 0; j < coefficients.size(); j++) {
            segment.coefficients[segmentIndex][j] = coefficients[j];
        }

        segment.timeDeltas[segmentIndex] = timestamps[i + chunkSize - 1] - timestamps[i];
        segmentIndex++;
        if (segmentIndex >= POLY_COUNT) break;
    }
}

// Replace existing calls to `fitNormalizedPolynomial` in other functions with `compressDataToSegment`.
