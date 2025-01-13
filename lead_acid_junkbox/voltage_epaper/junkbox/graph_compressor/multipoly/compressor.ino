#include <stdint.h>

#define POLY_COUNT 64 // Number of polynomials in each segment
#define SEGMENTS 8    // Total number of segments

// Storage structure
struct PolynomialSegment {
    int8_t coefficients[POLY_COUNT][6]; // 5th degree (6 coefficients), quantized to 8-bit
    uint16_t timeDeltas[POLY_COUNT];   // 16-bit time deltas
};

// Data buffer
PolynomialSegment segments[SEGMENTS];

// Rolling buffer indices
uint8_t head = 0; // Points to the oldest segment
uint8_t tail = 0; // Points to the newest segment
uint8_t count = 0; // Number of valid segments in buffer

// Add a new segment to the buffer
void addSegment(const PolynomialSegment &newSegment) {
    segments[tail] = newSegment;
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
    oldest = segments[head];
    secondOldest = segments[(head + 1) % SEGMENTS];
}

// Remove the oldest two segments
void removeOldestTwo() {
    head = (head + 2) % SEGMENTS;
    count -= 2;
}

#include <math.h>

// Function to fit data into a 5th-degree polynomial
// `data` is an array of raw scalar data, `size` is the number of samples
// `coefficients` stores the resulting quantized 8-bit polynomial coefficients
void fitPolynomial(const float *data, uint16_t size, int8_t *coefficients) {
    float a[6] = {0}; // Holds double-precision polynomial coefficients

    // Perform a least-squares fit (simplified example for brevity)
    for (uint16_t i = 0; i < size; i++) {
        float x = (float)i;
        float y = data[i];

        // Construct polynomial terms
        a[0] += 1;
        a[1] += x;
        a[2] += x * x;
        a[3] += x * x * x;
        a[4] += x * x * x * x;
        a[5] += x * x * x * x * x;
        // Accumulate weighted sums (simplified, you can replace with matrix inversion)
    }

    // Quantize coefficients to 8-bit integers
    for (int i = 0; i < 6; i++) {
        coefficients[i] = (int8_t)(a[i] / 256.0f); // Normalize and quantize
    }
}

// Function to compress raw data into a polynomial segment
// `rawData` is the input scalar data, `timeStamps` are the corresponding timestamps
// `segment` is the output polynomial segment
void compressDataToSegment(const float *rawData, const uint16_t *timeStamps, uint16_t dataSize, PolynomialSegment &segment) {
    uint16_t segmentIndex = 0;

    // Divide raw data into chunks for each polynomial
    for (uint16_t i = 0; i < dataSize; i += POLY_COUNT) {
        uint16_t chunkSize = (i + POLY_COUNT < dataSize) ? POLY_COUNT : (dataSize - i);

        // Fit a polynomial for the current chunk
        fitPolynomial(&rawData[i], chunkSize, segment.coefficients[segmentIndex]);

        // Calculate time deltas for the chunk
        segment.timeDeltas[segmentIndex] = (timeStamps[i + chunkSize - 1] - timeStamps[i]);

        segmentIndex++;
        if (segmentIndex >= POLY_COUNT) {
            break;
        }
    }
}

// Function to recompress two segments into one
// `oldest` and `secondOldest` are the input segments
// `result` is the output recompressed segment
void recompressSegments(const PolynomialSegment &oldest, const PolynomialSegment &secondOldest, PolynomialSegment &result) {
    uint16_t combinedIndex = 0;

    // Combine coefficients and time deltas
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        if (combinedIndex >= POLY_COUNT) break;

        // Combine coefficients by averaging
        for (uint8_t j = 0; j < 6; j++) {
            result.coefficients[combinedIndex][j] = (oldest.coefficients[i][j] + secondOldest.coefficients[i][j]) / 2;
        }

        // Combine time deltas by summing
        result.timeDeltas[combinedIndex] = oldest.timeDeltas[i] + secondOldest.timeDeltas[i];
        combinedIndex++;
    }

    // Handle remaining polynomials if the second segment is larger
    if (combinedIndex < POLY_COUNT) {
        for (uint16_t i = 0; i < POLY_COUNT && combinedIndex < POLY_COUNT; i++) {
            // Copy remaining secondOldest data
            for (uint8_t j = 0; j < 6; j++) {
                result.coefficients[combinedIndex][j] = secondOldest.coefficients[i][j];
            }
            result.timeDeltas[combinedIndex] = secondOldest.timeDeltas[i];
            combinedIndex++;
        }
    }
}

// Recompress the oldest two segments in the buffer
void recompressOldestSegments() {
    if (count < 2) {
        // Not enough segments to recompress
        return;
    }

    PolynomialSegment oldest, secondOldest, recompressedSegment;

    // Retrieve the oldest and second-oldest segments
    getOldestSegments(oldest, secondOldest);

    // Recompress the two segments
    recompressSegments(oldest, secondOldest, recompressedSegment);

    // Remove the two oldest segments
    removeOldestTwo();

    // Add the recompressed segment back into the buffer
    addSegment(recompressedSegment);
}
