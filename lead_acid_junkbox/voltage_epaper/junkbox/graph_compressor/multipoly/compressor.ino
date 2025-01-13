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



// Function to fit scalar data into a normalized 5th-degree polynomial
// `data` contains the scalar data
// `timestamps` contains corresponding 16-bit timestamps
// `size` is the number of data points
// `coefficients` stores the resulting quantized 8-bit polynomial coefficients
void fitNormalizedPolynomial(const float *data, const uint16_t *timestamps, uint16_t size, int8_t *coefficients) {
    if (size < 2) return; // Insufficient data to fit a polynomial

    // Step 1: Normalize timestamps to avoid large values
    uint16_t tMin = timestamps[0];
    float tRange = (float)(timestamps[size - 1] - tMin);
    if (tRange == 0) tRange = 1; // Avoid division by zero

    float normalizedT[size];
    for (uint16_t i = 0; i < size; i++) {
        normalizedT[i] = (float)(timestamps[i] - tMin) / tRange;
    }

    // Step 2: Construct and solve least squares system
    float A[6][6] = {0}; // Normal equations matrix
    float B[6] = {0};    // Right-hand side vector

    for (uint16_t i = 0; i < size; i++) {
        float t = normalizedT[i];
        float y = data[i];
        float powers[6] = {1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t};

        // Accumulate contributions to A and B
        for (uint8_t row = 0; row < 6; row++) {
            for (uint8_t col = 0; col < 6; col++) {
                A[row][col] += powers[row] * powers[col];
            }
            B[row] += powers[row] * y;
        }
    }

    // Solve Ax = B using Gaussian elimination (simplified for brevity)
    float x[6] = {0}; // Polynomial coefficients in double precision
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = i + 1; j < 6; j++) {
            float factor = A[j][i] / A[i][i];
            for (uint8_t k = 0; k < 6; k++) {
                A[j][k] -= factor * A[i][k];
            }
            B[j] -= factor * B[i];
        }
    }
    for (int8_t i = 5; i >= 0; i--) {
        x[i] = B[i];
        for (int8_t j = i + 1; j < 6; j++) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= A[i][i];
    }

    // Step 3: Quantize coefficients to 8-bit integers
    for (uint8_t i = 0; i < 6; i++) {
        coefficients[i] = (int8_t)(x[i] * 256); // Scale for 8-bit representation
    }
}


void compressDataToSegment(const float *rawData, const uint16_t *timestamps, uint16_t dataSize, PolynomialSegment &segment) {
    uint16_t segmentIndex = 0;

    // Divide data into chunks for each polynomial
    for (uint16_t i = 0; i < dataSize; i += POLY_COUNT) {
        uint16_t chunkSize = (i + POLY_COUNT < dataSize) ? POLY_COUNT : (dataSize - i);

        // Fit a polynomial for the current chunk
        fitNormalizedPolynomial(&rawData[i], &timestamps[i], chunkSize, segment.coefficients[segmentIndex]);

        // Store the time delta of the chunk
        segment.timeDeltas[segmentIndex] = timestamps[i + chunkSize - 1] - timestamps[i];

        segmentIndex++;
        if (segmentIndex >= POLY_COUNT) break;
    }
}

// Evaluate a polynomial at a given timestamp
float evaluatePolynomial(const int8_t *coefficients, float tNormalized) {
    float tPower = 1.0;
    float result = 0.0;

    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * tPower / 256.0; // Scale back from quantized 8-bit format
        tPower *= tNormalized;
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
