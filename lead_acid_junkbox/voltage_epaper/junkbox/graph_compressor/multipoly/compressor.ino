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
