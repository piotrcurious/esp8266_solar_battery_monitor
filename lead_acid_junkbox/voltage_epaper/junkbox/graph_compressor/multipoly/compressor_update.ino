#define MAX_SEGMENTS 8
#define POLY_COUNT 64

struct PolynomialSegment {
    int8_t coefficients[POLY_COUNT][6]; // Quantized coefficients for each polynomial
    uint16_t timeDeltas[POLY_COUNT];    // Time deltas for each polynomial
};

// Global buffer for polynomial segments
PolynomialSegment segmentBuffer[MAX_SEGMENTS];
uint8_t segmentCount = 0;  // Number of active segments in the buffer

// Tracking previous timestamp
uint32_t lastTimestamp = 0;

// Sample scalar data (simulated random data for now)
float sampleScalarData() {
    return (float)(random(0, 1000)) / 100.0; // Random data in range [0, 10.0]
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
        compressDataToSegmentWithNormalization(rawData, timestamps, dataIndex, newSegment);

        // Add the new segment to the buffer
        if (segmentCount < MAX_SEGMENTS) {
            segmentBuffer[segmentCount++] = newSegment;
        } else {
            // Recompress the buffer if full
            recompressWithStacking();
            segmentBuffer[segmentCount - 1] = newSegment; // Replace the oldest after recompression
        }

        // Reset the data index for the next segment
        dataIndex = 0;
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0)); // Initialize randomness for simulation
    lastTimestamp = millis();
}

void loop() {
    // Simulate sampling at random intervals
    delay(random(50, 500)); // Random delay between 50 ms to 500 ms

    // Sample scalar data
    float sampledData = sampleScalarData();

    // Log the sampled data
    uint32_t currentTimestamp = millis();
    logSampledData(sampledData, currentTimestamp);

    // Debug output (optional)
    Serial.print("Sampled Data: ");
    Serial.println(sampledData);
    Serial.print("Segment Count: ");
    Serial.println(segmentCount);
}

