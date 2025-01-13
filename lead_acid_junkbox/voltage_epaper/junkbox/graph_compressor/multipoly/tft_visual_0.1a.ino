#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(); // Create TFT instance
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define GRAPH_HEIGHT 120
#define RAW_GRAPH_Y 20
#define COMPRESSED_GRAPH_Y (RAW_GRAPH_Y + GRAPH_HEIGHT + 10)

uint16_t rawGraphBuffer[SCREEN_WIDTH];  // Scrolling buffer for raw data visualization
uint16_t compressedGraphBuffer[SCREEN_WIDTH]; // Scrolling buffer for compressed data visualization
uint16_t graphIndex = 0; // Tracks the current position in the graph buffer

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
void updateRawGraph(float sampledData) {
    // Scale the data to fit the graph
    int graphValue = GRAPH_HEIGHT - (int)(sampledData * (GRAPH_HEIGHT / 10.0));
    if (graphValue < 0) graphValue = 0;
    if (graphValue >= GRAPH_HEIGHT) graphValue = GRAPH_HEIGHT - 1;

    // Update the graph buffer
    rawGraphBuffer[graphIndex] = graphValue;

    // Clear the previous point
    tft.drawPixel(graphIndex, RAW_GRAPH_Y + rawGraphBuffer[graphIndex], TFT_BLACK);

    // Draw the new point
    tft.drawPixel(graphIndex, RAW_GRAPH_Y + graphValue, TFT_GREEN);

    // Increment the graph index
    graphIndex = (graphIndex + 1) % SCREEN_WIDTH;
}


void updateCompressedGraph(const PolynomialSegment *segments, uint8_t segmentCount) {
    if (segmentCount == 0) {
        // No segments to visualize
        return;
    }

    // Clear the previous compressed graph
    tft.fillRect(0, COMPRESSED_GRAPH_Y, SCREEN_WIDTH, GRAPH_HEIGHT, TFT_BLACK);

    // Initialize graphing parameters
    uint32_t tCurrent = 0; // Absolute time tracker for continuity
    uint16_t x = 0;        // Screen x-coordinate for plotting

    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];

        // Process each polynomial in the segment
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            uint16_t tDelta = segment.timeDeltas[polyIndex];
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
                float reconstructedValue = evaluatePolynomial(segment.coefficients[polyIndex], tNorm);

                // Scale the data to fit the graph
                int graphValue = GRAPH_HEIGHT - (int)(reconstructedValue * (GRAPH_HEIGHT / 10.0));
                if (graphValue < 0) graphValue = 0;
                if (graphValue >= GRAPH_HEIGHT) graphValue = GRAPH_HEIGHT - 1;

                // Draw the point on the graph
                tft.drawPixel(x, COMPRESSED_GRAPH_Y + graphValue, TFT_RED);
                x++;
            }

            // Update the current time
            tCurrent = tEnd;
        }
    }
}


void loop() {
    // Simulate sampling at random intervals
    delay(random(50, 500)); // Random delay between 50 ms to 500 ms

    // Sample scalar data
    float sampledData = sampleScalarData();

    // Log the sampled data
    uint32_t currentTimestamp = millis();
    logSampledData(sampledData, currentTimestamp);

    // Update the raw data graph
    updateRawGraph(sampledData);

    // Update the compressed data graph
    updateCompressedGraph(segmentBuffer, segmentCount);
}

