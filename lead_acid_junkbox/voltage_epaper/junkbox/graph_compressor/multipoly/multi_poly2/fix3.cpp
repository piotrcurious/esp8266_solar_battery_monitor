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

void logSampledData(float data, uint32_t currentTimestamp) {
    static float rawData[POLY_COUNT];
    static uint16_t timestamps[POLY_COUNT];
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
    const uint16_t POINTS_PER_POLY = 8;  // Adjust this value as needed
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
            if (segmentCount < MAX_SEGMENTS) {
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
