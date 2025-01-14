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
            // Initialize new segment's timeDeltas
            for (uint16_t i = 0; i < POLY_COUNT; i++) {
                segmentBuffer[tail].timeDeltas[i] = 0;
            }
        }

        // Fit polynomial to current data chunk
        float new_coefficients[6];
        uint32_t new_timeDelta;
        compressDataToSegment(rawData, timestamps, LOG_BUFFER_POINTS_PER_POLY, new_coefficients, new_timeDelta);

        // Store the polynomial in current segment
        for (uint8_t i = 0; i < 6; i++) {
            segmentBuffer[tail].coefficients[currentPolyIndex][i] = new_coefficients[i];
        }
        segmentBuffer[tail].timeDeltas[currentPolyIndex] = new_timeDelta;

        currentPolyIndex++;

        // If current segment is full, prepare for next segment
        if (currentPolyIndex >= POLY_COUNT) {
            currentPolyIndex = 0;
            
            if (segmentCount < SEGMENTS) {
                // Create new segment
                addSegment(PolynomialSegment());
                // Initialize timeDeltas for new segment
                for (uint16_t i = 0; i < POLY_COUNT; i++) {
                    segmentBuffer[tail].timeDeltas[i] = 0;
                }
            } else {
                // Trigger recompression when buffer is full
                recompressSegments();
            }
        }

        // Reset data buffer
        dataIndex = 0;
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

    stackPolynomials(oldest, secondOldest, recompressedSegment);

    // Remove the two oldest segments
    removeOldestTwo();
    
    // Add recompressed segment at the correct position (head)
    uint8_t insertPos = head;
    head = (head + 1) % SEGMENTS;  // Update head to next position
    
    // Shift existing segments if needed
    for (uint8_t i = segmentCount; i > 0; i--) {
        uint8_t currentPos = (insertPos + i - 1) % SEGMENTS;
        uint8_t newPos = (insertPos + i) % SEGMENTS;
        segmentBuffer[newPos] = segmentBuffer[currentPos];
    }
    
    // Insert recompressed segment
    segmentBuffer[insertPos] = recompressedSegment;
    segmentCount++; // Increment because we're adding one segment
}

void updateCompressedGraph(const PolynomialSegment *segments, uint8_t count) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // First pass: find value range across all valid segments
    uint8_t currentSegment = head;
    for (uint8_t i = 0; i < count; i++) {
        const PolynomialSegment &segment = segments[currentSegment];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
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
            uint32_t numSteps = min(50UL, tDelta);
            uint32_t stepSize = tDelta / numSteps;
            float lastX = -1, lastY = -1;
            
            for (uint32_t t = 0; t <= tDelta; t += stepSize) {
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
