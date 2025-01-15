void updateCompressedGraph(const PolynomialSegment *segments, uint8_t count) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // Calculate total valid time span in polynomial log
    uint32_t totalPolyTime = 0;
    uint8_t lastValidSegment = head;
    uint16_t lastValidPoly = 0;
    
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
                    float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
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
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t numSteps = min(100UL, tDelta);
            int32_t stepSize = tDelta / numSteps;
            int32_t t = tDelta;  // Changed to signed
            
            while (t > 0) {  // Changed to while loop
                uint32_t actualTime = tCurrent - (tDelta - t);
                // Only plot points within the time window
                if (actualTime >= windowStart && actualTime <= windowEnd) {
                    float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                    uint16_t x = mapFloat(actualTime, windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
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
                t -= stepSize;
            }
            
            tCurrent -= tDelta;
            // If we've gone before the window start, we can stop
            if (tCurrent < windowStart) break;
        }
        if (tCurrent < windowStart) break;
    }
}
