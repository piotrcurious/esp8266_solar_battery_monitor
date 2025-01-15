void updateCompressedGraph(const PolynomialSegment *segments, uint8_t count) {
    if (count == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
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
            // Evaluate backwards from tDelta to 0
            for (uint32_t t = tDelta; t != UINT32_MAX; t -= max(1UL, tDelta/10)) {
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
                if (t == 0) break; // Handle the case when t wraps around
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

    // Second pass: plot the data backwards
    uint32_t tEnd = windowEnd;
    currentSegment = (head + count - 1) % SEGMENTS; // Start from the last segment
    
    for (int8_t i = count - 1; i >= 0; i--) {
        const PolynomialSegment &segment = segments[currentSegment];
        
        // Process polynomials in reverse order
        for (int16_t polyIndex = POLY_COUNT - 1; polyIndex >= 0; polyIndex--) {
            if (segment.timeDeltas[polyIndex] == 0) continue;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t numSteps = min(100UL, tDelta);
            uint32_t stepSize = tDelta / numSteps;
            float lastX = -1, lastY = -1;
            
            // Evaluate backwards from tEnd
            for (uint32_t t = tDelta; t != UINT32_MAX; t -= stepSize) {
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                uint16_t x = mapFloat(tEnd - (tDelta - t), windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
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
                
                if (t == 0) break; // Handle the case when t reaches 0
            }
            
            tEnd -= tDelta;
        }
        currentSegment = (currentSegment - 1 + SEGMENTS) % SEGMENTS;
    }
}
