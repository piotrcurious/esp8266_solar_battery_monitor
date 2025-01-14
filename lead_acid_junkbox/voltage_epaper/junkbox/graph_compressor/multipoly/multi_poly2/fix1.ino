void updateCompressedGraph(const PolynomialSegment *segments, uint8_t segmentCount) {
    if (segmentCount == 0) return;

    // Get the time window from raw data graph for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[dataIndex - 1];
    uint32_t timeSpan = windowEnd - windowStart;

    // Find min/max values for Y-axis scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    
    // First pass: find value range across all segments
    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];
        
        uint32_t tCurrent = 0;
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            // Sample polynomial at regular intervals
            for (float t = 0; t <= 1.0; t += 0.1) {
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
            }
            tCurrent += tDelta;
        }
    }

    // Ensure we have valid min/max values
    if (isinf(minValue) || isinf(maxValue)) {
        minValue = raw_graphMinY;
        maxValue = raw_graphMaxY;
    }

    // Add small margin to prevent values from touching screen edges
    float valueRange = maxValue - minValue;
    minValue -= valueRange * 0.05;
    maxValue += valueRange * 0.05;

    // Second pass: plot the data
    uint32_t tCurrent = 0;
    for (uint8_t segIndex = 0; segIndex < segmentCount; segIndex++) {
        const PolynomialSegment &segment = segments[segIndex];
        
        for (uint16_t polyIndex = 0; polyIndex < POLY_COUNT; polyIndex++) {
            if (segment.timeDeltas[polyIndex] == 0) break;
            
            uint32_t tStart = tCurrent;
            uint32_t tDelta = segment.timeDeltas[polyIndex];
            uint32_t tEnd = tStart + tDelta;
            
            // Skip polynomials outside the visible time window
            if (tStart > windowEnd || tEnd < windowStart) {
                tCurrent = tEnd;
                continue;
            }

            // Plot with higher resolution for smoother curves
            float stepSize = tDelta > 1000 ? 0.01f : 0.05f;
            float lastX = -1, lastY = -1;
            
            for (float t = 0; t <= 1.0; t += stepSize) {
                uint32_t tAbsolute = tStart + (uint32_t)(t * tDelta);
                
                // Skip points outside the visible window
                if (tAbsolute < windowStart || tAbsolute > windowEnd) continue;
                
                float value = evaluatePolynomial(segment.coefficients[polyIndex], t);
                
                // Map to screen coordinates
                uint16_t x = mapFloat(tAbsolute, windowStart, windowEnd, 0, SCREEN_WIDTH - 1);
                uint16_t y = mapFloat(value, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
                
                // Ensure coordinates are within screen bounds
                x = constrain(x, 0, SCREEN_WIDTH - 1);
                y = constrain(y, 0, SCREEN_HEIGHT - 1);
                
                // Draw line to current point if we have a previous point
                if (lastX >= 0 && lastY >= 0) {
                    tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
                } else {
                    tft.drawPixel(x, y, TFT_YELLOW);
                }
                
                lastX = x;
                lastY = y;
            }
            
            tCurrent = tEnd;
        }
    }
}
