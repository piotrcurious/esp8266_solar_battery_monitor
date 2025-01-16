void updateCompressedGraphBackwardsFast(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex) {
    if (count == 0) return;

    // Time window for alignment
    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];

    // Min/Max values for Y-axis scaling
    float minValue = INFINITY, maxValue = -INFINITY;

    // Initialize tracking indices
    int16_t segmentIndex = count - 1;
    int16_t polyIndex = polyindex;

    // First pass: Calculate min/max values
    uint32_t tCurrent = windowEnd;

    for (int16_t i = 0; i < count; ++i) {
        const PolynomialSegment &segment = segments[segmentIndex];
        for (int16_t j = (i == 0 ? polyIndex : POLY_COUNT - 1); j >= 0; --j) {
            uint32_t tDelta = segment.timeDeltas[j];
            if (tDelta == 0) continue;

            uint32_t numSteps = min(100UL, tDelta);
            uint32_t stepSize = tDelta / numSteps;

            for (uint32_t t = stepSize; t <= tDelta; t += stepSize) {
                uint32_t actualTime = tCurrent - t;
                if (actualTime < windowStart || actualTime > windowEnd) break;

                float value = evaluatePolynomial(segment.coefficients[j], t);
                minValue = min(minValue, value);
                maxValue = max(maxValue, value);
            }
            tCurrent -= tDelta;
            if (tCurrent < windowStart) break;
        }
        if (--segmentIndex < 0) break;
    }

    if (isinf(minValue) || isinf(maxValue)) {
        minValue = raw_graphMinY;
        maxValue = raw_graphMaxY;
    }

    // Add margin for aesthetics
    float valueRange = maxValue - minValue;
    minValue -= valueRange * 0.05f;
    maxValue += valueRange * 0.05f;

    // Plotting the compressed graph
    uint32_t xMin = windowStart, xMax = windowEnd;
    uint32_t lastDataX = xMax;

    segmentIndex = count - 1;
    polyIndex = polyindex;

    for (int x = SCREEN_WIDTH - 1; x >= 0; --x) {
        uint32_t dataX = mapUint(x, 0, SCREEN_WIDTH - 1, xMin, xMax);
        while (segmentIndex >= 0 && lastDataX - dataX >= segments[segmentIndex].timeDeltas[polyIndex]) {
            tft.drawLine(x, 0, x, SCREEN_HEIGHT, TFT_DARKGREEN);
            lastDataX -= segments[segmentIndex].timeDeltas[polyIndex];
            if (--polyIndex < 0) {
                polyIndex = POLY_COUNT - 1;
                if (--segmentIndex < 0) break;
                tft.drawLine(x, 0, x, SCREEN_HEIGHT, TFT_RED);
            }
        }

        // Compute the fitted Y value
        const PolynomialSegment &segment = segments[segmentIndex];
        uint32_t tDelta = lastDataX - dataX;
        float yFitted = 0.0f;

        for (uint8_t j = 0; j < 6; ++j) {
            yFitted += segment.coefficients[polyIndex][j] * powf(tDelta, j);
        }

        if (!isnan(yFitted)) {
            uint16_t y = mapFloat(yFitted, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
            if (y < SCREEN_HEIGHT) {
                tft.drawPixel(x, y, TFT_CYAN);
            }
        }
    }
}
