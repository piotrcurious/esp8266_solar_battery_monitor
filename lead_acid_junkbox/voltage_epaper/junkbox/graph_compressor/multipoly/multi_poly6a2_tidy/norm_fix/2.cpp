// Normalization helper functions
static float normalizeTime(float t, float tMax) {
    return t / tMax;
}

static float denormalizeTime(float tNorm, float tMax) {
    return tNorm * tMax;
}

// Previous compressDataToSegment and basic evaluatePolynomial functions remain the same...

static void combinePolynomials(const PolynomialSegment &oldest, const PolynomialSegment &secondOldest, PolynomialSegment &recompressedSegment) {
    AdvancedPolynomialFitter fitter;
    #define MEMORY_LIMIT 1000
    uint16_t currentPolyIndex = 0;

    for (uint16_t i = 0; i < POLY_COUNT; i=i+2) {
        if (oldest.timeDeltas[i] == 0 || oldest.timeDeltas[i+1] == 0) break;
        
        std::vector<float> timestamps;
        std::vector<float> values;
        float combinedTimeDelta = oldest.timeDeltas[i] + oldest.timeDeltas[i+1];
        float RECOMPRESS_RESOLUTION = combinedTimeDelta/MEMORY_LIMIT;

        // Sample points from first polynomial with normalized time
        uint32_t tStart = 0;
        for (float t = 0; t <= oldest.timeDeltas[i]; t += RECOMPRESS_RESOLUTION) {
            float tNorm = normalizeTime(t, oldest.timeDeltas[i]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomial(oldest.coefficients[i], POLY_DEGREE+1, tNorm));
        }

        // Sample points from second polynomial with normalized time
        tStart += oldest.timeDeltas[i];
        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {
            float tNorm = normalizeTime(t, oldest.timeDeltas[i+1]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomial(oldest.coefficients[i+1], POLY_DEGREE+1, tNorm));
        }

        // Fit new polynomial in normalized space
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);

        // Store coefficients (already for normalized domain)
        for (uint8_t j = 0; j < newCoefficients.size() && j < POLY_DEGREE+1; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        recompressedSegment.timeDeltas[currentPolyIndex] = combinedTimeDelta;
        currentPolyIndex++;
    }

    // Handle second oldest segment similarly
    for (uint16_t i = 0; i < POLY_COUNT; i=i+2) {
        if (secondOldest.timeDeltas[i] == 0 || secondOldest.timeDeltas[i+1] == 0) break;
        
        std::vector<float> timestamps;
        std::vector<float> values;
        float combinedTimeDelta = secondOldest.timeDeltas[i] + secondOldest.timeDeltas[i+1];
        float RECOMPRESS_RESOLUTION = combinedTimeDelta/MEMORY_LIMIT;

        uint32_t tStart = 0;
        for (float t = 0; t <= secondOldest.timeDeltas[i]; t += RECOMPRESS_RESOLUTION) {
            float tNorm = normalizeTime(t, secondOldest.timeDeltas[i]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i], POLY_DEGREE+1, tNorm));
        }

        tStart += secondOldest.timeDeltas[i];
        for (float t = 0; t <= secondOldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {
            float tNorm = normalizeTime(t, secondOldest.timeDeltas[i+1]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i+1], POLY_DEGREE+1, tNorm));
        }

        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);

        for (uint8_t j = 0; j < newCoefficients.size() && j < POLY_DEGREE+1; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        recompressedSegment.timeDeltas[currentPolyIndex] = combinedTimeDelta;
        currentPolyIndex++;
    }
}

void updateCompressedGraphBackwardsFastOpt(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex, bool clear_under, bool draw_lines) {
    if (count == 0) return;

    uint32_t windowStart = raw_timestamps[0];
    uint32_t windowEnd = raw_timestamps[raw_dataIndex - 1];
    float new_minValue = INFINITY, new_maxValue = -INFINITY;
    
    uint32_t xMin = windowStart, xMax = windowEnd - raw_log_delta;
    int16_t segmentIndex = count - 1;
    int16_t polyIndex = polyindex; 
    uint32_t lastDataX = xMax;
    uint16_t SWdelta = mapFloat(raw_log_delta+windowStart, windowStart, windowEnd, 0, SCREEN_WIDTH);
    uint16_t Swidth = SCREEN_WIDTH-SWdelta-1;
 
    if(SWdelta) {
        tft.drawRect(SCREEN_WIDTH-SWdelta, 0, SWdelta, SCREEN_HEIGHT-1, TFT_RED);
    }   
   
    int16_t lastY = -1;
    for (int x = Swidth; x >= 0; --x) {
        float dataX = mapFloat(x, +0.0, Swidth, xMin, xMax);  
        double tDelta = segments[segmentIndex].timeDeltas[polyIndex] - (lastDataX - dataX);   
        
        if(clear_under) {
            tft.drawFastVLine(x, 0, SCREEN_HEIGHT, TFT_BLACK);
        }
        
        while (segmentIndex >= 0 && ((lastDataX - dataX) >= segments[segmentIndex].timeDeltas[polyIndex])) {
            tft.drawFastVLine(x, 0, SCREEN_HEIGHT, 0x0821);
            lastDataX -= segments[segmentIndex].timeDeltas[polyIndex];
            if (--polyIndex < 0) {
                polyIndex = POLY_COUNT - 1;
                if (--segmentIndex < 0) break;
                tft.drawFastVLine(x, 0, SCREEN_HEIGHT, TFT_RED);
            }
            tDelta = segments[segmentIndex].timeDeltas[polyIndex] - (lastDataX - dataX);
        }

        // Normalize tDelta before evaluation
        float tNorm = normalizeTime(tDelta, segments[segmentIndex].timeDeltas[polyIndex]);
        double yFitted = 0.0f;
        double tPower = 1.0;
        
        for (uint8_t i = 0; i < POLY_DEGREE+1; i++) {
            yFitted += segments[segmentIndex].coefficients[polyIndex][i] * tPower;
            tPower *= tNorm;
        }

        new_minValue = min(new_minValue, (float)yFitted);
        new_maxValue = max(new_maxValue, (float)yFitted);

        uint16_t y = 0;
        if (!isnan(yFitted)) {
            y = mapFloat(yFitted, minValue, maxValue, SCREEN_HEIGHT - 1, 0);
            if (y < SCREEN_HEIGHT) {
                if(draw_lines && lastY > 0) {
                    tft.drawLine(x, y, x+1, lastY, TFT_YELLOW);  
                } else {
                    tft.drawPixel(x, y, TFT_WHITE);  
                }                
            }
        }
        lastY = y;
    }
    minValue = new_minValue;
    maxValue = new_maxValue;
}

void updateMinMax(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex, uint32_t windowStart, uint32_t windowEnd, bool clear_under, bool draw_lines) {
    int16_t segmentIndex = count - 1;
    int16_t polyIndex = polyindex;
    uint32_t tCurrent = windowEnd;
    minValue = INFINITY;
    maxValue = -INFINITY;
 
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

                // Normalize time before evaluation
                float tNorm = normalizeTime(t, tDelta);
                float value = evaluatePolynomial(segment.coefficients[j], POLY_DEGREE+1, tNorm);
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
}
