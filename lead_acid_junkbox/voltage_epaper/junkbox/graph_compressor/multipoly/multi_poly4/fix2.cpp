// Fixed polynomial evaluation to match logging time scale

float evaluatePolynomial(const float *coefficients, float t) {
    // t is already in milliseconds within the segment's time delta range
    float result = 0.0;
    float tPower = 1.0;  // t^0 = 1
    
    for (int i = 0; i < 6; i++) {
        result += coefficients[i] * tPower;
        tPower *= t;  // More efficient than pow()
    }
    return result;
}

void generateCombinedData(PolynomialSegment &seg1, uint8_t poly1_start, uint8_t poly1_end,
                         PolynomialSegment &seg2, uint8_t poly2_start, uint8_t poly2_end,
                         float *combinedData, uint16_t *combinedTimestamps, uint16_t &combinedSize) {
    combinedSize = 0;
    const uint16_t SAMPLES_PER_POLY = 16;  // Increased for better accuracy
    uint32_t timeOffset = 0;

    // Process polynomials from first segment
    for (uint8_t i = poly1_start; i <= poly1_end && i < POLY_COUNT; i++) {
        if (seg1.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg1.timeDeltas[i];
        float stepSize = (float)tDelta / (SAMPLES_PER_POLY - 1);
        
        for (uint16_t j = 0; j < SAMPLES_PER_POLY && combinedSize < POLY_COUNT * SAMPLES_PER_POLY; j++) {
            float t = j * stepSize;  // Time in milliseconds from start of polynomial
            
            combinedData[combinedSize] = evaluatePolynomial(seg1.coefficients[i], t);
            combinedTimestamps[combinedSize] = (j == 0 && i > poly1_start) ? 0 : stepSize;
            combinedSize++;
        }
        timeOffset += tDelta;
    }

    // Process polynomials from second segment with proper time continuation
    for (uint8_t i = poly2_start; i <= poly2_end && i < POLY_COUNT; i++) {
        if (seg2.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg2.timeDeltas[i];
        float stepSize = (float)tDelta / (SAMPLES_PER_POLY - 1);
        
        for (uint16_t j = 0; j < SAMPLES_PER_POLY && combinedSize < POLY_COUNT * SAMPLES_PER_POLY; j++) {
            float t = j * stepSize;  // Time in milliseconds from start of polynomial
            
            combinedData[combinedSize] = evaluatePolynomial(seg2.coefficients[i], t);
            combinedTimestamps[combinedSize] = (j == 0 && i > poly2_start) ? 0 : stepSize;
            combinedSize++;
        }
        timeOffset += tDelta;
    }
}

void stackPolynomials(const PolynomialSegment &seg1, const PolynomialSegment &seg2, PolynomialSegment &result) {
    const uint8_t POLYS_TO_COMBINE = 2;  // Reduced to improve accuracy
    uint8_t resultPolyIndex = 0;
    
    // Initialize result
    for (uint8_t i = 0; i < POLY_COUNT; i++) {
        result.timeDeltas[i] = 0;
        for (uint8_t j = 0; j < 6; j++) {
            result.coefficients[i][j] = 0;
        }
    }
    
    // Process segments sequentially
    for (uint8_t segIndex = 0; segIndex < 2; segIndex++) {
        const PolynomialSegment &currentSeg = (segIndex == 0) ? seg1 : seg2;
        
        for (uint8_t i = 0; i < POLY_COUNT && resultPolyIndex < POLY_COUNT; i += POLYS_TO_COMBINE) {
            if (currentSeg.timeDeltas[i] == 0) continue;
            
            uint8_t endIdx = min(i + POLYS_TO_COMBINE - 1, POLY_COUNT - 1);
            while (endIdx > i && currentSeg.timeDeltas[endIdx] == 0) endIdx--;
            
            float combinedData[POLY_COUNT * 16];
            uint16_t combinedTimestamps[POLY_COUNT * 16];
            uint16_t combinedSize = 0;
            
            generateCombinedData(const_cast<PolynomialSegment&>(currentSeg), i, endIdx,
                               const_cast<PolynomialSegment&>(currentSeg), 0, -1,
                               combinedData, combinedTimestamps, combinedSize);
            
            if (combinedSize >= 4) {
                float coefficients[6];
                uint16_t timeDelta;
                
                compressDataToSegment(combinedData, combinedTimestamps, combinedSize,
                                    coefficients, timeDelta);
                
                for (uint8_t j = 0; j < 6; j++) {
                    result.coefficients[resultPolyIndex][j] = coefficients[j];
                }
                result.timeDeltas[resultPolyIndex] = timeDelta;
                resultPolyIndex++;
            }
        }
    }
}

void recompressSegments() {
    if (count < 2) return;

    PolynomialSegment oldest, secondOldest;
    getOldestSegments(oldest, secondOldest);
    
    // Create recompressed segment
    PolynomialSegment recompressedSegment;
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        recompressedSegment.timeDeltas[i] = 0;
    }

    stackPolynomials(oldest, secondOldest, recompressedSegment);

    // Remove the two oldest segments and add the recompressed one
    removeOldestTwo();
    addSegment(recompressedSegment);

    Serial.print("Recompressed. New segment count: ");
    Serial.println(count);
}
