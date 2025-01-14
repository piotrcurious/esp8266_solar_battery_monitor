void generateCombinedData(PolynomialSegment &seg1, uint8_t poly1_start, uint8_t poly1_end,
                         PolynomialSegment &seg2, uint8_t poly2_start, uint8_t poly2_end,
                         float *combinedData, uint16_t *combinedTimestamps, uint16_t &combinedSize) {
    combinedSize = 0;
    const uint16_t SAMPLES_PER_POLY = 8;  // Number of samples per polynomial interval

    // Process polynomials from first segment
    uint32_t tAccum = 0;
    for (uint8_t i = poly1_start; i <= poly1_end; i++) {
        if (seg1.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg1.timeDeltas[i];
        for (uint16_t j = 0; j < SAMPLES_PER_POLY; j++) {
            float tNorm = (float)j / (SAMPLES_PER_POLY - 1);
            float value = evaluatePolynomial(seg1.coefficients[i], tNorm);
            
            combinedData[combinedSize] = value;
            combinedTimestamps[combinedSize] = tDelta / SAMPLES_PER_POLY;
            combinedSize++;
        }
        tAccum += tDelta;
    }

    // Process polynomials from second segment
    for (uint8_t i = poly2_start; i <= poly2_end; i++) {
        if (seg2.timeDeltas[i] == 0) break;
        
        uint32_t tDelta = seg2.timeDeltas[i];
        for (uint16_t j = 0; j < SAMPLES_PER_POLY; j++) {
            float tNorm = (float)j / (SAMPLES_PER_POLY - 1);
            float value = evaluatePolynomial(seg2.coefficients[i], tNorm);
            
            combinedData[combinedSize] = value;
            combinedTimestamps[combinedSize] = tDelta / SAMPLES_PER_POLY;
            combinedSize++;
        }
        tAccum += tDelta;
    }
}

void stackPolynomials(const PolynomialSegment &seg1, const PolynomialSegment &seg2, PolynomialSegment &result) {
    // Initialize result
    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        result.timeDeltas[i] = 0;
    }

    const uint8_t POLYS_TO_COMBINE = 4;  // Number of polynomials to combine into one
    uint8_t resultPolyIndex = 0;
    
    // Process first segment
    for (uint8_t i = 0; i < POLY_COUNT && resultPolyIndex < POLY_COUNT; i += POLYS_TO_COMBINE) {
        // Check if we have valid polynomials
        if (seg1.timeDeltas[i] == 0) break;
        
        // Determine end index for this group
        uint8_t endIdx = min(i + POLYS_TO_COMBINE - 1, (uint8_t)(POLY_COUNT - 1));
        while (endIdx > i && seg1.timeDeltas[endIdx] == 0) endIdx--;
        
        // Generate combined data points
        float combinedData[POLY_COUNT * 8];  // Plenty of space for samples
        uint16_t combinedTimestamps[POLY_COUNT * 8];
        uint16_t combinedSize = 0;
        
        PolynomialSegment& seg1_nonconst = const_cast<PolynomialSegment&>(seg1);
        PolynomialSegment& seg2_nonconst = const_cast<PolynomialSegment&>(seg2);
        
        generateCombinedData(seg1_nonconst, i, endIdx,
                           seg2_nonconst, 0, -1,  // Don't include seg2 yet
                           combinedData, combinedTimestamps, combinedSize);
        
        if (combinedSize >= 4) {  // Need at least 4 points for fitting
            float coefficients[6];
            uint16_t timeDelta;
            
            compressDataToSegment(combinedData, combinedTimestamps, combinedSize,
                                coefficients, timeDelta);
            
            // Store the result
            for (uint8_t j = 0; j < 6; j++) {
                result.coefficients[resultPolyIndex][j] = coefficients[j];
            }
            result.timeDeltas[resultPolyIndex] = timeDelta;
            
            Serial.print("Combined polynomials ");
            Serial.print(i);
            Serial.print(" to ");
            Serial.print(endIdx);
            Serial.print(" into result polynomial ");
            Serial.println(resultPolyIndex);
            
            resultPolyIndex++;
        }
    }
    
    // Process second segment similarly
    uint8_t startFromPoly = 0;
    for (uint8_t i = 0; i < POLY_COUNT && resultPolyIndex < POLY_COUNT; i += POLYS_TO_COMBINE) {
        if (seg2.timeDeltas[i] == 0) break;
        
        uint8_t endIdx = min(i + POLYS_TO_COMBINE - 1, (uint8_t)(POLY_COUNT - 1));
        while (endIdx > i && seg2.timeDeltas[endIdx] == 0) endIdx--;
        
        float combinedData[POLY_COUNT * 8];
        uint16_t combinedTimestamps[POLY_COUNT * 8];
        uint16_t combinedSize = 0;
        
        PolynomialSegment& seg1_nonconst = const_cast<PolynomialSegment&>(seg1);
        PolynomialSegment& seg2_nonconst = const_cast<PolynomialSegment&>(seg2);
        
        generateCombinedData(seg2_nonconst, i, endIdx,
                           seg1_nonconst, 0, -1,  // Don't include seg1
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
            
            Serial.print("Combined polynomials from seg2 ");
            Serial.print(i);
            Serial.print(" to ");
            Serial.print(endIdx);
            Serial.print(" into result polynomial ");
            Serial.println(resultPolyIndex);
            
            resultPolyIndex++;
        }
    }

    Serial.print("Final polynomial count after stacking: ");
    Serial.println(resultPolyIndex);
}
