// Add helper function for already normalized time evaluation
static double evaluatePolynomialNormalized(const float *coefficients, uint8_t degree, double tNorm) {
    double result = 0.0;
    double tPower = 1.0;  // tNorm^0 = 1
    
    for (int i = 0; i < degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= tNorm;  // More efficient than pow()
    }
    return result;
}

// Modified combinePolynomials function
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

        // Sample points from first polynomial
        uint32_t tStart = 0;
        for (float t = 0; t <= oldest.timeDeltas[i]; t += RECOMPRESS_RESOLUTION) {
            // Use normalized time for polynomial evaluation
            float tNorm1 = normalizeTime(t, oldest.timeDeltas[i]);
            // But normalize the timestamp for the new combined polynomial
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomialNormalized(oldest.coefficients[i], POLY_DEGREE+1, tNorm1));
        }

        // Sample points from second polynomial
        tStart += oldest.timeDeltas[i];
        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {
            float tNorm2 = normalizeTime(t, oldest.timeDeltas[i+1]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomialNormalized(oldest.coefficients[i+1], POLY_DEGREE+1, tNorm2));
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
            float tNorm1 = normalizeTime(t, secondOldest.timeDeltas[i]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomialNormalized(secondOldest.coefficients[i], POLY_DEGREE+1, tNorm1));
        }

        tStart += secondOldest.timeDeltas[i];
        for (float t = 0; t <= secondOldest.timeDeltas[i+1]; t += RECOMPRESS_RESOLUTION) {
            float tNorm2 = normalizeTime(t, secondOldest.timeDeltas[i+1]);
            timestamps.push_back(normalizeTime(tStart + t, combinedTimeDelta));
            values.push_back(evaluatePolynomialNormalized(secondOldest.coefficients[i+1], POLY_DEGREE+1, tNorm2));
        }

        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);

        for (uint8_t j = 0; j < newCoefficients.size() && j < POLY_DEGREE+1; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        recompressedSegment.timeDeltas[currentPolyIndex] = combinedTimeDelta;
        currentPolyIndex++;
    }
}
