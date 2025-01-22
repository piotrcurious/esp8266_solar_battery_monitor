// Add normalization helper functions
static float normalizeTime(float t, float tMax) {
    return t / tMax;
}

static float denormalizeTime(float tNorm, float tMax) {
    return tNorm * tMax;
}

static void compressDataToSegment(const PolynomialSegment *segments, uint8_t count, uint16_t polyindex, const float *rawData, const uint32_t *timestamps, uint16_t dataSize, float *coefficients, uint32_t &timeDelta) {
    AdvancedPolynomialFitter fitter;
    int8_t segmentIndex = count - 1;
    int16_t polyIndex = polyindex; 
    
    #define BOUNDARY_MARGIN3 16
    #define BOUNDARY_DELTA3 100
    std::vector<float> x3(dataSize + BOUNDARY_MARGIN3);
    std::vector<float> y3(dataSize + BOUNDARY_MARGIN3);

    float timestamp_absolute = 0;
    
    // Calculate total time span for normalization
    float totalTime = 0;
    for (uint16_t j = 0; j < dataSize; j++) {
        totalTime += timestamps[j];
    }

    // Add boundary points with normalized timestamps
    for (uint8_t i = 0; i < BOUNDARY_MARGIN3; i++) {
        x3[i] = normalizeTime(-(((BOUNDARY_MARGIN3)-i)*BOUNDARY_DELTA3), totalTime);
        if(currentPolyIndex == 0) {
            if (segmentIndex > 0) {
                uint32_t previous_poly_range = segments[segmentIndex-1].timeDeltas[POLY_COUNT-1];
                float tNorm = normalizeTime(previous_poly_range + x3[i] * totalTime, previous_poly_range);
                y3[i] = evaluatePolynomial(segments[segmentIndex-1].coefficients[POLY_COUNT-1], POLY_DEGREE+1, tNorm);
            } else {
                y3[i] = rawData[0];
            }
        } else {
            uint32_t previous_poly_range = segments[segmentIndex].timeDeltas[polyIndex-1];
            float tNorm = normalizeTime(previous_poly_range + x3[i] * totalTime, previous_poly_range);
            y3[i] = evaluatePolynomial(segments[segmentIndex].coefficients[polyIndex-1], POLY_DEGREE+1, tNorm);
        }
    }

    // Add main data points with normalized timestamps
    for (uint16_t j = 0; j < dataSize; j++) {
        timestamp_absolute += timestamps[j];
        x3[j + BOUNDARY_MARGIN3] = normalizeTime(timestamp_absolute, totalTime);
        y3[j + BOUNDARY_MARGIN3] = rawData[j];
    }

    // Fit polynomial with normalized x values
    std::vector<float> fitted_coefficients3 = fitter.fitPolynomial(x3, y3, SUB_FIT_POLY_DEGREE, AdvancedPolynomialFitter::NONE);

    // Store coefficients (these are now for normalized domain)
    for (uint8_t j = 0; j < fitted_coefficients3.size() && j < SUB_FIT_POLY_DEGREE+1; j++) {
        coefficients[j] = fitted_coefficients3[j];
    }

    // Now handle the main polynomial fitting with normalization
    #define BOUNDARY_MARGIN 16
    #define BOUNDARY_DELTA 100
    std::vector<float> x(dataSize + BOUNDARY_MARGIN);
    std::vector<float> y(dataSize + BOUNDARY_MARGIN);

    timestamp_absolute = 0;

    // Add boundary points with normalized timestamps
    for (uint8_t i = 0; i < BOUNDARY_MARGIN/2; i++) {
        x[i] = normalizeTime(-(((BOUNDARY_MARGIN/2)-i)*BOUNDARY_DELTA), totalTime);
        if(currentPolyIndex == 0) {
            if (segmentIndex > 0) {
                uint32_t previous_poly_range = segments[segmentIndex-1].timeDeltas[POLY_COUNT-1];
                float tNorm = normalizeTime(previous_poly_range + x[i] * totalTime, previous_poly_range);
                y[i] = evaluatePolynomial(segments[segmentIndex-1].coefficients[POLY_COUNT-1], POLY_DEGREE+1, tNorm);
            } else {
                y[i] = evaluatePolynomial(coefficients, SUB_FIT_POLY_DEGREE+1, x[i]);
            }
        } else {
            uint32_t previous_poly_range = segments[segmentIndex].timeDeltas[polyIndex-1];
            float tNorm = normalizeTime(previous_poly_range + x[i] * totalTime, previous_poly_range);
            y[i] = evaluatePolynomial(segments[segmentIndex].coefficients[polyIndex-1], POLY_DEGREE+1, tNorm);
        }
    }

    // Add main data points with normalized timestamps
    timestamp_absolute = 0;
    for (uint16_t j = 0; j < dataSize; j++) {
        timestamp_absolute += timestamps[j];
        x[j + BOUNDARY_MARGIN/2] = normalizeTime(timestamp_absolute, totalTime);
        y[j + BOUNDARY_MARGIN/2] = rawData[j];
    }

    // Add end boundary points
    for (uint8_t i = 0; i < BOUNDARY_MARGIN/2; i++) {
        x[dataSize + BOUNDARY_MARGIN/2 + i] = normalizeTime(timestamp_absolute + (((BOUNDARY_MARGIN/2)*i)*BOUNDARY_DELTA), totalTime);
        y[dataSize + BOUNDARY_MARGIN/2 + i] = evaluatePolynomial(coefficients, SUB_FIT_POLY_DEGREE+1, x[dataSize + BOUNDARY_MARGIN/2 + i]);
    }

    // Fit polynomial with normalized x values
    std::vector<float> fitted_coefficients = fitter.fitPolynomial(x, y, POLY_DEGREE, AdvancedPolynomialFitter::NONE);

    // Store coefficients (these are now for normalized domain)
    for (uint8_t j = 0; j < fitted_coefficients.size() && j < POLY_DEGREE+1; j++) {
        coefficients[j] = fitted_coefficients[j];
    }

    timeDelta = timestamp_absolute;
}

// Modified evaluation function to handle normalized time
static double evaluatePolynomial(const float *coefficients, uint8_t degree, double t) {
    // Normalize t to [0,1] range using the segment's time delta
    double tNorm = t / segmentBuffer[segmentCount-1].timeDeltas[currentPolyIndex];
    
    double result = 0.0;
    double tPower = 1.0;
    
    for (int i = 0; i < degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= tNorm;
    }
    return result;
}
