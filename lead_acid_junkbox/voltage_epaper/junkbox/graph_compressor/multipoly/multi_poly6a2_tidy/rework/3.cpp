// ... (previous code remains the same until evaluateCompressedValue and samplePolynomialPoints)

    float evaluateCompressedValue(uint32_t timestamp) {
        // Process segments backwards from newest to oldest
        uint32_t tCurrent = rawTimestamps[rawDataIndex - 1] - timestamp;
        
        for (int8_t i = segmentCount - 1; i >= 0; i--) {
            for (int8_t j = (i == segmentCount - 1 ? currentPolyIndex - 1 : POLY_COUNT - 1); j >= 0; j--) {
                if (segmentBuffer[i].timeDeltas[j] == 0) continue;
                
                uint32_t segmentDelta = segmentBuffer[i].timeDeltas[j];
                if (tCurrent <= segmentDelta) {
                    // Time within this polynomial's range
                    return evaluatePolynomial(segmentBuffer[i].coefficients[j], POLY_DEGREE+1, segmentDelta - tCurrent);
                }
                tCurrent -= segmentDelta;
            }
        }
        return NAN;
    }
    
    void samplePolynomialPoints(const float* coeffs1, uint32_t timeDelta1,
                               const float* coeffs2, uint32_t timeDelta2,
                               float resolution, std::vector<float>& timestamps,
                               std::vector<float>& values) {
        // Sample points from first polynomial
        for (float t = 0; t <= timeDelta1; t += resolution) {
            timestamps.push_back(t);
            values.push_back(evaluatePolynomial(coeffs1, POLY_DEGREE+1, t));
        }
        
        // Sample points from second polynomial with corrected timestamps
        for (float t = 0; t <= timeDelta2; t += resolution) {
            timestamps.push_back(t + timeDelta1);  // Offset timestamps by first polynomial's duration
            values.push_back(evaluatePolynomial(coeffs2, POLY_DEGREE+1, t));
        }
    }
    
    void combinePolynomials(const PolynomialSegment &first, const PolynomialSegment &second, PolynomialSegment &result) {
        AdvancedPolynomialFitter fitter;
        uint16_t currentPoly = 0;
        
        for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
            if (first.timeDeltas[i] == 0 || first.timeDeltas[i+1] == 0) break;
            
            std::vector<float> timestamps;
            std::vector<float> values;
            float resolution = (first.timeDeltas[i] + first.timeDeltas[i+1]) / MEMORY_LIMIT;
            
            // Use updated sampling function with proper timestamp handling
            samplePolynomialPoints(
                first.coefficients[i], first.timeDeltas[i],
                first.coefficients[i+1], first.timeDeltas[i+1],
                resolution, timestamps, values
            );
            
            // Fit new polynomial
            auto newCoeffs = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);
            
            // Store results
            for (uint8_t j = 0; j < newCoeffs.size() && j < POLY_DEGREE+1; j++) {
                result.coefficients[currentPoly][j] = newCoeffs[j];
            }
            result.timeDeltas[currentPoly] = first.timeDeltas[i] + first.timeDeltas[i+1];
            currentPoly++;
        }
        
        // Process second segment polynomials similarly
        for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
            if (second.timeDeltas[i] == 0 || second.timeDeltas[i+1] == 0) break;
            
            std::vector<float> timestamps;
            std::vector<float> values;
            float resolution = (second.timeDeltas[i] + second.timeDeltas[i+1]) / MEMORY_LIMIT;
            
            samplePolynomialPoints(
                second.coefficients[i], second.timeDeltas[i],
                second.coefficients[i+1], second.timeDeltas[i+1],
                resolution, timestamps, values
            );
            
            auto newCoeffs = fitter.fitPolynomial(timestamps, values, POLY_DEGREE, AdvancedPolynomialFitter::NONE);
            
            for (uint8_t j = 0; j < newCoeffs.size() && j < POLY_DEGREE+1; j++) {
                result.coefficients[currentPoly][j] = newCoeffs[j];
            }
            result.timeDeltas[currentPoly] = second.timeDeltas[i] + second.timeDeltas[i+1];
            currentPoly++;
        }
    }

// ... (rest of the code remains the same)
