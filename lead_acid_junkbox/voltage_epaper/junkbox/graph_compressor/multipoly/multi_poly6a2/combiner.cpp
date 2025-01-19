// Helper struct to store boundary analysis results
struct BoundaryAnalysis {
    float valueDifference;      // Value difference at boundary
    float slopeDiscontinuity;   // First derivative difference at boundary
    float curvatureChange;      // Second derivative difference at boundary
    float boundaryPoint;        // Time point where polynomials meet
};

// Helper struct to store combined polynomial characteristics
struct CombinedFeatures {
    float maxDiscontinuity;     // Maximum jump in value
    float maxSlopeChange;       // Maximum sudden change in slope
    float errorMetric;          // Overall error metric for the fit
    bool isSmooth;             // C1 continuity at boundary
    bool preservesMonotonicity; // Whether monotonicity is preserved
    std::vector<float> criticalPoints; // Points where derivative changes sign
};

// Analyze boundary between two polynomials
BoundaryAnalysis analyzeBoundary(const float* coeff1, const float* coeff2, 
                                float t1, float t2, uint8_t degree) {
    BoundaryAnalysis analysis;
    analysis.boundaryPoint = t1;

    // Calculate values at boundary
    float v1 = evaluatePolynomial(coeff1, degree, t1);
    float v2 = evaluatePolynomial(coeff2, degree, 0); // t=0 for second polynomial
    analysis.valueDifference = abs(v2 - v1);

    // Calculate first derivatives at boundary
    float d1 = evaluatePolynomialDerivative(coeff1, degree, t1);
    float d2 = evaluatePolynomialDerivative(coeff2, degree, 0);
    analysis.slopeDiscontinuity = abs(d2 - d1);

    // Calculate second derivatives at boundary
    float c1 = evaluatePolynomialSecondDerivative(coeff1, degree, t1);
    float c2 = evaluatePolynomialSecondDerivative(coeff2, degree, 0);
    analysis.curvatureChange = abs(c2 - c1);

    return analysis;
}

// Analyze features of combined polynomial from actual data points
CombinedFeatures analyzeCombinedPolynomial(const std::vector<float>& timestamps,
                                          const std::vector<float>& values,
                                          const std::vector<float>& fittedCoeffs,
                                          float boundaryPoint,
                                          float tolerance = 1e-4) {
    CombinedFeatures features;
    features.maxDiscontinuity = 0;
    features.maxSlopeChange = 0;
    features.errorMetric = 0;
    features.isSmooth = true;
    features.preservesMonotonicity = true;
    
    float prevSlope = 0;
    float prevValue = 0;
    bool firstPoint = true;
    bool wasIncreasing = false;
    
    // Analyze actual vs fitted data
    for (size_t i = 0; i < timestamps.size(); i++) {
        float t = timestamps[i];
        float actualValue = values[i];
        float fittedValue = evaluatePolynomial(fittedCoeffs.data(), fittedCoeffs.size(), t);
        float slope = evaluatePolynomialDerivative(fittedCoeffs.data(), fittedCoeffs.size(), t);
        
        // Calculate error metric
        float error = abs(fittedValue - actualValue);
        features.errorMetric = max(features.errorMetric, error);
        
        if (!firstPoint) {
            // Check for sudden value changes
            float valueDiff = abs(fittedValue - prevValue);
            features.maxDiscontinuity = max(features.maxDiscontinuity, valueDiff);
            
            // Check for sudden slope changes
            float slopeDiff = abs(slope - prevSlope);
            features.maxSlopeChange = max(features.maxSlopeChange, slopeDiff);
            
            // Check monotonicity preservation
            bool isIncreasing = slope > 0;
            if (!firstPoint && isIncreasing != wasIncreasing) {
                features.criticalPoints.push_back(t);
            }
            wasIncreasing = isIncreasing;
            
            // Special analysis near boundary point
            if (abs(t - boundaryPoint) < tolerance) {
                features.isSmooth = abs(slope - prevSlope) < tolerance;
            }
        }
        
        prevValue = fittedValue;
        prevSlope = slope;
        firstPoint = false;
    }
    
    return features;
}

// Main polynomial combination function with improved analysis
void combinePolynomials(const PolynomialSegment &oldest, 
                       const PolynomialSegment &secondOldest, 
                       PolynomialSegment &recompressedSegment,
                       float maxAllowedError = 0.1f) {
    AdvancedPolynomialFitter fitter;
    uint16_t currentPolyIndex = 0;
    const float MEMORY_LIMIT = 1000;

    for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
        if (oldest.timeDeltas[i] == 0 || oldest.timeDeltas[i+1] == 0) break;

        float resolution = (oldest.timeDeltas[i] + oldest.timeDeltas[i+1]) / MEMORY_LIMIT;
        
        // Analyze boundary between polynomials
        BoundaryAnalysis boundary = analyzeBoundary(
            oldest.coefficients[i], oldest.coefficients[i+1],
            oldest.timeDeltas[i], oldest.timeDeltas[i+1], 6
        );

        // Generate combined data points
        std::vector<float> timestamps;
        std::vector<float> values;
        uint32_t tStart = 0;

        // First polynomial data points
        for (float t = 0; t <= oldest.timeDeltas[i]; t += resolution) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i], 6, t));
        }

        // Second polynomial data points
        tStart += oldest.timeDeltas[i];
        for (float t = 0; t <= oldest.timeDeltas[i+1]; t += resolution) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i+1], 6, t));
        }

        // Fit combined polynomial
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, 
                                                                 AdvancedPolynomialFitter::NONE);

        // Analyze features of the combined polynomial
        CombinedFeatures features = analyzeCombinedPolynomial(
            timestamps, values, newCoefficients, oldest.timeDeltas[i]
        );

        // Decide whether to combine based on mathematical analysis
        bool canCombine = 
            features.isSmooth &&                          // C1 continuity
            features.maxDiscontinuity < maxAllowedError && // No large jumps
            features.maxSlopeChange < maxAllowedError &&   // No sudden slope changes
            features.errorMetric < maxAllowedError &&      // Good overall fit
            features.criticalPoints.size() <= 2;           // Limited oscillation

        if (!canCombine) {
            // Find optimal split point near region of highest discontinuity
            float optimalSplit = oldest.timeDeltas[i]; // Default to original boundary
            float minErrorMetric = features.errorMetric;
            
            // Search for better split point around original boundary
            for (float splitT = oldest.timeDeltas[i] * 0.8f; 
                 splitT <= oldest.timeDeltas[i] * 1.2f; 
                 splitT += resolution) {
                    
                // Generate split data points
                std::vector<float> splitTimestamps;
                std::vector<float> splitValues;
                
                for (size_t j = 0; j < timestamps.size(); j++) {
                    if (timestamps[j] <= splitT) {
                        splitTimestamps.push_back(timestamps[j]);
                        splitValues.push_back(values[j]);
                    }
                }
                
                if (splitTimestamps.size() < 3) continue; // Need minimum points for fitting
                
                // Try fitting polynomial to split section
                std::vector<float> splitCoeffs = fitter.fitPolynomial(
                    splitTimestamps, splitValues, 5, AdvancedPolynomialFitter::NONE
                );
                
                // Analyze split results
                CombinedFeatures splitFeatures = analyzeCombinedPolynomial(
                    splitTimestamps, splitValues, splitCoeffs, splitT
                );
                
                if (splitFeatures.errorMetric < minErrorMetric) {
                    minErrorMetric = splitFeatures.errorMetric;
                    optimalSplit = splitT;
                }
            }
            
            // Store first part up to optimal split
            std::vector<float> firstPartTimestamps;
            std::vector<float> firstPartValues;
            
            for (size_t j = 0; j < timestamps.size(); j++) {
                if (timestamps[j] <= optimalSplit) {
                    firstPartTimestamps.push_back(timestamps[j]);
                    firstPartValues.push_back(values[j]);
                }
            }
            
            // Fit and store first part
            std::vector<float> firstPartCoeffs = fitter.fitPolynomial(
                firstPartTimestamps, firstPartValues, 5, AdvancedPolynomialFitter::NONE
            );
            
            for (uint8_t j = 0; j < firstPartCoeffs.size() && j < 6; j++) {
                recompressedSegment.coefficients[currentPolyIndex][j] = firstPartCoeffs[j];
            }
            recompressedSegment.timeDeltas[currentPolyIndex] = optimalSplit;
            currentPolyIndex++;
            
            // Handle remaining part
            std::vector<float> secondPartTimestamps;
            std::vector<float> secondPartValues;
            
            for (size_t j = 0; j < timestamps.size(); j++) {
                if (timestamps[j] > optimalSplit) {
                    secondPartTimestamps.push_back(timestamps[j] - optimalSplit);
                    secondPartValues.push_back(values[j]);
                }
            }
            
            if (secondPartTimestamps.size() >= 3) {
                std::vector<float> secondPartCoeffs = fitter.fitPolynomial(
                    secondPartTimestamps, secondPartValues, 5, AdvancedPolynomialFitter::NONE
                );
                
                for (uint8_t j = 0; j < secondPartCoeffs.size() && j < 6; j++) {
                    recompressedSegment.coefficients[currentPolyIndex][j] = secondPartCoeffs[j];
                }
                recompressedSegment.timeDeltas[currentPolyIndex] = 
                    (oldest.timeDeltas[i] + oldest.timeDeltas[i+1]) - optimalSplit;
                currentPolyIndex++;
            }
        } else {
            // Store combined polynomial
            for (uint8_t j = 0; j < newCoefficients.size() && j < 6; j++) {
                recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
            }
            recompressedSegment.timeDeltas[currentPolyIndex] = 
                oldest.timeDeltas[i] + oldest.timeDeltas[i+1];
            currentPolyIndex++;
        }
    }

    // Process secondOldest segment similarly...
    // (Similar code for secondOldest segment)
}
