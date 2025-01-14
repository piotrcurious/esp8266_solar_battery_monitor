void combinePolynomials(const PolynomialSegment &oldest, const PolynomialSegment &secondOldest, PolynomialSegment &recompressedSegment) {
    AdvancedPolynomialFitter fitter;

    uint16_t currentPolyIndex = 0;

    for (uint16_t i = 0; i < POLY_COUNT; i++) {
        // Stop if no more valid time deltas
        if (oldest.timeDeltas[i] == 0 || secondOldest.timeDeltas[i] == 0) break;

        // Reconstruct data from both polynomials
        std::vector<float> timestamps;
        std::vector<float> values;

        // Reconstruct data points from the first polynomial
        uint32_t tStart = 0;
        for (uint32_t t = 0; t <= oldest.timeDeltas[i]; t += oldest.timeDeltas[i] / 50) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(oldest.coefficients[i], t));
        }

        // Adjust second polynomial's timestamps to continue from the end of the first
        tStart += oldest.timeDeltas[i];
        for (uint32_t t = 0; t <= secondOldest.timeDeltas[i]; t += secondOldest.timeDeltas[i] / 50) {
            timestamps.push_back(tStart + t);
            values.push_back(evaluatePolynomial(secondOldest.coefficients[i], t));
        }

        // Fit a new polynomial to the combined data
        std::vector<float> newCoefficients = fitter.fitPolynomial(timestamps, values, 5, AdvancedPolynomialFitter::NONE);

        // Store the new polynomial in the recompressed segment
        for (uint8_t j = 0; j < newCoefficients.size() && j < 6; j++) {
            recompressedSegment.coefficients[currentPolyIndex][j] = newCoefficients[j];
        }

        // Store the combined time delta
        recompressedSegment.timeDeltas[currentPolyIndex] = oldest.timeDeltas[i] + secondOldest.timeDeltas[i];
        currentPolyIndex++;
    }
}
