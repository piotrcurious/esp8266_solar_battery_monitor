void recompressSegments() {
    if (segmentCount < 2) {
        Serial.println("Not enough segments to recompress.");
        return;
    }

    PolynomialSegment recompressedSegment;
    initializeSegment(recompressedSegment);

    stackPolynomials(segmentBuffer[head], segmentBuffer[(head + 1) % SEGMENTS], recompressedSegment);

    // Remove two oldest segments
    removeOldestTwo();

    // Add recompressed segment
    addSegment(recompressedSegment);

    Serial.print("Recompressed. New segment count: ");
    Serial.println(count);
}
