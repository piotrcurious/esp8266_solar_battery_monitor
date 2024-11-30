// Differential Polynomial Structure
struct Polynomial {
  int8_t a3, a2, a1, a0;  // Coefficients (8-bit differences or absolute for the first polynomial)
  float tEnd;             // End time of the polynomial
};

// Global Variables
Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

// Fit a polynomial with continuity from the previous polynomial
void fitDifferentialPolynomial(Polynomial& prevPoly, DataPoint* segment, int count, Polynomial& poly) {
  // Use the previous polynomial's end time as the new segment's start
  float tStart = prevPoly.tEnd;
  poly.tEnd = segment[count - 1].timestamp;

  // Adjust timestamps for continuity
  float adjustedTimestamps[count];
  for (int i = 0; i < count; i++) {
    adjustedTimestamps[i] = segment[i].timestamp - tStart;
  }

  // Fit a 3rd-order polynomial to the adjusted data (least squares)
  float sumX[4] = {0}, sumY = 0, sumXY[4] = {0}, sumX2Y = 0;
  for (int i = 0; i < count; i++) {
    float t = adjustedTimestamps[i];
    float v = segment[i].value;
    float t2 = t * t;
    float t3 = t2 * t;
    sumX[0] += t3 * t;
    sumX[1] += t3;
    sumX[2] += t2;
    sumX[3] += t;
    sumY += v;
    sumXY[0] += t3 * v;
    sumXY[1] += t2 * v;
    sumXY[2] += t * v;
    sumX2Y += v;
  }

  // Solve for coefficients (simplified for clarity)
  poly.a3 = quantizeCoefficient(sumXY[0] / sumX[0]) - prevPoly.a3;
  poly.a2 = quantizeCoefficient(sumXY[1] / sumX[1]) - prevPoly.a2;
  poly.a1 = quantizeCoefficient(sumXY[2] / sumX[2]) - prevPoly.a1;
  poly.a0 = quantizeCoefficient(sumX2Y / count) - prevPoly.a0;
}

// Compress data using differential polynomials
void compressTemporaryBuffer() {
  if (tempBufferCount == 0) return;

  int start = 0;

  // Store the first polynomial as absolute coefficients
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  fitPolynomial(&temporaryBuffer[start], tempBufferCount, firstPoly);
  storageBuffer[storageCount++] = firstPoly;

  // Compress subsequent data as differential polynomials
  while (start < tempBufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int end = start + 1;
    Polynomial currentPoly = {0, 0, 0, 0, 0};

    // Fit differential polynomial
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &temporaryBuffer[start], end - start + 1, currentPoly);
    storageBuffer[storageCount++] = currentPoly;

    start = end;  // Move to the next segment
  }

  resetTemporaryBuffer();
}

// Decompress a value using differential polynomials
float decompressValue(float timestamp) {
  Polynomial prevPoly = {0, 0, 0, 0, 0};  // Initialize with a baseline
  for (int i = 0; i < storageCount; i++) {
    Polynomial currentPoly = storageBuffer[i];
    // Compute absolute coefficients from differential encoding
    currentPoly.a3 += prevPoly.a3;
    currentPoly.a2 += prevPoly.a2;
    currentPoly.a1 += prevPoly.a1;
    currentPoly.a0 += prevPoly.a0;

    // Check if the timestamp falls within the polynomial's range
    if (timestamp <= currentPoly.tEnd) {
      float t = timestamp - (i == 0 ? 0 : prevPoly.tEnd);  // Normalize timestamp
      return currentPoly.a3 * t * t * t +
             currentPoly.a2 * t * t +
             currentPoly.a1 * t +
             currentPoly.a0;
    }
    prevPoly = currentPoly;  // Update previous polynomial
  }
  return NAN;  // Timestamp out of range
}

// Example usage
void setup() {
  Serial.begin(115200);

  // Simulate adding data points
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    addDataPoint(i * 0.5, sin(i * 0.1));
  }

  // Compress data
  compressTemporaryBuffer();

  // Print storage buffer
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tEnd=%.2f\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tEnd);
  }

  // Decompress a value
  float timestamp = 1.5;
  float value = decompressValue(timestamp);
  if (!isnan(value)) {
    Serial.printf("Decompressed value at %.2f: %.2f\n", timestamp, value);
  }
}

void loop() {}
