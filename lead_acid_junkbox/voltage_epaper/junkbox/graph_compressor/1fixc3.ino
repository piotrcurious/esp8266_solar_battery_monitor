// Optimized Polynomial Structure
struct Polynomial {
  int8_t a3, a2, a1, a0;  // 8-bit differential coefficients
  uint16_t tDelta;        // Delta time (in milliseconds) from the previous polynomial
};

// Global Variables
Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

// Quantization scale for coefficients
const float QUANTIZATION_SCALE = 128.0;

// Quantize a coefficient to an 8-bit value
int8_t quantizeCoefficient(float value) {
  return (int8_t)(value * QUANTIZATION_SCALE);
}

// Dequantize an 8-bit coefficient back to a float
float dequantizeCoefficient(int8_t value) {
  return (float)value / QUANTIZATION_SCALE;
}

// Fit a polynomial and compute coefficients relative to the previous one
void fitDifferentialPolynomial(Polynomial& prevPoly, DataPoint* segment, int count, Polynomial& poly) {
  // Use the previous polynomial's end time as the new segment's start
  float tStart = prevPoly.tDelta / 1000.0;
  poly.tDelta = (uint16_t)((segment[count - 1].timestamp - tStart) * 1000);

  // Adjust timestamps for continuity
  float adjustedTimestamps[count];
  for (int i = 0; i < count; i++) {
    adjustedTimestamps[i] = segment[i].timestamp - tStart;
  }

  // Fit a 3rd-order polynomial using least squares (simplified for clarity)
  float coeffs[4] = {0};  // a3, a2, a1, a0
  // Polynomial fitting code here (implementation skipped for brevity)

  // Quantize coefficients and compute differentials
  poly.a3 = quantizeCoefficient(coeffs[0]) - prevPoly.a3;
  poly.a2 = quantizeCoefficient(coeffs[1]) - prevPoly.a2;
  poly.a1 = quantizeCoefficient(coeffs[2]) - prevPoly.a1;
  poly.a0 = quantizeCoefficient(coeffs[3]) - prevPoly.a0;
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
  float cumulativeCoeffs[4] = {0};       // Absolute coefficients: a3, a2, a1, a0
  float prevT = 0;

  for (int i = 0; i < storageCount; i++) {
    Polynomial currentPoly = storageBuffer[i];

    // Update cumulative coefficients
    cumulativeCoeffs[0] += dequantizeCoefficient(currentPoly.a3);
    cumulativeCoeffs[1] += dequantizeCoefficient(currentPoly.a2);
    cumulativeCoeffs[2] += dequantizeCoefficient(currentPoly.a1);
    cumulativeCoeffs[3] += dequantizeCoefficient(currentPoly.a0);

    // Compute the current polynomial's time range
    float currentT = prevT + currentPoly.tDelta / 1000.0;

    // Check if the timestamp falls within the polynomial's range
    if (timestamp <= currentT) {
      float t = timestamp - prevT;  // Normalize timestamp
      return cumulativeCoeffs[0] * t * t * t +
             cumulativeCoeffs[1] * t * t +
             cumulativeCoeffs[2] * t +
             cumulativeCoeffs[3];
    }

    prevT = currentT;
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
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tDelta=%d\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tDelta);
  }

  // Decompress a value
  float timestamp = 1.5;
  float value = decompressValue(timestamp);
  if (!isnan(value)) {
    Serial.printf("Decompressed value at %.2f: %.2f\n", timestamp, value);
  }
}

void loop() {}
