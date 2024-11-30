// Structure to hold polynomials
struct Polynomial {
  int8_t a3, a2, a1, a0;  // 8-bit differential coefficients
  uint16_t tDelta;        // Time delta (in milliseconds) from the previous polynomial
};

// Constants
#define MAX_TEMPORARY 256  // Maximum temporary buffer size
#define MAX_STORAGE 128    // Maximum compressed storage size
const float QUANTIZATION_SCALE = 128.0;

// Data point structure
struct DataPoint {
  float timestamp;
  float value;
};

// Buffers
DataPoint temporaryBuffer[MAX_TEMPORARY];
int tempBufferCount = 0;

Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

// Helper Functions
int8_t quantizeCoefficient(float value) {
  return (int8_t)(value * QUANTIZATION_SCALE);
}

float dequantizeCoefficient(int8_t value) {
  return (float)value / QUANTIZATION_SCALE;
}

// Polynomial fitting function (least squares approximation)
void fitPolynomial(DataPoint* data, int count, float coeffs[4]) {
  // Initialize sums for least squares computation
  float sumX[4] = {0}, sumY = 0, sumXY[4] = {0};
  
  for (int i = 0; i < count; i++) {
    float t = data[i].timestamp;
    float v = data[i].value;
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;

    sumX[0] += t4 * t;
    sumX[1] += t4;
    sumX[2] += t3;
    sumX[3] += t2;

    sumY += v;
    sumXY[0] += t3 * v;
    sumXY[1] += t2 * v;
    sumXY[2] += t * v;
    sumXY[3] += v;
  }

  // Calculate coefficients (simplified least squares solution for 3rd-order polynomial)
  coeffs[0] = sumXY[0] / sumX[0];
  coeffs[1] = sumXY[1] / sumX[1];
  coeffs[2] = sumXY[2] / sumX[2];
  coeffs[3] = sumXY[3] / count;
}

// Fit a differential polynomial based on previous coefficients
void fitDifferentialPolynomial(Polynomial& prevPoly, DataPoint* segment, int count, Polynomial& poly) {
  float tStart = prevPoly.tDelta / 1000.0;
  poly.tDelta = (uint16_t)((segment[count - 1].timestamp - tStart) * 1000);

  // Adjust timestamps for fitting
  DataPoint adjustedSegment[count];
  for (int i = 0; i < count; i++) {
    adjustedSegment[i].timestamp = segment[i].timestamp - tStart;
    adjustedSegment[i].value = segment[i].value;
  }

  // Fit a polynomial to the adjusted data
  float coeffs[4] = {0};
  fitPolynomial(adjustedSegment, count, coeffs);

  // Quantize coefficients and compute differentials
  poly.a3 = quantizeCoefficient(coeffs[0]) - prevPoly.a3;
  poly.a2 = quantizeCoefficient(coeffs[1]) - prevPoly.a2;
  poly.a1 = quantizeCoefficient(coeffs[2]) - prevPoly.a1;
  poly.a0 = quantizeCoefficient(coeffs[3]) - prevPoly.a0;
}

// Compress data into polynomials
void compressTemporaryBuffer() {
  if (tempBufferCount == 0) return;

  int start = 0;

  // Fit the first polynomial (absolute coefficients)
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  fitPolynomial(&temporaryBuffer[start], tempBufferCount, firstPoly);
  storageBuffer[storageCount++] = firstPoly;

  // Fit subsequent polynomials (differential encoding)
  while (start < tempBufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int end = min(start + 8, tempBufferCount); // Adjustable segment size for better fitting
    Polynomial currentPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &temporaryBuffer[start], end - start, currentPoly);

    storageBuffer[storageCount++] = currentPoly;
    start = end;
  }

  tempBufferCount = 0;  // Reset temporary buffer
}

// Decompress a value at a given timestamp
float decompressValue(float timestamp) {
  float cumulativeCoeffs[4] = {0};
  float prevT = 0;

  for (int i = 0; i < storageCount; i++) {
    Polynomial& currentPoly = storageBuffer[i];

    // Compute cumulative coefficients
    cumulativeCoeffs[0] += dequantizeCoefficient(currentPoly.a3);
    cumulativeCoeffs[1] += dequantizeCoefficient(currentPoly.a2);
    cumulativeCoeffs[2] += dequantizeCoefficient(currentPoly.a1);
    cumulativeCoeffs[3] += dequantizeCoefficient(currentPoly.a0);

    float currentT = prevT + currentPoly.tDelta / 1000.0;

    // Check if the timestamp is within this polynomial's range
    if (timestamp <= currentT) {
      float t = timestamp - prevT;  // Normalize timestamp
      return cumulativeCoeffs[0] * t * t * t +
             cumulativeCoeffs[1] * t * t +
             cumulativeCoeffs[2] * t +
             cumulativeCoeffs[3];
    }

    prevT = currentT;
  }

  return NAN;  // Timestamp is out of range
}

// Add a data point to the temporary buffer
void addDataPoint(float timestamp, float value) {
  if (tempBufferCount >= MAX_TEMPORARY) return;
  temporaryBuffer[tempBufferCount++] = {timestamp, value};
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

  // Print compressed storage
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
