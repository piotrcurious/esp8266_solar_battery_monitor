#include <Arduino.h>

// Constants
#define MAX_TEMPORARY 64  // Maximum temporary buffer size
#define MAX_STORAGE 128   // Maximum storage size for compressed polynomials
#define COEFF_LIMIT 127   // 8-bit signed coefficient limit

// Data Buffers
struct DataPoint {
  float timestamp;
  float value;
};

DataPoint temporaryBuffer[MAX_TEMPORARY];
int tempBufferCount = 0;

struct Polynomial {
  int8_t a3, a2, a1, a0;  // 3rd-order polynomial coefficients (8-bit)
  float tStart, tEnd;     // Valid time range
};

Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

// Function to reset the temporary buffer
void resetTemporaryBuffer() {
  tempBufferCount = 0;
}

// Add a new data point to the temporary buffer
void addDataPoint(float timestamp, float value) {
  if (tempBufferCount < MAX_TEMPORARY) {
    temporaryBuffer[tempBufferCount++] = {timestamp, value};
  }
}

// Quantize a coefficient to fit within 8-bit limits
int8_t quantizeCoefficient(float coeff) {
  int quantized = round(coeff);
  if (quantized > COEFF_LIMIT) return COEFF_LIMIT;
  if (quantized < -COEFF_LIMIT) return -COEFF_LIMIT;
  return (int8_t)quantized;
}

// Fit a 3rd-order polynomial to the data segment
void fitPolynomial(DataPoint* segment, int count, Polynomial& poly) {
  // Use a basic least-squares fitting approach
  float sumX[4] = {0}, sumY = 0, sumXY[4] = {0}, sumX2Y = 0;
  for (int i = 0; i < count; i++) {
    float t = segment[i].timestamp;
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

  // Solve linear system (coefficients a3, a2, a1, a0)
  float det = sumX[0] * (sumX[1] * sumX[2] - sumX[3] * sumX[3]) -
              sumX[1] * (sumX[1] * sumX[2] - sumX[3] * sumY) +
              sumX[2] * (sumX[3] * sumY - sumX[1] * sumXY[1]);

  if (det == 0) return;  // Singular matrix, fallback needed

  poly.a3 = quantizeCoefficient(sumXY[0] / det);
  poly.a2 = quantizeCoefficient(sumXY[1] / det);
  poly.a1 = quantizeCoefficient(sumXY[2] / det);
  poly.a0 = quantizeCoefficient(sumX2Y / det);

  poly.tStart = segment[0].timestamp;
  poly.tEnd = segment[count - 1].timestamp;
}

// Compress the temporary buffer into the storage buffer
void compressTemporaryBuffer() {
  int segmentSize = 8;  // Number of points per segment (tunable)
  for (int i = 0; i < tempBufferCount; i += segmentSize) {
    if (storageCount >= MAX_STORAGE) break;

    int count = min(segmentSize, tempBufferCount - i);
    Polynomial poly;
    fitPolynomial(&temporaryBuffer[i], count, poly);
    storageBuffer[storageCount++] = poly;
  }

  resetTemporaryBuffer();
}

// Retrieve a value from the storage buffer given a timestamp
float decompressValue(float timestamp) {
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    if (timestamp >= poly.tStart && timestamp <= poly.tEnd) {
      float t = timestamp - poly.tStart;  // Normalize to segment start
      return poly.a3 * t * t * t +
             poly.a2 * t * t +
             poly.a1 * t +
             poly.a0;
    }
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
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tStart=%.2f, tEnd=%.2f\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tStart, poly.tEnd);
  }
}

void loop() {
  // Example decompression
  float timestamp = 1.5;
  float value = decompressValue(timestamp);
  if (!isnan(value)) {
    Serial.printf("Decompressed value at %.2f: %.2f\n", timestamp, value);
  }
  delay(1000);
}
