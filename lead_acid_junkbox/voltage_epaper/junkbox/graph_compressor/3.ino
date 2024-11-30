#include <math.h>

// Polynomial structure
struct Polynomial {
  int8_t a3, a2, a1, a0;  // Quantized differential coefficients
  uint16_t tDelta;        // Time delta (in milliseconds) from the previous polynomial
};

// Data point structure
struct DataPoint {
  float timestamp;
  float value;
};

// Compression settings
struct CompressionSettings {
  float maxError;       // Maximum allowed error
  int maxSegmentSize;   // Maximum number of data points per segment
};

// Constants
#define MAX_TEMPORARY 256
#define MAX_STORAGE 128
const float QUANTIZATION_SCALE = 128.0;

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

// Abstract base class for fitting strategies
class FittingStrategy {
public:
  virtual void fit(DataPoint* data, int count, float* coeffs) = 0;
};

// Least squares fitting strategy
class LeastSquaresFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
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

    // Calculate coefficients
    coeffs[0] = sumXY[0] / sumX[0];
    coeffs[1] = sumXY[1] / sumX[1];
    coeffs[2] = sumXY[2] / sumX[2];
    coeffs[3] = sumXY[3] / count;
  }
};

// Compressor class
class Compressor {
private:
  CompressionSettings settings;
  FittingStrategy* fittingStrategy;

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
    fittingStrategy->fit(adjustedSegment, count, coeffs);

    // Quantize coefficients and compute differentials
    poly.a3 = quantizeCoefficient(coeffs[0]) - prevPoly.a3;
    poly.a2 = quantizeCoefficient(coeffs[1]) - prevPoly.a2;
    poly.a1 = quantizeCoefficient(coeffs[2]) - prevPoly.a1;
    poly.a0 = quantizeCoefficient(coeffs[3]) - prevPoly.a0;
  }

  // Compute residual error for a segment
  float computeResidualError(DataPoint* data, int count, float* coeffs) {
    float maxError = 0;
    for (int i = 0; i < count; i++) {
      float t = data[i].timestamp - data[0].timestamp;
      float predictedValue = coeffs[0] * t * t * t +
                             coeffs[1] * t * t +
                             coeffs[2] * t +
                             coeffs[3];
      float error = fabs(predictedValue - data[i].value);
      if (error > maxError) maxError = error;
    }
    return maxError;
  }

public:
  Compressor(CompressionSettings settings, FittingStrategy* strategy)
      : settings(settings), fittingStrategy(strategy) {}

  void compress(DataPoint* buffer, int bufferCount) {
    if (bufferCount == 0) return;

    int start = 0;

    // Fit the first polynomial (absolute coefficients)
    Polynomial firstPoly = {0, 0, 0, 0, 0};
    float coeffs[4] = {0};
    fittingStrategy->fit(&buffer[start], bufferCount, coeffs);
    firstPoly.a3 = quantizeCoefficient(coeffs[0]);
    firstPoly.a2 = quantizeCoefficient(coeffs[1]);
    firstPoly.a1 = quantizeCoefficient(coeffs[2]);
    firstPoly.a0 = quantizeCoefficient(coeffs[3]);
    firstPoly.tDelta = (uint16_t)((buffer[start + bufferCount - 1].timestamp - buffer[start].timestamp) * 1000);
    storageBuffer[storageCount++] = firstPoly;

    // Fit subsequent polynomials
    while (start < bufferCount) {
      if (storageCount >= MAX_STORAGE) break;

      int segmentSize = 2;
      float maxResidual = 0;

      while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
        fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
        maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

        if (maxResidual > settings.maxError) break;

        segmentSize++;
      }

      segmentSize--;
      fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, storageBuffer[storageCount]);
      storageCount++;
      start += segmentSize;
    }
  }
};

// Example usage
void setup() {
  Serial.begin(115200);

  // Add data points to the temporary buffer
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    temporaryBuffer[i] = {i * 0.5, sin(i * 0.1)};
    tempBufferCount++;
  }

  // Configure compression
  CompressionSettings settings = {0.01, 16};  // Max error and max segment size
  LeastSquaresFitting fittingStrategy;
  Compressor compressor(settings, &fittingStrategy);

  // Compress the temporary buffer
  compressor.compress(temporaryBuffer, tempBufferCount);

  // Print compressed storage
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tDelta=%d\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tDelta);
  }
}

void loop() {}
