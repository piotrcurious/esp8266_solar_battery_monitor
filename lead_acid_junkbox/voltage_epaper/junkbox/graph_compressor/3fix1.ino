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
int8_t quantizeCoefficient(float value, float scale = QUANTIZATION_SCALE) {
  return (int8_t)(value * scale);
}

float dequantizeCoefficient(int8_t value, float scale = QUANTIZATION_SCALE) {
  return (float)value / scale;
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
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Initialize sums for least squares computation
    float sumX[4] = {0}, sumY = 0, sumXY[4] = {0};
    for (int i = 0; i < count; i++) {
      float t = data[i].timestamp - data[0].timestamp;
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
