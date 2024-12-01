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
#define MAX_STORAGE 256
const float QUANTIZATION_SCALE = 128.0;

// Buffers
DataPoint temporaryBuffer[MAX_TEMPORARY];
int tempBufferCount = 0;

Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

// Helper Functions
int8_t quantizeCoefficient(float value, float scale = QUANTIZATION_SCALE) {
  return (int8_t)round(value * scale);
}

float dequantizeCoefficient(int8_t value, float scale = QUANTIZATION_SCALE) {
  return (float)value / scale;
}

// Abstract base class for fitting strategies
class FittingStrategy {
public:
  virtual void fit(DataPoint* data, int count, float* coeffs) = 0;
};
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
//    Serial.print("residual:");
//    Serial.println(maxError);
    return maxError;
  }

/*
class AdaptiveFitting : public FittingStrategy {
public:
 CompressionSettings settings;
 
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Initialize sums for least squares computation
    float sumX[5] = {0}, sumY = 0, sumXY[5] = {0};
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
      sumX[4] += t;

      sumY += v;
      sumXY[0] += t3 * v;
      sumXY[1] += t2 * v;
      sumXY[2] += t * v;
      sumXY[3] += v;
    }

    // Dynamically decide order based on residuals
    for (int order = 3; order >= 0; order--) {
      coeffs[0] = (order >= 3) ? sumXY[0] / sumX[0] : 0;
      coeffs[1] = (order >= 2) ? sumXY[1] / sumX[1] : 0;
      coeffs[2] = (order >= 1) ? sumXY[2] / sumX[2] : 0;
      coeffs[3] = sumXY[3] / sumX[3];

      float residual = computeResidualError(data, count, coeffs);
      if (residual <= settings.maxError) break;  // Use the lowest order that fits
    }
  }
};

*/
/*
class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0;
      return;
    }

    // Normalize timestamps to start from 0 for numerical stability
    float t0 = data[0].timestamp;
    float normTimestamps[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      normTimestamps[i] = data[i].timestamp - t0;
    }

    // Initialize coefficients
    float sumT[4] = {0}, sumV = 0, sumTV[4] = {0};
    for (int i = 0; i < count; i++) {
      float t = normTimestamps[i];
      float v = data[i].value;

      sumT[0] += pow(t, 6);  // t^6 for higher order fitting
      sumT[1] += pow(t, 4);  // t^4
      sumT[2] += pow(t, 2);  // t^2
      sumT[3] += 1;          // t^0

      sumV += v;
      sumTV[0] += pow(t, 3) * v;
      sumTV[1] += pow(t, 2) * v;
      sumTV[2] += t * v;
      sumTV[3] += v;
    }

    // Solve for coefficients using weighted least squares
    coeffs[0] = sumTV[0] / sumT[0];  // a3
    coeffs[1] = sumTV[1] / sumT[1];  // a2
    coeffs[2] = sumTV[2] / sumT[2];  // a1
    coeffs[3] = sumTV[3] / sumT[3];  // a0

    // Iterative refinement loop
    for (int iter = 0; iter < 3; iter++) {
      float residuals[MAX_TEMPORARY];
      for (int i = 0; i < count; i++) {
        float t = normTimestamps[i];
        float fittedValue = coeffs[0] * pow(t, 3) + coeffs[1] * pow(t, 2) + coeffs[2] * t + coeffs[3];
        residuals[i] = data[i].value - fittedValue;
      }

      // Adjust coefficients based on residuals
      for (int i = 0; i < count; i++) {
        float t = normTimestamps[i];
        coeffs[0] += 0.01 * residuals[i] * pow(t, 3);
        coeffs[1] += 0.01 * residuals[i] * pow(t, 2);
        coeffs[2] += 0.01 * residuals[i] * t;
        coeffs[3] += 0.01 * residuals[i];
      }
    }
  }
};

*/

class EnhancedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count < 4) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Extended normalization strategy
    float t0 = data[0].timestamp;
    float tMax = data[count-1].timestamp;
    float timeSpan = tMax - t0;

    if (timeSpan == 0) {
      std::fill(coeffs, coeffs + 4, 0.0f);
      return;
    }

    // Compute end-point weights for bias reduction
    float startWeight = computeEndPointWeight(data, count, true);
    float endWeight = computeEndPointWeight(data, count, false);

    // Adaptive matrix construction
    float A[4][4] = {0};
    float B[4] = {0};

    for (int i = 0; i < count; ++i) {
      float t = (data[i].timestamp - t0) / timeSpan;
      float v = data[i].value;

      // Adaptive weighting strategy
      float positionWeight = 1.0f;
      if (i == 0) positionWeight *= startWeight;
      if (i == count-1) positionWeight *= endWeight;

      // Inverse variance weighting
      float varianceWeight = 1.0f / (1.0f + std::abs(v - computeMean(data, count)));
      float weight = positionWeight * varianceWeight;

      A[0][0] += weight * pow(t, 6);
      A[0][1] += weight * pow(t, 5);
      A[0][2] += weight * pow(t, 4);
      A[0][3] += weight * pow(t, 3);

      A[1][0] += weight * pow(t, 5);
      A[1][1] += weight * pow(t, 4);
      A[1][2] += weight * pow(t, 3);
      A[1][3] += weight * pow(t, 2);

      A[2][0] += weight * pow(t, 4);
      A[2][1] += weight * pow(t, 3);
      A[2][2] += weight * pow(t, 2);
      A[2][3] += weight * t;

      A[3][0] += weight * pow(t, 3);
      A[3][1] += weight * pow(t, 2);
      A[3][2] += weight * t;
      A[3][3] += weight;

      B[0] += weight * v * pow(t, 3);
      B[1] += weight * v * pow(t, 2);
      B[2] += weight * v * t;
      B[3] += weight * v;
    }

    // Gauss-Jordan elimination with improved stability
    for (int i = 0; i < 4; ++i) {
      int maxRow = i;
      for (int k = i + 1; k < 4; ++k) {
        if (std::abs(A[k][i]) > std::abs(A[maxRow][i])) {
          maxRow = k;
        }
      }

      std::swap(A[i], A[maxRow]);
      std::swap(B[i], B[maxRow]);

      float diag = A[i][i];
      if (std::abs(diag) < 1e-10) {
        coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0.0f;
        return;
      }

      for (int j = i; j < 4; ++j) {
        A[i][j] /= diag;
      }
      B[i] /= diag;

      for (int k = 0; k < 4; ++k) {
        if (k != i) {
          float factor = A[k][i];
          for (int j = i; j < 4; ++j) {
            A[k][j] -= factor * A[i][j];
          }
          B[k] -= factor * B[i];
        }
      }
    }

    // Rescale and apply end-point correction
    coeffs[0] = B[0] / pow(timeSpan, 3);
    coeffs[1] = B[1] / pow(timeSpan, 2);
    coeffs[2] = B[2] / timeSpan;
    coeffs[3] = B[3];

    // Advanced refinement with end-point sensitivity
    advancedRefinement(data, count, coeffs, t0, timeSpan);
  }

private:
  // Compute end-point weight to reduce bias
  float computeEndPointWeight(DataPoint* data, int count, bool isStart) {
    if (count <= 2) return 1.0f;

    int compareIndex = isStart ? 1 : count - 2;
    float endpointValue = isStart ? data[0].value : data[count-1].value;
    float compareValue = data[compareIndex].value;
    
    // Compute local slope and curvature
    float slopeDiff = std::abs(endpointValue - compareValue);
    float meanValue = computeMean(data, count);
    
    // More weight for points close to mean or with less deviation
    float weight = 1.0f / (1.0f + slopeDiff / meanValue);
    
    // Ensure weight is between 0.5 and 1.5
    return std::max(0.5f, std::min(1.5f, weight));
  }

  // Compute mean value
  float computeMean(DataPoint* data, int count) {
    float sum = 0;
    for (int i = 0; i < count; ++i) {
      sum += data[i].value;
    }
    return sum / count;
  }

  // Advanced refinement with end-point sensitivity
  void advancedRefinement(DataPoint* data, int count, float* coeffs, 
                          float t0, float timeSpan) {
    const int maxIter = 5;
    for (int iter = 0; iter < maxIter; ++iter) {
      float totalError = 0;
      std::vector<float> errors(count, 0);

      // Compute residuals with end-point emphasis
      for (int i = 0; i < count; ++i) {
        float t = (data[i].timestamp - t0) / timeSpan;
        float predicted = coeffs[0] * pow(t, 3) + 
                          coeffs[1] * pow(t, 2) + 
                          coeffs[2] * t + 
                          coeffs[3];
        
        // End-point error amplification
        float endPointMultiplier = 1.0f;
        if (i == 0 || i == count-1) {
          endPointMultiplier = 2.0f;  // More weight on end points
        }

        errors[i] = endPointMultiplier * (data[i].value - predicted);
        totalError += std::abs(errors[i]);
      }

      // Break if error is very small
      if (totalError < 1e-6) break;

      // Adaptive coefficient adjustment
      for (int i = 0; i < count; ++i) {
        float t = (data[i].timestamp - t0) / timeSpan;
        
        // Decreasing learning rate with iterations
        float learningRate = 0.02f / (iter + 1);
        
        // More aggressive adjustment for end points
        float endPointMultiplier = (i == 0 || i == count-1) ? 2.0f : 1.0f;

        coeffs[0] += endPointMultiplier * learningRate * errors[i] * pow(t, 3);
        coeffs[1] += endPointMultiplier * learningRate * errors[i] * pow(t, 2);
        coeffs[2] += endPointMultiplier * learningRate * errors[i] * t;
        coeffs[3] += endPointMultiplier * learningRate * errors[i];
      }
    }
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
    Serial.print("segmentError:");
    Serial.println(maxError);
    return maxError;
  }


/*
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

  // Store the absolute timestamp for the first polynomial
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
    Polynomial nextPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, nextPoly);

    // Compute the relative time delta
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;
    nextPoly.tDelta = (uint16_t)((tEnd - tStart) * 1000);

    storageBuffer[storageCount++] = nextPoly;
    start += segmentSize;
  }
}
*/

/*
void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Track the end of the previous segment

  // Fit the first polynomial (absolute coefficients)
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  float coeffs[4] = {0};
  fittingStrategy->fit(&buffer[start], bufferCount, coeffs);
  firstPoly.a3 = quantizeCoefficient(coeffs[0]);
  firstPoly.a2 = quantizeCoefficient(coeffs[1]);
  firstPoly.a1 = quantizeCoefficient(coeffs[2]);
  firstPoly.a0 = quantizeCoefficient(coeffs[3]);

  // Store the absolute timestamp of the first data point
  firstPoly.tDelta = 0;  // First polynomial uses absolute time as reference
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
    Polynomial nextPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, nextPoly);

    // Compute the relative time delta
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    // Store time delta in milliseconds, relative to the previous end timestamp
    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Update previous end timestamp
    previousEndTimestamp = tEnd;

    // Store the polynomial
    storageBuffer[storageCount++] = nextPoly;

    // Move to the next segment
    start += segmentSize;
  }
}
*/
/*
void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Start with the absolute timestamp
  float previousEndValue = buffer[0].value;          // Start with the absolute value

  // First polynomial
  Polynomial firstPoly = {0, 0, 0, 0, 0};
  float coeffs[4] = {0};
  fittingStrategy->fit(&buffer[start], bufferCount, coeffs);

  firstPoly.a3 = quantizeCoefficient(coeffs[0]);
  firstPoly.a2 = quantizeCoefficient(coeffs[1]);
  firstPoly.a1 = quantizeCoefficient(coeffs[2]);
  firstPoly.a0 = quantizeCoefficient(coeffs[3]);

  firstPoly.tDelta = 0;  // First polynomial uses absolute timestamp
  storageBuffer[storageCount++] = firstPoly;

  // Fit subsequent polynomials
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;
    float maxResidual = 0;

    // Fit and evaluate segments to find the optimal fit
    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;

    // Create a polynomial for the current segment
    Polynomial nextPoly = {0, 0, 0, 0, 0};
    fitDifferentialPolynomial(storageBuffer[storageCount - 1], &buffer[start], segmentSize, nextPoly);

    // Compute the relative time delta
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Re-evaluate the reconstructed value at the segment's end
    float reconstructedEndValue = previousEndValue +
                                  (dequantizeCoefficient(nextPoly.a3) * pow(tEnd - previousEndTimestamp, 3)) +
                                  (dequantizeCoefficient(nextPoly.a2) * pow(tEnd - previousEndTimestamp, 2)) +
                                  (dequantizeCoefficient(nextPoly.a1) * (tEnd - previousEndTimestamp)) +
                                  dequantizeCoefficient(nextPoly.a0);

    // Adjust coefficients to minimize residual error
    float errorCorrection = buffer[start + segmentSize - 1].value - reconstructedEndValue;
    nextPoly.a0 = quantizeCoefficient(dequantizeCoefficient(nextPoly.a0) + errorCorrection);

    // Update the base values for the next segment
    previousEndTimestamp = tEnd;
    previousEndValue = reconstructedEndValue + errorCorrection;

    // Store the polynomial
    storageBuffer[storageCount++] = nextPoly;

    // Move to the next segment
    start += segmentSize;
  }
}

*/
void compress(DataPoint* buffer, int bufferCount) {
  if (bufferCount == 0) return;

  int start = 0;
  float previousEndTimestamp = buffer[0].timestamp;  // Start with the absolute timestamp
  float previousEndValue = buffer[0].value;          // Start with the absolute value

  // Compression loop
  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

//    int segmentSize = 2;  // Start with a minimal segment size
//    int segmentSize = 3;  // Start with semi segment size
//    int segmentSize = 4;  // Start with semi segment size

//    int segmentSize = settings.maxSegmentSize/2;  // Start with semi segment size

    int segmentSize = settings.maxSegmentSize;  // Start with a maximal segment size

    float maxResidual = 0;

    Polynomial nextPoly = {0, 0, 0, 0, 0};
    float coeffs[4] = {0};

    if (bufferCount-start <segmentSize*2) {
      segmentSize = (bufferCount-start)/2; // equalize last two segments
    }

    if (bufferCount-start <segmentSize) { // trim any excess data
      segmentSize = bufferCount-start;
      }

    // Try increasing segment sizes to find the best fit
    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      fittingStrategy->fit(&buffer[start], segmentSize, coeffs);
      maxResidual = computeResidualError(&buffer[start], segmentSize, coeffs);

    if ( (((buffer[start + segmentSize - 1].timestamp) - previousEndTimestamp) * 1000) > 65535) segmentSize--;
    if (segmentSize < 4) break;
    if (maxResidual < settings.maxError) break;
    segmentSize--;
    if (segmentSize < 4) break;
    
    }

//    segmentSize--;  // Step back to the last valid size

    // Fit the polynomial for this segment
    fittingStrategy->fit(&buffer[start], segmentSize, coeffs);

    // Quantize the coefficients
    nextPoly.a3 = quantizeCoefficient(coeffs[0]);
    nextPoly.a2 = quantizeCoefficient(coeffs[1]);
    nextPoly.a1 = quantizeCoefficient(coeffs[2]);
    nextPoly.a0 = quantizeCoefficient(coeffs[3]);

    // Compute time delta and correct residuals
    float tStart = buffer[start].timestamp;
    float tEnd = buffer[start + segmentSize - 1].timestamp;

    nextPoly.tDelta = (uint16_t)((tEnd - previousEndTimestamp) * 1000);

    // Correct for residual error
    float reconstructedEndValue = previousEndValue +
                                  (dequantizeCoefficient(nextPoly.a3) * pow(tEnd - previousEndTimestamp, 3)) +
                                  (dequantizeCoefficient(nextPoly.a2) * pow(tEnd - previousEndTimestamp, 2)) +
                                  (dequantizeCoefficient(nextPoly.a1) * (tEnd - previousEndTimestamp)) +
                                  dequantizeCoefficient(nextPoly.a0);

    float errorCorrection = buffer[start + segmentSize - 1].value - reconstructedEndValue;
    nextPoly.a0 = quantizeCoefficient(dequantizeCoefficient(nextPoly.a0) + errorCorrection);

    // Update the base values for the next segment
    previousEndTimestamp = tEnd;
    previousEndValue = reconstructedEndValue + errorCorrection;

    // Store the polynomial
    storageBuffer[storageCount++] = nextPoly;

    // Move to the next segment
    start += segmentSize;
  }
}

}; // class compressor


/*
float evaluateCompressedData(Polynomial* storageBuffer, int storageCount, float timestamp) {
  float baseTimestamp = 0.0; // Absolute timestamp for the first polynomial
  float baseValue = 0.0;     // Base value at the start of the current polynomial

  for (int i = 0; i < storageCount; i++) {
    Polynomial poly = storageBuffer[i];
    float tDelta = poly.tDelta / 1000.0; // Convert milliseconds to seconds

    if (timestamp >= baseTimestamp && timestamp <= baseTimestamp + tDelta) {
      // Timestamp falls within this polynomial's range
      float relativeTime = timestamp - baseTimestamp;

      // Dequantize coefficients
      float a3 = dequantizeCoefficient(poly.a3);
      float a2 = dequantizeCoefficient(poly.a2);
      float a1 = dequantizeCoefficient(poly.a1);
      float a0 = dequantizeCoefficient(poly.a0);

      // Evaluate the cubic polynomial relative to the base value
      return baseValue +
             (a3 * relativeTime * relativeTime * relativeTime) +
             (a2 * relativeTime * relativeTime) +
             (a1 * relativeTime) +
             (a0);
    }

    // Update base timestamp and base value for the next segment
    baseTimestamp += tDelta;

    // Evaluate cumulative baseValue for continuity
    baseValue += (dequantizeCoefficient(poly.a3) * tDelta * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a2) * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a1) * tDelta) +
                 (dequantizeCoefficient(poly.a0));
  }

  // Return NAN if the timestamp is out of range
  return NAN;
}
*/


float evaluateCompressedData(Polynomial* storageBuffer, int storageCount, float timestamp) {
  float baseTimestamp = 0;  // Start with the absolute timestamp
  float baseValue = 0.0;

  for (int i = 0; i < storageCount; i++) {
    Polynomial poly = storageBuffer[i];
    float tDelta = poly.tDelta / 1000.0;

    if (timestamp >= baseTimestamp && timestamp <= baseTimestamp + tDelta) {
      // Timestamp falls within this polynomial's range
      float relativeTime = timestamp - baseTimestamp;

      // Dequantize coefficients
      float a3 = dequantizeCoefficient(poly.a3);
      float a2 = dequantizeCoefficient(poly.a2);
      float a1 = dequantizeCoefficient(poly.a1);
      float a0 = dequantizeCoefficient(poly.a0);

      // Evaluate the polynomial
      return baseValue +
             (a3 * relativeTime * relativeTime * relativeTime) +
             (a2 * relativeTime * relativeTime) +
             (a1 * relativeTime) +
             (a0);
    }

    // Update base timestamp and base value for the next segment
    baseTimestamp += tDelta;
    baseValue += (dequantizeCoefficient(poly.a3) * tDelta * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a2) * tDelta * tDelta) +
                 (dequantizeCoefficient(poly.a1) * tDelta) +
                 (dequantizeCoefficient(poly.a0));
  }

  return NAN;  // Timestamp out of range
}

void testDecompressionAccuracy() {
  // Generate synthetic test data
  for (int i = 0; i < MAX_TEMPORARY; i++) {
    temporaryBuffer[i].timestamp = i * 0.1f;    // Example: 0.1s intervals
    temporaryBuffer[i].value = sin(i * 0.05f);   // Example: Sine wave
    Serial.printf("Timestamp: %.3f, Value: %.3f\n", temporaryBuffer[i].timestamp, temporaryBuffer[i].value);
  }
  tempBufferCount = MAX_TEMPORARY-1;  // Example: 100 data points

  // Compression
  CompressionSettings settings = {0.01f, 32};  // Max error: 0.01, max segment size: 10
  //LeastSquaresFitting fittingStrategy;
  //AdaptiveFitting fittingStrategy;
  EnhancedFitting fittingStrategy;
  
  Compressor compressor(settings, &fittingStrategy);
  compressor.compress(temporaryBuffer, tempBufferCount);

//  storageCount = 0 ; 
  Serial.println(storageCount);
  for (int i = 0; i < storageCount; i++) {
    Polynomial& poly = storageBuffer[i];
    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tDelta=%d\n",
                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tDelta);
 }

  
  // Validate decompression
  bool allPassed = true;
  for (int i = 0; i < tempBufferCount; i++) {
    float originalTimestamp = temporaryBuffer[i].timestamp;
    float originalValue = temporaryBuffer[i].value;

    float decompressedValue = evaluateCompressedData(storageBuffer, storageCount, originalTimestamp);
    float error = fabs(originalValue - decompressedValue);

     //Serial.printf("PASSED: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
      //              originalTimestamp, originalValue, decompressedValue, error);

    if (error > settings.maxError) {
      Serial.printf("ERROR: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
                    originalTimestamp, originalValue, decompressedValue, error);
      allPassed = false;
    } else {
      Serial.printf("PASSED: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
                    originalTimestamp, originalValue, decompressedValue, error);
    }
  }

  if (allPassed) {
    Serial.println("All decompression tests passed!");
  } else {
    Serial.println("Decompression test failed!");
  }
}


// Example usage
void setup() {
  Serial.begin(115200);

  // Add data points to the temporary buffer
//  for (int i = 0; i < MAX_TEMPORARY; i++) {
//    temporaryBuffer[i] = {i * 0.5, sin(i * 0.1)};
//    Serial.printf("Timestamp: %.3f, Value: %.3f\n", temporaryBuffer[i].timestamp, temporaryBuffer[i].value);
//    tempBufferCount++;
//  }

  // Configure compression
//  CompressionSettings settings = {0.01, 10};  // Max error and max segment size
//  LeastSquaresFitting fittingStrategy;
//  Compressor compressor(settings, &fittingStrategy);

  // Compress the temporary buffer
  //compressor.compress(temporaryBuffer, tempBufferCount);

  // Print compressed storage
//  storageCount = MAX_STORAGE;
//  for (int i = 0; i < storageCount; i++) {
//    Polynomial& poly = storageBuffer[i];
//    Serial.printf("Poly %d: a3=%d, a2=%d, a1=%d, a0=%d, tDelta=%d\n",
//                  i, poly.a3, poly.a2, poly.a1, poly.a0, poly.tDelta);
// }

  testDecompressionAccuracy();
//  testdecompression();
}

void graph_data(){
    for (int i = 0; i < tempBufferCount; i++) {
    float originalTimestamp = temporaryBuffer[i].timestamp;
    float originalValue = temporaryBuffer[i].value;

    float decompressedValue = evaluateCompressedData(storageBuffer, storageCount, originalTimestamp);
    float error = fabs(originalValue - decompressedValue);

     //Serial.printf("PASSED: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
      //              originalTimestamp, originalValue, decompressedValue, error);

//      Serial.printf("ERROR: Timestamp %.2f, Original %.3f, Decompressed %.3f, Error %.3f\n",
//                    originalTimestamp, originalValue, decompressedValue, error);
      Serial.printf("Original:%.3f,Decompressed:%.3f,Error:%.3f\n",
                     originalValue, decompressedValue, error);

  }

}
void loop() {
//  graph_data();
  yield();
  delay(1000);
  }
