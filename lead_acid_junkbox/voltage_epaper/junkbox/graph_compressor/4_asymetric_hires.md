Switching to asymmetrical polynomials is a clever idea to fit higher-order polynomials into the same storage space. By asymmetrical, we mean coefficients may represent higher-order terms more compactly while preserving smaller terms with greater precision. This approach effectively reallocates the precision and resolution in a way that minimizes storage but retains accuracy.

Here's how we can improve the compressor, evaluation, and fitting functions:


---

1. Representation of Asymmetrical Polynomials

Modify the polynomial structure to allow coefficients of varying precision and quantization scales. The idea is to store smaller coefficients (e.g., lower-order terms) with higher precision and larger coefficients (e.g., higher-order terms) with lower precision.

struct Polynomial {
  int8_t a3;   // Higher-order term, lower precision
  int16_t a2;  // Medium precision
  int32_t a1;  // High precision
  float a0;    // Absolute value term, full precision
  uint16_t tDelta;  // Time delta
};

a3: Stored as an int8_t for compactness (low precision).

a2: Stored as an int16_t for medium precision.

a1: Stored as an int32_t for higher precision.

a0: Stored as a float to maintain absolute precision for the offset term.



---

2. Updated Fitting Strategy

Adjust the fitting strategy to:

Fit higher-order terms first but reduce their precision using quantization tailored to their ranges.

Use asymmetrical weights for terms to minimize error in the presence of quantization.


class AsymmetricalPolynomialFitting {
private:
  float quantizationScaleA3;  // Quantization for higher-order term
  float quantizationScaleA2;  // Quantization for medium-order term
  float quantizationScaleA1;  // Quantization for lower-order term

  // Quantization helpers
  int8_t quantizeA3(float value) { return (int8_t)(value * quantizationScaleA3); }
  int16_t quantizeA2(float value) { return (int16_t)(value * quantizationScaleA2); }
  int32_t quantizeA1(float value) { return (int32_t)(value * quantizationScaleA1); }
  float dequantizeA3(int8_t value) { return (float)value / quantizationScaleA3; }
  float dequantizeA2(int16_t value) { return (float)value / quantizationScaleA2; }
  float dequantizeA1(int32_t value) { return (float)value / quantizationScaleA1; }

public:
  AsymmetricalPolynomialFitting(float qScaleA3, float qScaleA2, float qScaleA1)
      : quantizationScaleA3(qScaleA3), quantizationScaleA2(qScaleA2), quantizationScaleA1(qScaleA1) {}

  void fit(DataPoint* data, int count, Polynomial& polynomial) {
    if (count <= 2) {
      polynomial.a3 = 0;
      polynomial.a2 = 0;
      polynomial.a1 = 0;
      polynomial.a0 = data[0].value;
      polynomial.tDelta = (uint16_t)((data[count - 1].timestamp - data[0].timestamp) * 1000);
      return;
    }

    // Use a least-squares approach to compute the polynomial
    float sumX[4] = {0}, sumY = 0, sumXY[4] = {0};
    for (int i = 0; i < count; i++) {
      float t = data[i].timestamp - data[0].timestamp;
      float v = data[i].value;
      float t2 = t * t;
      float t3 = t2 * t;

      sumX[0] += t3 * t2;
      sumX[1] += t3;
      sumX[2] += t2;
      sumX[3] += t;

      sumY += v;
      sumXY[0] += t3 * v;
      sumXY[1] += t2 * v;
      sumXY[2] += t * v;
    }

    // Fit coefficients using least-squares
    float a3 = sumXY[0] / sumX[0];
    float a2 = sumXY[1] / sumX[1];
    float a1 = sumXY[2] / sumX[2];
    float a0 = sumY / count;

    // Quantize coefficients
    polynomial.a3 = quantizeA3(a3);
    polynomial.a2 = quantizeA2(a2);
    polynomial.a1 = quantizeA1(a1);
    polynomial.a0 = a0;
    polynomial.tDelta = (uint16_t)((data[count - 1].timestamp - data[0].timestamp) * 1000);
  }
};


---

3. Updated Compressor

Integrate the asymmetrical fitting strategy into the compressor, using the AsymmetricalPolynomialFitting class:

void compressWithAsymmetricalFitting(DataPoint* buffer, int bufferCount, CompressionSettings settings) {
  if (bufferCount == 0) return;

  AsymmetricalPolynomialFitting fitting(0.05, 0.01, 0.001);  // Configure quantization scales
  int start = 0;

  while (start < bufferCount) {
    if (storageCount >= MAX_STORAGE) break;

    int segmentSize = 2;
    float maxResidual = 0;

    while (segmentSize <= settings.maxSegmentSize && start + segmentSize <= bufferCount) {
      Polynomial poly;
      fitting.fit(&buffer[start], segmentSize, poly);

      // Evaluate residual error
      maxResidual = computeResidualError(&buffer[start], segmentSize, poly);

      if (maxResidual > settings.maxError) break;

      segmentSize++;
    }

    segmentSize--;
    Polynomial poly;
    fitting.fit(&buffer[start], segmentSize, poly);
    storageBuffer[storageCount++] = poly;
    start += segmentSize;
  }
}


---

4. Updated Evaluation Function

Update the evaluation function to account for asymmetrical quantization:

float evaluatePolynomial(const Polynomial& poly, float t) {
  float tNorm = t / 1000.0;  // Convert ms to seconds
  return dequantizeA3(poly.a3) * pow(tNorm, 3) +
         dequantizeA2(poly.a2) * pow(tNorm, 2) +
         dequantizeA1(poly.a1) * tNorm +
         poly.a0;
}


---

5. Testing Routine

Add a test routine to evaluate the compression:

void testCompression() {
  for (int i = 0; i < tempBufferCount; i++) {
    float t = temporaryBuffer[i].timestamp;
    float originalValue = temporaryBuffer[i].value;

    // Find corresponding polynomial
    float decompressedValue = 0;
    for (int j = 0; j < storageCount; j++) {
      if (t <= storageBuffer[j].tDelta) {
        decompressedValue = evaluatePolynomial(storageBuffer[j], t);
        break;
      }
    }

    // Compare values
    Serial.print("Original: ");
    Serial.print(originalValue);
    Serial.print(", Decompressed: ");
    Serial.println(decompressedValue);
  }
}


---

Advantages of Asymmetrical Polynomials

1. Higher storage efficiency while maintaining fidelity.


2. Improved handling of higher-order terms due to focused quantization.


3. Smooth transitions between segments by carefully controlling residual errors.



Let me know how this works for your project or if further refinements are needed!

