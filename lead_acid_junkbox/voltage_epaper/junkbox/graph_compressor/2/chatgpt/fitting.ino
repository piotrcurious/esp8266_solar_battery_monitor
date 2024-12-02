#include <limits>

class AdvancedQuantizedFitting : public FittingStrategy {
public:
  void fit(DataPoint* data, int count, float* coeffs) override {
    if (count <= 2) {
      coeffs[0] = coeffs[1] = coeffs[2] = 0;
      coeffs[3] = data[0].value;
      return;
    }

    // Normalize timestamps to avoid numerical instability
    float t0 = data[0].timestamp;
    float normalizedT[MAX_TEMPORARY];
    for (int i = 0; i < count; i++) {
      normalizedT[i] = data[i].timestamp - t0;
    }

    // Step 1: Initialize Vandermonde matrix for polynomial fitting
    float vandermonde[4][4] = {0};
    float rhs[4] = {0};

    for (int i = 0; i < count; i++) {
      float t = normalizedT[i];
      float v = data[i].value;

      float t2 = t * t;
      float t3 = t2 * t;

      // Fill Vandermonde matrix
      vandermonde[0][0] += t3 * t;
      vandermonde[0][1] += t3;
      vandermonde[0][2] += t2;
      vandermonde[0][3] += t;

      vandermonde[1][1] += t2;
      vandermonde[1][2] += t;
      vandermonde[1][3] += 1;

      // Fill RHS vector
      rhs[0] += t3 * v;
      rhs[1] += t2 * v;
      rhs[2] += t * v;
      rhs[3] += v;
    }

    // Symmetric matrix completion
    vandermonde[1][0] = vandermonde[0][1];
    vandermonde[2][0] = vandermonde[0][2];
    vandermonde[2][1] = vandermonde[1][2];
    vandermonde[3][0] = vandermonde[0][3];
    vandermonde[3][1] = vandermonde[1][3];
    vandermonde[3][2] = vandermonde[2][3];

    // Step 2: Solve for floating-point coefficients (Gaussian elimination or matrix inversion)
    float floatCoeffs[4];
    solveLinearSystem(vandermonde, rhs, floatCoeffs);

    // Step 3: Quantize coefficients iteratively to minimize error
    float bestError = std::numeric_limits<float>::max();
    int8_t bestQuantizedCoeffs[4] = {0};
    for (int8_t q3 = -127; q3 <= 127; q3++) {
      for (int8_t q2 = -127; q2 <= 127; q2++) {
        for (int8_t q1 = -127; q1 <= 127; q1++) {
          for (int8_t q0 = -127; q0 <= 127; q0++) {
            float candidate[4] = {
              dequantizeCoefficient(q3),
              dequantizeCoefficient(q2),
              dequantizeCoefficient(q1),
              dequantizeCoefficient(q0),
            };

            float error = computeResidualError(data, count, candidate);
            if (error < bestError) {
              bestError = error;
              bestQuantizedCoeffs[0] = q3;
              bestQuantizedCoeffs[1] = q2;
              bestQuantizedCoeffs[2] = q1;
              bestQuantizedCoeffs[3] = q0;
            }
          }
        }
      }
    }

    // Step 4: Assign the best quantized coefficients to the output
    coeffs[0] = dequantizeCoefficient(bestQuantizedCoeffs[0]);
    coeffs[1] = dequantizeCoefficient(bestQuantizedCoeffs[1]);
    coeffs[2] = dequantizeCoefficient(bestQuantizedCoeffs[2]);
    coeffs[3] = dequantizeCoefficient(bestQuantizedCoeffs[3]);
  }

private:
  void solveLinearSystem(float matrix[4][4], float rhs[4], float* solution) {
    // Basic Gaussian elimination for 4x4 matrix
    for (int i = 0; i < 4; i++) {
      float pivot = matrix[i][i];
      for (int j = i; j < 4; j++) matrix[i][j] /= pivot;
      rhs[i] /= pivot;

      for (int k = i + 1; k < 4; k++) {
        float factor = matrix[k][i];
        for (int j = i; j < 4; j++) matrix[k][j] -= factor * matrix[i][j];
        rhs[k] -= factor * rhs[i];
      }
    }

    // Back substitution
    for (int i = 3; i >= 0; i--) {
      solution[i] = rhs[i];
      for (int j = i + 1; j < 4; j++) {
        solution[i] -= matrix[i][j] * solution[j];
      }
    }
  }
};
