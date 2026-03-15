#include <Arduino.h>

#define MAX_TEMPORARY 512
#define MAX_STORAGE 256

struct DataPoint {
  float timestamp;
  float value;
};

DataPoint temporaryBuffer[MAX_TEMPORARY];
int tempBufferCount = 0;

struct Polynomial {
  float a3, a2, a1, a0;
  float tStart, tEnd;
};

Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

void addDataPoint(float timestamp, float value) {
  if (tempBufferCount < MAX_TEMPORARY) {
    temporaryBuffer[tempBufferCount++] = {timestamp, value};
  }
}

void fitPolynomial(DataPoint* segment, int count, float* coeffs) {
  if (count < 1) return;
  if (count < 4) {
      coeffs[0] = 0; coeffs[1] = 0; coeffs[2] = 0;
      float sumV = 0;
      for(int i=0; i<count; i++) sumV += segment[i].value;
      coeffs[3] = sumV/count;
      return;
  }
  double X[4][4] = {0};
  double Y[4] = {0};
  double t0 = segment[0].timestamp;
  for (int i = 0; i < count; i++) {
    double t = segment[i].timestamp - t0;
    double v = segment[i].value;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    X[0][0] += t6; X[0][1] += t5; X[0][2] += t4; X[0][3] += t3;
    X[1][0] += t5; X[1][1] += t4; X[1][2] += t3; X[1][3] += t2;
    X[2][0] += t4; X[2][1] += t3; X[2][2] += t2; X[2][3] += t;
    X[3][0] += t3; X[3][1] += t2; X[3][2] += t;  X[3][3] += 1;
    Y[0] += t3 * v; Y[1] += t2 * v; Y[2] += t * v; Y[3] += v;
  }
  for (int i = 0; i < 4; i++) {
    int pivot = i;
    for (int j = i + 1; j < 4; j++) {
      if (std::abs(X[j][i]) > std::abs(X[pivot][i])) pivot = j;
    }
    for (int j = i; j < 4; j++) std::swap(X[i][j], X[pivot][j]);
    std::swap(Y[i], Y[pivot]);
    if (std::abs(X[i][i]) > 1e-12) {
        for (int j = i + 1; j < 4; j++) {
          double factor = X[j][i] / X[i][i];
          Y[j] -= factor * Y[i];
          for (int k = i; k < 4; k++) X[j][k] -= factor * X[i][k];
        }
    }
  }
  double res[4] = {0};
  for (int i = 3; i >= 0; i--) {
    if (std::abs(X[i][i]) > 1e-12) {
        double sum = 0;
        for (int j = i + 1; j < 4; j++) sum += X[i][j] * res[j];
        res[i] = (Y[i] - sum) / X[i][i];
    }
  }
  for(int i=0; i<4; i++) coeffs[i] = (float)res[i];
}

float spatialErrorThreshold = 10.0;
float magnitudeErrorThreshold = 0.05;

void compress() {
  int start = 0;
  while (start < tempBufferCount && storageCount < MAX_STORAGE) {
    int end = start + 1;
    float bestCoeffs[4] = {0};
    bool found = false;
    while (end < tempBufferCount) {
      float currentCoeffs[4] = {0};
      fitPolynomial(&temporaryBuffer[start], end - start + 1, currentCoeffs);
      float maxRes = 0;
      for (int i = start; i <= end; i++) {
        float t = temporaryBuffer[i].timestamp - temporaryBuffer[start].timestamp;
        float pred = currentCoeffs[0]*t*t*t + currentCoeffs[1]*t*t + currentCoeffs[2]*t + currentCoeffs[3];
        float err = std::abs(pred - temporaryBuffer[i].value);
        if (err > maxRes) maxRes = err;
      }
      if (maxRes <= magnitudeErrorThreshold && (temporaryBuffer[end].timestamp - temporaryBuffer[start].timestamp) <= spatialErrorThreshold) {
        memcpy(bestCoeffs, currentCoeffs, sizeof(bestCoeffs));
        found = true;
        end++;
      } else {
        break;
      }
    }
    if (!found) {
        bestCoeffs[0]=0; bestCoeffs[1]=0; bestCoeffs[2]=0; bestCoeffs[3]=temporaryBuffer[start].value;
        end = start + 1;
    }
    Polynomial& p = storageBuffer[storageCount++];
    p.a3 = bestCoeffs[0]; p.a2 = bestCoeffs[1]; p.a1 = bestCoeffs[2]; p.a0 = bestCoeffs[3];
    p.tStart = temporaryBuffer[start].timestamp;
    p.tEnd = temporaryBuffer[end - 1].timestamp;
    start = end;
  }
}

void setup() {
    Serial.begin(115200);
    String line;
    while (Serial.available()) {
        line = Serial.readStringUntil('\n');
        if (line.length() == 0) continue;
        int commaIndex = line.indexOf(',');
        if (commaIndex == -1) continue;
        addDataPoint(line.substring(0, commaIndex).toFloat(), line.substring(commaIndex + 1).toFloat());
    }
    compress();
    Serial.print("SIZE:"); Serial.println(storageCount * sizeof(Polynomial));
    for (int i = 0; i < storageCount; i++) {
        Polynomial& p = storageBuffer[i];
        for (float t = p.tStart; t <= p.tEnd + 0.001; t += 0.1) {
            float dt = t - p.tStart;
            float v = p.a3*dt*dt*dt + p.a2*dt*dt + p.a1*dt + p.a0;
            Serial.print("RESULT:"); Serial.print(t); Serial.print(":"); Serial.println(v);
        }
    }
}
void loop() {}
