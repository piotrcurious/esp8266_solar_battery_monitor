#include <Arduino.h>
#include <vector>
#define MAX_TEMPORARY 512
#define MAX_STORAGE 128
struct DataPoint { float t, v; };
struct Poly { float c[6]; float tStart, tEnd; int order; };
DataPoint tempBuf[MAX_TEMPORARY];
int tempCount = 0;
Poly storage[MAX_STORAGE];
int storageCount = 0;
void addPoint(float t, float v) { if(tempCount < MAX_TEMPORARY) tempBuf[tempCount++] = {t, v}; }
void solve(const std::vector<std::vector<double>>& A, const std::vector<double>& b, std::vector<double>& x) {
    int n = A.size(); std::vector<std::vector<double>> mat = A; std::vector<double> vec = b;
    for (int i = 0; i < n; i++) {
        int pivot = i; for (int j = i + 1; j < n; j++) if (std::abs(mat[j][i]) > std::abs(mat[pivot][i])) pivot = j;
        std::swap(mat[i], mat[pivot]); std::swap(vec[i], vec[pivot]);
        if (std::abs(mat[i][i]) > 1e-18) for (int j = i + 1; j < n; j++) { double f = mat[j][i] / mat[i][i]; vec[j] -= f * vec[i]; for (int k = i; k < n; k++) mat[j][k] -= f * mat[i][k]; }
    }
    x.assign(n, 0); for (int i = n - 1; i >= 0; i--) if (std::abs(mat[i][i]) > 1e-18) { double s = 0; for (int j = i + 1; j < n; j++) s += mat[i][j] * x[j]; x[i] = (vec[i] - s) / mat[i][i]; }
}
void fit(DataPoint* data, int n, int order, Poly& p) {
    int m = order + 1; if (n < m) { memset(p.c, 0, sizeof(p.c)); double s = 0; for(int i=0; i<n; i++) s += data[i].v; p.c[0] = s/n; p.tStart = data[0].t; p.tEnd = data[n-1].t; p.order = order; return; }
    std::vector<std::vector<double>> A(m, std::vector<double>(m, 0.0)); std::vector<double> B(m, 0.0); double t0 = data[0].t;
    for (int i = 0; i < n; i++) {
        double t = data[i].t - t0, v = data[i].v; std::vector<double> powers(2 * order + 1); powers[0] = 1.0; for(int j=1; j<=2*order; j++) powers[j] = powers[j-1] * t;
        for(int r=0; r<m; r++) { for(int c=0; c<m; c++) A[r][c] += powers[r+c]; B[r] += powers[r] * v; }
    }
    std::vector<double> x; solve(A, B, x); memset(p.c, 0, sizeof(p.c)); for(int i=0; i<m; i++) p.c[i] = (float)x[i]; p.tStart = data[0].t; p.tEnd = data[n-1].t; p.order = order;
}
void compress(int order, float threshold) {
    int start = 0;
    while (start < tempCount && storageCount < MAX_STORAGE) {
        int end = start + 1; Poly bc; bool found = false;
        while (end < tempCount) {
            Poly cc; fit(&tempBuf[start], end - start + 1, order, cc);
            float me = 0; for (int i = start; i <= end; i++) {
                float dt = tempBuf[i].t - tempBuf[start].t, pr = 0, tp = 1; for(int j=0; j<=order; j++) { pr += cc.c[j] * tp; tp *= dt; }
                float err = std::abs(pr - tempBuf[i].v); if(err > me) me = err;
            }
            if (me <= threshold && (tempBuf[end].t - tempBuf[start].t) <= 15.0) { bc = cc; found = true; end++; } else break;
        }
        if (!found) { fit(&tempBuf[start], 1, order, bc); end = start + 1; }
        storage[storageCount++] = bc; start = end;
    }
}
void setup() {
    Serial.begin(115200); String line;
    while (Serial.available()) { line = Serial.readStringUntil('\n'); if (line.length() == 0) continue; int ci = line.indexOf(','); if (ci == -1) continue; addPoint(line.substring(0, ci).toFloat(), line.substring(ci + 1).toFloat()); }
    compress(5, 0.05); Serial.print("SIZE:"); Serial.println(storageCount * sizeof(Poly));
    for (int i = 0; i < storageCount; i++) {
        Poly& p = storage[i]; for (float t = p.tStart; t <= p.tEnd + 0.001; t += 0.1) {
            float dt = t - p.tStart, v = 0, tp = 1; for(int j=0; j<=p.order; j++) { v += p.c[j] * tp; tp *= dt; } Serial.print("RESULT:"); Serial.print(t); Serial.print(":"); Serial.println(v);
        }
    }
}
void loop() {}
