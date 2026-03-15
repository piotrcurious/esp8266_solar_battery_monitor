#include <Arduino.h>
#include <vector>
#define MAX_TEMP 256
#define MAX_STOR 64
struct DP { float t, v, u; };
struct PolyW { float c[4]; float tStart, tEnd; };
DP temp[MAX_TEMP]; int tc = 0;
PolyW stor[MAX_STOR]; int sc = 0;
void add(float t, float v, float u = 1.0) { if(tc < MAX_TEMP) temp[tc++] = {t, v, u}; }
void solveWeighted(DP* d, int n, int order, float* coeffs) {
    int m = order + 1; std::vector<std::vector<double>> A(m, std::vector<double>(m, 0.0)); std::vector<double> B(m, 0.0); double t0 = d[0].t;
    for (int i = 0; i < n; i++) {
        double t = d[i].t - t0, v = d[i].v, w = 1.0 / (d[i].u * d[i].u); std::vector<double> p(2 * order + 1); p[0] = 1.0; for(int j=1; j<=2*order; j++) p[j] = p[j-1] * t;
        for(int r=0; r<m; r++) { for(int c=0; c<m; c++) A[r][c] += p[r+c] * w; B[r] += p[r] * v * w; }
    }
    for (int i = 0; i < m; i++) {
        int pivot = i; for (int j = i + 1; j < m; j++) if (std::abs(A[j][i]) > std::abs(A[pivot][i])) pivot = j;
        std::swap(A[i], A[pivot]); std::swap(B[i], B[pivot]);
        if (std::abs(A[i][i]) > 1e-18) for (int j = i + 1; j < m; j++) { double f = A[j][i] / A[i][i]; B[j] -= f * B[i]; for (int k = i; k < m; k++) A[j][k] -= f * A[i][k]; }
    }
    std::vector<double> x(m, 0.0); for (int i = m - 1; i >= 0; i--) if (std::abs(A[i][i]) > 1e-18) { double s = 0; for (int j = i + 1; j < m; j++) s += A[i][j] * x[j]; x[i] = (B[i] - s) / A[i][i]; }
    for(int i=0; i<m; i++) coeffs[i] = (float)x[i];
}
void compress() {
    int s = 0;
    while (s < tc && sc < MAX_STOR) {
        int e = s + 1; PolyW bc; bool found = false;
        while (e < tc) {
            float cc[4] = {0}; solveWeighted(&temp[s], e-s+1, 3, cc);
            float me = 0; for (int i = s; i <= e; i++) {
                float dt = temp[i].t - temp[s].t, pr = 0, tp = 1; for(int j=0; j<4; j++) { pr += cc[j] * tp; tp *= dt; }
                float err = std::abs(pr - temp[i].v); if(err > me) me = err;
            }
            if (me <= 0.1 && (temp[e].t - temp[s].t) <= 15.0) { for(int j=0; j<4; j++) bc.c[j] = cc[j]; bc.tStart = temp[s].t; bc.tEnd = temp[e].t; found = true; e++; } else break;
        }
        if (!found) { solveWeighted(&temp[s], 1, 3, bc.c); bc.tStart = temp[s].t; bc.tEnd = temp[s].t; e = s + 1; }
        stor[sc++] = bc; s = e;
    }
}
void setup() {
    Serial.begin(115200); String line;
    while (Serial.available()) { line = Serial.readStringUntil('\n'); if (line.length() == 0) continue; int ci = line.indexOf(','); if (ci == -1) continue; add(line.substring(0, ci).toFloat(), line.substring(ci + 1).toFloat(), 0.1 + 0.05 * (tc*0.1)); }
    compress(); Serial.print("SIZE:"); Serial.println(sc * sizeof(PolyW));
    for (int i = 0; i < sc; i++) {
        PolyW& p = stor[i]; for (float t = p.tStart; t <= p.tEnd + 0.001; t += 0.1) {
            float dt = t - p.tStart, v = 0, tp = 1; for(int j=0; j<4; j++) { v += p.c[j] * tp; tp *= dt; } Serial.print("RESULT:"); Serial.print(t); Serial.print(":"); Serial.println(v);
        }
    }
}
void loop() {}
