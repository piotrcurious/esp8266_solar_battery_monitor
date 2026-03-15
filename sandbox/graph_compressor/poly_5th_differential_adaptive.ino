#include <Arduino.h>
#include <vector>
#define MAX_TEMP 256
#define MAX_STOR 64
const float SCALE = 1000.0;
struct DP { float t, v; };
struct Poly5DA { int16_t c[6]; float tStart, tEnd; };
DP temp[MAX_TEMP]; int tc = 0;
Poly5DA stor[MAX_STOR]; int sc = 0;
void add(float t, float v) { if(tc < MAX_TEMP) temp[tc++] = {t, v}; }
void solve(const std::vector<std::vector<double>>& A, const std::vector<double>& b, std::vector<double>& x) {
    int n = A.size(); std::vector<std::vector<double>> mat = A; std::vector<double> vec = b;
    for (int i = 0; i < n; i++) {
        int p = i; for (int j = i+1; j < n; j++) if (std::abs(mat[j][i]) > std::abs(mat[p][i])) p = j;
        std::swap(mat[i], mat[p]); std::swap(vec[i], vec[p]);
        if (std::abs(mat[i][i]) > 1e-15) for (int j = i+1; j < n; j++) { double f = mat[j][i] / mat[i][i]; vec[j] -= f * vec[i]; for (int k = i; k < n; k++) mat[j][k] -= f * mat[i][k]; }
    }
    x.assign(n, 0); for (int i = n-1; i >= 0; i--) if (std::abs(mat[i][i]) > 1e-15) { double s = 0; for (int j = i+1; j < n; j++) s += mat[i][j] * x[j]; x[i] = (vec[i]-s)/mat[i][i]; }
}
void fit(DP* d, int n, double* c) {
    if (n < 6) { memset(c, 0, 8*6); double s = 0; for(int i=0; i<n; i++) s += d[i].v; c[0] = s/n; return; }
    std::vector<std::vector<double>> A(6, std::vector<double>(6, 0.0)); std::vector<double> B(6, 0.0); double t0 = d[0].t;
    for (int i = 0; i < n; i++) {
        double t = d[i].t-t0, v = d[i].v, p[11]; p[0] = 1.0; for(int j=1; j<=10; j++) p[j] = p[j-1]*t;
        for(int r=0; r<6; r++) { for(int col=0; col<6; col++) A[r][col]+=p[r+col]; B[r]+=p[r]*v; }
    }
    std::vector<double> res; solve(A, B, res); for(int i=0; i<6; i++) c[i] = res[i];
}
void compress() {
    int s=0; double lc[6]={0};
    while (s<tc && sc<MAX_STOR) {
        int e=s+1; double bc[6]={0}; bool f=false;
        while (e<tc) {
            double cc[6]={0}; fit(&temp[s], e-s+1, cc);
            double me=0; for (int i=s; i<=e; i++) {
                double t=temp[i].t-temp[s].t, pr=0, tp=1; for(int j=0; j<6; j++) { pr+=cc[j]*tp; tp*=t; }
                double err=std::abs(pr-temp[i].v); if(err>me) me=err;
            }
            if (me<=0.05 && (temp[e].t-temp[s].t)<=15.0) { memcpy(bc, cc, 8*6); f=true; e++; } else break;
        }
        if (!f) { memset(bc, 0, 8*6); bc[0]=temp[s].v; e=s+1; }
        Poly5DA p; for(int i=0; i<6; i++) p.c[i]=(int16_t)((bc[i]-lc[i])*SCALE);
        p.tStart = temp[s].t; p.tEnd = temp[e-1].t; stor[sc++]=p; memcpy(lc, bc, 8*6); s=e;
    }
}
void setup() {
    Serial.begin(115200); String line;
    while (Serial.available()) { line = Serial.readStringUntil('\n'); if(line.length()==0) continue; int ci=line.indexOf(','); if(ci==-1) continue; add(line.substring(0, ci).toFloat(), line.substring(ci+1).toFloat()); }
    compress(); Serial.print("SIZE:"); Serial.println(sc*sizeof(Poly5DA));
    double cc[6]={0};
    for(int i=0; i<sc; i++) {
        Poly5DA p=stor[i]; for(int j=0; j<6; j++) cc[j]+=(double)p.c[j]/SCALE;
        for(float t=p.tStart; t<=p.tEnd+0.001; t+=0.1) { float dt = t - p.tStart; double v=0, tp=1; for(int j=0; j<6; j++) { v+=cc[j]*tp; tp*=dt; } Serial.print("RESULT:"); Serial.print(t); Serial.print(":"); Serial.println((float)v); }
    }
}
void loop() {}
