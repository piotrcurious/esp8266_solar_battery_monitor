#include <Arduino.h>

#define MAX_TEMPORARY 512
#define MAX_STORAGE 256
const float SCALE = 1000.0;

struct DataPoint { float timestamp, value; };
struct Polynomial { int16_t a3, a2, a1, a0; float tDelta; };

DataPoint temporaryBuffer[MAX_TEMPORARY];
int tempBufferCount = 0;
Polynomial storageBuffer[MAX_STORAGE];
int storageCount = 0;

void addDataPoint(float t, float v) { if (tempBufferCount < MAX_TEMPORARY) temporaryBuffer[tempBufferCount++] = {t, v}; }

void fitPolynomial(DataPoint* segment, int count, double* coeffs) {
    if (count < 4) { coeffs[0]=0; coeffs[1]=0; coeffs[2]=0; double s=0; for(int i=0; i<count; i++) s+=segment[i].value; coeffs[3]=s/count; return; }
    double X[4][4]={0}, Y[4]={0}, t0=segment[0].timestamp;
    for (int i=0; i<count; i++) {
        double t=segment[i].timestamp-t0, v=segment[i].value, t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t, t6=t5*t;
        X[0][0]+=t6; X[0][1]+=t5; X[0][2]+=t4; X[0][3]+=t3; X[1][0]+=t5; X[1][1]+=t4; X[1][2]+=t3; X[1][3]+=t2;
        X[2][0]+=t4; X[2][1]+=t3; X[2][2]+=t2; X[2][3]+=t; X[3][0]+=t3; X[3][1]+=t2; X[3][2]+=t; X[3][3]+=1;
        Y[0]+=t3*v; Y[1]+=t2*v; Y[2]+=t*v; Y[3]+=v;
    }
    for (int i=0; i<4; i++) {
        int p=i; for(int j=i+1; j<4; j++) if(std::abs(X[j][i])>std::abs(X[p][i])) p=j;
        for(int j=i; j<4; j++) std::swap(X[i][j], X[p][j]); std::swap(Y[i], Y[p]);
        if(std::abs(X[i][i])>1e-12) for(int j=i+1; j<4; j++) { double f=X[j][i]/X[i][i]; Y[j]-=f*Y[i]; for(int k=i; k<4; k++) X[j][k]-=f*X[i][k]; }
    }
    for (int i=3; i>=0; i--) { if(std::abs(X[i][i])>1e-12) { double s=0; for(int j=i+1; j<4; j++) s+=X[i][j]*coeffs[j]; coeffs[i]=(Y[i]-s)/X[i][i]; } else coeffs[i]=0; }
}

void compress() {
    int start=0; float l3=0, l2=0, l1=0, l0=0;
    while (start<tempBufferCount && storageCount<MAX_STORAGE) {
        int end=start+1; double bc[4]={0}; bool f=false;
        while (end<tempBufferCount) {
            double cc[4]={0}; fitPolynomial(&temporaryBuffer[start], end-start+1, cc);
            double me=0; for (int i=start; i<=end; i++) {
                double t=temporaryBuffer[i].timestamp-temporaryBuffer[start].timestamp;
                double p=cc[0]*t*t*t+cc[1]*t*t+cc[2]*t+cc[3]; double e=std::abs(p-temporaryBuffer[i].value); if(e>me) me=e;
            }
            if(me<=0.05 && (temporaryBuffer[end].timestamp-temporaryBuffer[start].timestamp)<=10.0) { memcpy(bc, cc, sizeof(bc)); f=true; end++; } else break;
        }
        if(!f) { bc[0]=0; bc[1]=0; bc[2]=0; bc[3]=temporaryBuffer[start].value; end=start+1; }
        Polynomial p; p.a3=(int16_t)((bc[0]-l3)*SCALE); p.a2=(int16_t)((bc[1]-l2)*SCALE); p.a1=(int16_t)((bc[2]-l1)*SCALE); p.a0=(int16_t)((bc[3]-l0)*SCALE);
        p.tDelta=temporaryBuffer[end-1].timestamp-temporaryBuffer[start].timestamp; storageBuffer[storageCount++]=p;
        l3=bc[0]; l2=bc[1]; l1=bc[2]; l0=bc[3]; start=end;
    }
}

void setup() {
    Serial.begin(115200); String line;
    while (Serial.available()) { line=Serial.readStringUntil('\n'); if(line.length()>0) addDataPoint(line.substring(0, line.indexOf(',')).toFloat(), line.substring(line.indexOf(',')+1).toFloat()); }
    compress(); Serial.print("SIZE:"); Serial.println(storageCount*sizeof(Polynomial));
    float c3=0, c2=0, c1=0, c0=0, ct=temporaryBuffer[0].timestamp;
    for(int i=0; i<storageCount; i++) {
        Polynomial p=storageBuffer[i]; c3+=(float)p.a3/SCALE; c2+=(float)p.a2/SCALE; c1+=(float)p.a1/SCALE; c0+=(float)p.a0/SCALE;
        for(float t=0; t<=p.tDelta+0.001; t+=0.1) { Serial.print("RESULT:"); Serial.print(ct+t); Serial.print(":"); Serial.println(c3*t*t*t+c2*t*t+c1*t+c0); }
        ct+=p.tDelta;
    }
}
void loop() {}
