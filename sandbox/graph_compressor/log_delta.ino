#include <Arduino.h>

class CompressedBuffer {
    static const int BS = 4096; uint8_t b[BS]; int h=0, c=0; float ct=0, cv=0; const float TS=0.01f, VS=0.01f;
    uint8_t comp(float d, float s) { if(std::abs(d)<1e-6) return 0; float a=std::abs(d), sc=a/s, cp=log2f(1.0f+sc); uint8_t r=(uint8_t)std::min(127.0f, cp*16.0f); if(d<0) r|=0x80; return r; }
    float expn(uint8_t cp, float s) { if(cp==0) return 0; bool n=(cp&0x80)!=0; float m=(cp&0x7F)/16.0f, e=(powf(2.0f,m)-1.0f)*s; return n?-e:e; }
public:
    void push(float t, float v) { if(c>=BS/2) return; if(c==0) { ct=t; cv=v; h=0; c++; return; }
        b[h]=comp(ct-t, TS); b[h+1]=comp(cv-v, VS); h=(h+2)%BS; ct=t; cv=v; c++; }
    void print() {
        Serial.print("SIZE:"); Serial.println(c*2); float at=ct, av=cv;
        Serial.print("RESULT:"); Serial.print(at); Serial.print(":"); Serial.println(av);
        int idx=(h-2+BS)%BS; for(int i=1; i<c; i++) { at+=expn(b[idx], TS); av+=expn(b[idx+1], VS); Serial.print("RESULT:"); Serial.print(at); Serial.print(":"); Serial.println(av); idx=(idx-2+BS)%BS; }
    }
};

void setup() {
    Serial.begin(115200); CompressedBuffer cb; String line;
    while (Serial.available()) { line=Serial.readStringUntil('\n'); if(line.length()>0) cb.push(line.substring(0, line.indexOf(',')).toFloat(), line.substring(line.indexOf(',')+1).toFloat()); }
    cb.print();
}
void loop() {}
