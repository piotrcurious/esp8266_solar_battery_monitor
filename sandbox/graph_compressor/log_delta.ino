#include <Arduino.h>

#define MAX_SAMPLES 1024

class LogDeltaBuffer {
private:
    uint8_t b[MAX_SAMPLES * 2]; // t, v pairs
    int count = 0;
    float firstT = 0, firstV = 0;
    const float TS = 0.01f, VS = 0.01f;

    uint8_t comp(float d, float s) {
        if (std::abs(d) < 1e-6) return 0;
        float a = std::abs(d);
        float sc = a / s;
        float cp = log2f(1.0f + sc);
        uint8_t r = (uint8_t)std::min(127.0f, cp * 16.0f);
        if (d < 0) r |= 0x80;
        return r;
    }

    float expn(uint8_t cp, float s) {
        if (cp == 0) return 0;
        bool n = (cp & 0x80) != 0;
        float m = (cp & 0x7F) / 16.0f;
        float e = (powf(2.0f, m) - 1.0f) * s;
        return n ? -e : e;
    }

public:
    void push(float* rawT, float* rawV, int rc) {
        if (rc == 0) return;
        firstT = rawT[0];
        firstV = rawV[0];
        count = rc;

        float rt = firstT;
        float rv = firstV;

        for (int i = 1; i < rc; i++) {
            float dt = rawT[i] - rt;
            float dv = rawV[i] - rv;

            uint8_t ct = comp(dt, TS);
            uint8_t cv = comp(dv, VS);

            b[(i-1)*2] = ct;
            b[(i-1)*2 + 1] = cv;

            // Feedback
            rt += expn(ct, TS);
            rv += expn(cv, VS);
        }
    }

    void print(float* rawT) {
        Serial.print("SIZE:"); Serial.println(count * 2);
        float ct = firstT;
        float cv = firstV;
        float err_t = 0, err_v = 0;

        Serial.print("RESULT:"); Serial.print(ct); Serial.print(":"); Serial.println(cv);

        for (int i = 1; i < count; i++) {
            float dt = expn(b[(i-1)*2], TS);
            float dv = expn(b[(i-1)*2 + 1], VS);

            // Kahan for T
            float yt = dt - err_t;
            float tt = ct + yt;
            err_t = (tt - ct) - yt;
            ct = tt;

            // Kahan for V
            float yv = dv - err_v;
            float tv = cv + yv;
            err_v = (tv - cv) - yv;
            cv = tv;

            Serial.print("RESULT:"); Serial.print(ct); Serial.print(":"); Serial.println(cv);
        }
    }
};

void setup() {
    Serial.begin(115200);
    static float rawT[MAX_SAMPLES], rawV[MAX_SAMPLES];
    int rc = 0;
    String line;
    while (Serial.available()) {
        line = Serial.readStringUntil('\n');
        if (line.length() == 0) continue;
        int ci = line.indexOf(',');
        if (ci != -1) {
            rawT[rc] = line.substring(0, ci).toFloat();
            rawV[rc] = line.substring(ci + 1).toFloat();
            rc++;
            if (rc >= MAX_SAMPLES) break;
        }
    }
    LogDeltaBuffer ldb;
    ldb.push(rawT, rawV, rc);
    ldb.print(rawT);
}
void loop() {}
