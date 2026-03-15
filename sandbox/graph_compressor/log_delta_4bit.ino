#include <Arduino.h>

#define MAX_SAMPLES 512

struct DataPoint {
    float t;
    float v;
};

class CompressedBuffer4Bit {
private:
    uint8_t compressed[MAX_SAMPLES/2];
    int count = 0;
    float firstValue = 0;
    float firstTimestamp = 0;
    const float VS = 0.05f;

    uint8_t pack(float delta) {
        int scaled = round(delta / VS);
        int clamped = std::max(-8, std::min(7, scaled));
        return (uint8_t)(clamped + 8) & 0x0F;
    }

    float unpack(uint8_t val) {
        int signedVal = (int)val - 8;
        return signedVal * VS;
    }

public:
    void push(float* rawT, float* rawV, int rawCount) {
        if (rawCount == 0) return;
        firstTimestamp = rawT[0];
        firstValue = rawV[0];
        count = rawCount;

        float currentReconstructed = firstValue;
        for (int i = 1; i < rawCount; i++) {
            float delta = rawV[i] - currentReconstructed;
            uint8_t p = pack(delta);

            // Feedback to prevent drift
            currentReconstructed += unpack(p);

            int byteIdx = (i-1) / 2;
            if ((i-1) % 2 == 0) {
                compressed[byteIdx] = p;
            } else {
                compressed[byteIdx] |= (p << 4);
            }
        }
    }

    void print(float* rawT) {
        Serial.print("SIZE:"); Serial.println((count-1)/2 + 1);
        float cv = firstValue;
        float err_comp = 0.0f; // Kahan summation

        Serial.print("RESULT:"); Serial.print(rawT[0]); Serial.print(":"); Serial.println(cv);
        for (int i = 1; i < count; i++) {
            int byteIdx = (i-1) / 2;
            uint8_t p;
            if ((i-1) % 2 == 0) p = compressed[byteIdx] & 0x0F;
            else p = (compressed[byteIdx] >> 4) & 0x0F;

            float y = unpack(p) - err_comp;
            float t = cv + y;
            err_comp = (t - cv) - y;
            cv = t;

            Serial.print("RESULT:"); Serial.print(rawT[i]); Serial.print(":"); Serial.println(cv);
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
        if (ci == -1) continue;
        rawT[rc] = line.substring(0, ci).toFloat();
        rawV[rc] = line.substring(ci + 1).toFloat();
        rc++;
        if (rc >= MAX_SAMPLES) break;
    }
    CompressedBuffer4Bit cb;
    cb.push(rawT, rawV, rc);
    cb.print(rawT);
}
void loop() {}
