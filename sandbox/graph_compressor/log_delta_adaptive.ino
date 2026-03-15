#include <Arduino.h>
#include <vector>

class AdaptiveCompressor {
private:
    float errorAccumulator = 0;
    float adaptiveScale = 1.0f;
    float scaleEMA = 1.0f;
    const float scaleAlpha = 0.1f;
    const float errorDiffusionRate = 0.7f;
public:
    uint8_t compress(float delta, float baseScale) {
        float adjustedDelta = delta + errorAccumulator * errorDiffusionRate;
        float currentScale = baseScale * adaptiveScale;
        float scaled = adjustedDelta / currentScale;
        float sign = scaled >= 0 ? 1.0f : -1.0f;
        float absScaled = std::abs(scaled);
        float compressed = sign * log2f(1.0f + absScaled);
        int quantized = std::max(-127, std::min(127, (int)round(compressed * 64)));
        float reconstructed = sign * (powf(2.0f, std::abs(quantized) / 64.0f) - 1.0f) * currentScale;
        errorAccumulator = delta - reconstructed;

        uint8_t result = std::abs(quantized) & 0x7F;
        if (quantized < 0) result |= 0x80;

        float targetScale = std::abs(delta) / 32.0f;
        scaleEMA = scaleAlpha * targetScale + (1.0f - scaleAlpha) * scaleEMA;
        adaptiveScale = std::max(0.001f, scaleEMA);

        return result;
    }
    float expand(uint8_t compressed, float baseScale) {
        bool negative = (compressed & 0x80) != 0;
        float magnitude = (compressed & 0x7F) / 64.0f;
        float currentScale = baseScale * adaptiveScale;
        float expanded = (powf(2.0f, magnitude) - 1.0f) * currentScale;
        if (negative) expanded = -expanded;

        float targetScale = std::abs(expanded) / 32.0f;
        scaleEMA = scaleAlpha * targetScale + (1.0f - scaleAlpha) * scaleEMA;
        adaptiveScale = std::max(0.001f, scaleEMA);

        return expanded;
    }
};

void setup() {
    Serial.begin(115200);
    AdaptiveCompressor tc, vc;
    std::vector<float> ts, vs;
    String line;
    while(Serial.available()){
        line = Serial.readStringUntil('\n');
        if(line.length()==0) continue;
        int ci = line.indexOf(',');
        if(ci==-1) continue;
        ts.push_back(line.substring(0, ci).toFloat());
        vs.push_back(line.substring(ci+1).toFloat());
    }
    if(ts.empty()) return;

    std::vector<uint8_t> compT, compV;
    float lastT = ts[0], lastV = vs[0];
    for(size_t i=1; i<ts.size(); i++){
        compT.push_back(tc.compress(ts[i]-lastT, 0.01f));
        compV.push_back(vc.compress(vs[i]-lastV, 0.01f));
        lastT = ts[i]; lastV = vs[i];
    }

    Serial.print("SIZE:"); Serial.println((int)(compT.size() + compV.size()));
    AdaptiveCompressor td, vd;
    float curT = ts[0], curV = vs[0];
    Serial.print("RESULT:"); Serial.print(curT); Serial.print(":"); Serial.println(curV);
    for(size_t i=0; i<compT.size(); i++){
        curT += td.expand(compT[i], 0.01f);
        curV += vd.expand(compV[i], 0.01f);
        Serial.print("RESULT:"); Serial.print(curT); Serial.print(":"); Serial.println(curV);
    }
}
void loop(){}
