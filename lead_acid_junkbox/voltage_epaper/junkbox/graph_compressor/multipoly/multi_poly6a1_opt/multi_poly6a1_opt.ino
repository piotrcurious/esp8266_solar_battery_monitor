#include <TFT_eSPI.h>
#include <vector>
#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "AdvancedPolynomialFitter.hpp"

TFT_eSPI tft = TFT_eSPI(); // TFT instance
#define W 320
#define H 240
#define MAX_RAW 2048
#define POLY_CT 4
#define SEG_CT 4
#define POLYS_COMB 2
#define POINTS_PER_POLY 16

float rawData[MAX_RAW], rawMinY = 0, rawMaxY = 0;
uint32_t rawTimestamps[MAX_RAW];
uint16_t rawIdx = 0;

struct PolySeg {
    float coeffs[POLY_CT][6];
    uint32_t deltas[POLY_CT];
};

PolySeg segBuf[SEG_CT];
uint8_t segCnt = 0, head = 0, tail = 0;
uint32_t lastTs = 0;

float mapF(float x, float in1, float in2, float out1, float out2) {
    return (x - in1) * (out2 - out1) / (in2 - in1) + out1;
}

void addSeg(const PolySeg &s) {
    segBuf[segCnt++] = s;
    tail = (tail + 1) % SEG_CT;
    if (segCnt > SEG_CT) head = (head + 1) % SEG_CT;
}

void compressToSeg(const float *data, const uint32_t *ts, uint16_t sz, float *coeff, uint32_t &delta) {
    AdvancedPolynomialFitter fit;
    std::vector<float> x(sz), y(sz);
    float tAbs = 0;

    for (uint16_t i = 0; i < sz; i++) {
        x[i] = tAbs;
        y[i] = data[i];
        tAbs += ts[i];
    }
    auto coeffs = fit.fitPolynomial(x, y, 5, AdvancedPolynomialFitter::NONE);
    for (uint8_t i = 0; i < coeffs.size() && i < 6; i++) coeff[i] = coeffs[i];
    delta = tAbs;
}

float evalPoly(const float *coeff, float t) {
    float res = 0, tp = 1;
    for (uint8_t i = 0; i < 6; i++) {
        res += coeff[i] * tp;
        tp *= t;
    }
    return res;
}

void recombine(const PolySeg &s1, const PolySeg &s2, PolySeg &out) {
    AdvancedPolynomialFitter fit;

    for (uint8_t i = 0; i < POLY_CT; i += 2) {
        if (!s1.deltas[i] || !s1.deltas[i + 1]) break;
        std::vector<float> ts, vals;
        uint32_t tStart = 0;

        for (float t = 0; t <= s1.deltas[i]; t += s1.deltas[i] / 50.0) {
            ts.push_back(tStart + t);
            vals.push_back(evalPoly(s1.coeffs[i], t));
        }
        tStart += s1.deltas[i];
        for (float t = 0; t <= s1.deltas[i + 1]; t += s1.deltas[i + 1] / 50.0) {
            ts.push_back(tStart + t);
            vals.push_back(evalPoly(s1.coeffs[i + 1], t));
        }
        auto newCoeff = fit.fitPolynomial(ts, vals, 5, AdvancedPolynomialFitter::NONE);
        for (uint8_t j = 0; j < newCoeff.size() && j < 6; j++) out.coeffs[i / 2][j] = newCoeff[j];
        out.deltas[i / 2] = s1.deltas[i] + s1.deltas[i + 1];
    }
}

void recompress() {
    if (segCnt < 2) return;
    PolySeg newSeg = {}, s1 = segBuf[head], s2 = segBuf[(head + 1) % SEG_CT];
    recombine(s1, s2, newSeg);
    segBuf[head] = newSeg;
    segCnt -= 2;
}

float sampleScalarData(uint32_t timestamp) {
    float scalar = random(0, 1000) / 100.0; // background noise
    scalar = scalar + 20 * sin((float)timestamp * 0.0001);

    return scalar; // Random data in range [0, 10.0]
}

void sampleLog(float d, uint32_t ts) {
    static float buf[POINTS_PER_POLY];
    static uint32_t tBuf[POINTS_PER_POLY];
    static uint8_t idx = 0, pIdx = 0;

    buf[idx] = d;
    tBuf[idx++] = ts - lastTs;
    lastTs = ts;

    if (idx >= POINTS_PER_POLY) {
        if (!segCnt) addSeg({});
        float coeff[6];
        uint32_t delta;
        compressToSeg(buf, tBuf, POINTS_PER_POLY, coeff, delta);

        for (uint8_t i = 0; i < 6; i++) segBuf[segCnt - 1].coeffs[pIdx][i] = coeff[i];
        segBuf[segCnt - 1].deltas[pIdx++] = delta;

        if (pIdx >= POLY_CT) {
            pIdx = 0;
            if (segCnt >= SEG_CT) recompress();
            addSeg({});
        }
        idx = 0;
    }
}

// Log sampled data into the current segment
void sampleRaw(float data, uint32_t currentTimestamp) {
    // Check if the current segment is full
    if (rawIdx >= MAX_RAW - 1) {
        rawMinY = rawMaxY;
        rawMaxY = 0;
        for (uint16_t i = 0; i < rawIdx; i++) {
            rawData[i] = rawData[i + 1];
            rawTimestamps[i] = rawTimestamps[i + 1];
            if (rawData[i] > rawMaxY) {
                rawMaxY = rawData[i];
            }
            if (rawData[i] < rawMinY) {
                rawMinY = rawData[i];
            }
        }
        rawData[rawIdx] = data;
        rawTimestamps[rawIdx] = currentTimestamp;
    } else {
        // Store the data and timestamp
        rawData[rawIdx] = data;
        rawTimestamps[rawIdx] = currentTimestamp;
        rawMinY = rawMaxY;
        rawMaxY = 0;
        for (uint16_t i = 0; i < rawIdx; i++) {
            if (rawData[i] > rawMaxY) {
                rawMaxY = rawData[i];
            }
            if (rawData[i] < rawMinY) {
                rawMinY = rawData[i];
            }
        }
        rawIdx++;
    }
}

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Raw", 10, 5, 2);
}


void drawRawGraph() {
    tft.drawRect(0, 0, W, H - 1, TFT_RED);

    for (uint16_t i = 0; i < rawIdx; i++) {
        uint16_t y = mapF(rawData[i], rawMinY, rawMaxY, H - 1, 0);
        uint16_t x = mapF(rawTimestamps[i], rawTimestamps[0], rawTimestamps[rawIdx - 1], 0, W);
        tft.drawPixel(x, y, TFT_GREEN);
    }
}

void updateCompressedGraph(const PolySeg *segments, uint8_t count) {
    if (!count) return;

    uint32_t wStart = rawTimestamps[0], wEnd = rawTimestamps[rawIdx - 1];
    float minY = INFINITY, maxY = -INFINITY;

    for (uint8_t seg = head, i = 0; i < count; i++, seg = (seg + 1) % SEG_CT) {
        const PolySeg &s = segments[seg];
        for (uint8_t poly = 0; poly < POLY_CT; poly++) {
            if (!s.deltas[poly]) break;
            for (float t = 0; t <= s.deltas[poly]; t += s.deltas[poly] / 10) {
                float v = evalPoly(s.coeffs[poly], t);
                minY = min(minY, v);
                maxY = max(maxY, v);
            }
        }
    }

    if (isinf(minY) || isinf(maxY)) {
        minY = rawMinY;
        maxY = rawMaxY;
    }

    float range = maxY - minY;
    minY -= range * 0.05;
    maxY += range * 0.05;

    for (uint8_t seg = head, i = 0; i < count; i++, seg = (seg + 1) % SEG_CT) {
        const PolySeg &s = segments[seg];
        uint32_t tCur = wStart;

        for (uint8_t poly = 0; poly < POLY_CT; poly++) {
            if (!s.deltas[poly]) break;

            uint32_t steps = min(100UL, s.deltas[poly]);
            uint32_t step = s.deltas[poly] / steps;
            float lastX = -1, lastY = -1;

            for (uint32_t t = 0; t <= s.deltas[poly]; t += step) {
                float v = evalPoly(s.coeffs[poly], t);
  //              uint16_t x = constrain(mapF(tCur + t, wStart, wEnd, 0, W - 1), 0, W - 1);
  //              uint16_t y = constrain(mapF(v, minY, maxY, H - 1, 0), 0, H - 1);
                uint16_t x = mapF(tCur + t, wStart, wEnd, 0, W - 1);
                uint16_t y = mapF(v, minY, maxY, H - 1, 0);

                x = constrain(x, 0, W - 1);
                y = constrain(y, 0, H - 1);

                if (lastX >= 0 && lastY >= 0)
                    tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
                else
                    tft.drawPixel(x, y, TFT_YELLOW);

                lastX = x;
                lastY = y;
            }

            tCur += s.deltas[poly];
        }
    }
}

void updateCompressedGraphBackwards(const PolySeg *segments, uint8_t count) {
    if (!count) return;

    uint32_t wStart = rawTimestamps[0], wEnd = rawTimestamps[rawIdx - 1];
    float minY = INFINITY, maxY = -INFINITY;

    // First pass: find min/max value range (backwards)
    uint8_t segIdx = (head + count - 1) % SEG_CT;
    for (int8_t i = count - 1; i >= 0; i--) {
        const PolySeg &seg = segments[segIdx];
        for (uint8_t p = 0; p < POLY_CT; p++) {
            if (!seg.deltas[p]) break;
            for (uint32_t t = seg.deltas[p]; t > 0; t -= seg.deltas[p] / 10) {
                float v = evalPoly(seg.coeffs[p], t);
                minY = min(minY, v);
                maxY = max(maxY, v);
            }
        }
        segIdx = (segIdx + SEG_CT - 1) % SEG_CT;
    }

    // Ensure valid min/max values
    if (isinf(minY) || isinf(maxY)) {
        minY = rawMinY;
        maxY = rawMaxY;
    }

    // Add margin
    float range = maxY - minY;
    minY -= range * 0.05;
    maxY += range * 0.05;

    // Second pass: plot the data (backwards)
    uint32_t tCur = wEnd;
    segIdx = (head + count - 1) % SEG_CT;
    for (int8_t i = count - 1; i >= 0; i--) {
        const PolySeg &seg = segments[segIdx];
        for (uint8_t p = 0; p < POLY_CT; p++) {
            if (!seg.deltas[p]) break;

            uint32_t stepSize = seg.deltas[p] / min(100UL, seg.deltas[p]);
            float lastX = -1, lastY = -1;

            for (uint32_t t = seg.deltas[p]; t > 0; t -= stepSize) {
                float v = evalPoly(seg.coeffs[p], t);
                uint16_t x = constrain(mapF(tCur - t, wStart, wEnd, 0, W - 1), 0, W - 1);
                uint16_t y = constrain(mapF(v, minY, maxY, H - 1, 0), 0, H - 1);

                if (lastX >= 0 && lastY >= 0)
                    tft.drawLine(lastX, lastY, x, y, TFT_YELLOW);
                else
                    tft.drawPixel(x, y, TFT_YELLOW);

                lastX = x;
                lastY = y;
            }
            tCur -= seg.deltas[p];
        }
        segIdx = (segIdx + SEG_CT - 1) % SEG_CT;
    }
}

void loop() {
    delay(random(10, 100)); // Simulate random sampling interval
    uint32_t now = millis();

    float data = sampleScalarData(now);
    sampleLog(data, now);
    sampleRaw(data, now); // Debug log

    tft.fillScreen(TFT_BLACK);
    drawRawGraph(); // Update raw data graph
    updateCompressedGraph(segBuf, segCnt); // Update compressed graph
}
