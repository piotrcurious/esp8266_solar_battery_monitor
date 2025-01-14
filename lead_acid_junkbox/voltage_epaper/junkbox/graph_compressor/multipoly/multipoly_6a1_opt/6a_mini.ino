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

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Raw", 10, 5, 2);
}
