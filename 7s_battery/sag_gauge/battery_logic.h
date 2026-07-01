#pragma once
#include <math.h>
#include <algorithm>

#ifndef clamp
#define clamp(v,lo,hi) (((v)<(lo))?(lo):((v)>(hi))?(hi):(v))
#endif

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

static inline float lp(float p, float in, float a) {
  return p + a * (in - p);
}

static inline void kahanAdd(float &sum, float &c, float input) {
  float y = input - c;
  float t = sum + y;
  c = (t - sum) - y;
  sum = t;
}

struct MonitorConfig {
    int   cells_s;
    float cap_ah;
    float nom_v_cell;
    float rint_fresh_mo;
    float rint_dead_mo;
    float bat_div;
    float cur_div;
    float acs_mv_a;
    float ref_load_a;
    float alpha_adc;
    float alpha_rint;
    float alpha_rest_v;
    float alpha_load_v;
    float idle_a;
    float sag_min_a;
    float charge_eff;
    uint32_t dim_ms;
    float km_per_wh;
    float thermal_k;
};

static constexpr MonitorConfig CFG_DEFAULT = {
    7, 20.0f, 3.60f, 60.0f, 300.0f,
    (110000.0f + 10000.0f) / 10000.0f, 2.0f, 185.0f, 10.0f,
    0.08f, 0.01f, 0.12f, 0.002f,
    0.20f, 1.50f, 0.99f, 30000, 0.05f, 0.2f
};

static float getRintSocFactor(float soc) {
  if (soc > 80.0f) return 1.0f;
  if (soc < 10.0f) return 1.8f; // Slightly more aggressive rise at end of life
  return 1.0f + (80.0f - soc) * (0.8f / 70.0f);
}

// Arrhenius-like Rint temp factor (Reference: 25C = 1.0)
// For every 10C drop, Rint increases by ~1.4x (Li-ion typical)
static float getRintTempFactor(float t_c) {
    float dt = 25.0f - t_c;
    return powf(1.035f, dt); // ~1.41x at 15C, ~2x at 5C
}

static constexpr uint32_t NVS_VERSION = 2;
static constexpr uint32_t NVS_MAGIC = 0x546B4142; // "BAtT"

static float socFromV(float v) {
  static constexpr struct { float v, s; } T[] = {
    {4.20f,100},{4.10f,95},{4.03f,90},{3.97f,85},{3.91f,80},
    {3.86f,75},{3.82f,70},{3.79f,65},{3.77f,60},{3.75f,55},
    {3.73f,50},{3.71f,45},{3.69f,40},{3.67f,35},{3.65f,30},
    {3.63f,25},{3.61f,20},{3.58f,15},{3.55f,10},{3.45f,5},{3.20f,0}
  };
  constexpr int N = sizeof(T)/sizeof(T[0]);
  if (v >= T[0].v) return 100.0f;
  if (v <= T[N-1].v) return 0.0f;
  for (int i = 0; i+1 < N; ++i)
    if (v <= T[i].v && v >= T[i+1].v) {
      float t = (v-T[i+1].v)/(T[i].v-T[i+1].v);
      return T[i+1].s + t*(T[i].s-T[i+1].s);
    }
  return 0.0f;
}

static float vFromSoc(float soc) {
  static constexpr struct { float v, s; } T[] = {
    {4.20f,100},{4.10f,95},{4.03f,90},{3.97f,85},{3.91f,80},
    {3.86f,75},{3.82f,70},{3.79f,65},{3.77f,60},{3.75f,55},
    {3.73f,50},{3.71f,45},{3.69f,40},{3.67f,35},{3.65f,30},
    {3.63f,25},{3.61f,20},{3.58f,15},{3.55f,10},{3.45f,5},{3.20f,0}
  };
  constexpr int N = sizeof(T)/sizeof(T[0]);
  if (soc >= 100.0f) return T[0].v;
  if (soc <= 0.0f) return T[N-1].v;
  for (int i = 0; i+1 < N; ++i)
    if (soc <= T[i].s && soc >= T[i+1].s) {
      float t = (soc-T[i+1].s)/(T[i].s-T[i+1].s);
      return T[i+1].v + t*(T[i].v-T[i+1].v);
    }
  return T[N-1].v;
}
