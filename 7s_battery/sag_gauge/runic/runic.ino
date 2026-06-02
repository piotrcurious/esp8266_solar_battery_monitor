#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <algorithm>
#include <cstring>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "../hardware_config.h"
#include "../battery_logic.h"

// ============================================================
// User hardware configuration
// ============================================================
static constexpr float CUR_POLARITY = 1.0f;
static constexpr float BAT_V_OFFSET = 0.0f;
static constexpr MonitorConfig CFG = CFG_DEFAULT;

static constexpr uint32_t UI_REFRESH_MS = 100;
static constexpr float ALPHA_ZERO = 0.005f;

static LGFX lcd;
static LGFX_Sprite canvas(&lcd);

// ============================================================
// State
// ============================================================
static float sBatMv = 0.0f, sCurMv = 0.0f, sZeroMv = 0.0f;
static float vPack = 0.0f, iA = 0.0f, vRested = 0.0f, rInt = 0.060f;
static float socOcv = 0.0f, socCoul = 0.0f, socBlend = 0.0f;
static float ahOut = 0.0f, ahOutK = 0.0f;
static uint32_t lastUiMs = 0;
static Preferences prefs;

// ============================================================
// Helpers
// ============================================================
static uint32_t adcAvgMv(int pin, int n) {
  uint32_t s = 0; for (int i = 0; i < n; ++i) s += analogReadMilliVolts(pin);
  return s / (uint32_t)n;
}


static uint32_t blendCol(uint32_t a, uint32_t b, float t) {
  t = clampf(t, 0.0f, 1.0f);
  int ar=(a>>16)&0xFF, ag=(a>>8)&0xFF, ab=a&0xFF;
  int br=(b>>16)&0xFF, bg=(b>>8)&0xFF, bb=b&0xFF;
  return lcd.color888((uint8_t)(ar+(br-ar)*t), (uint8_t)(ag+(bg-ag)*t), (uint8_t)(ab+(bb-ab)*t));
}

static uint32_t socCol(float s) {
  if (s < 15.0f) return blendCol(0xFF0000, 0xFF6400, s / 15.0f);
  if (s < 40.0f) return blendCol(0xFF6400, 0xFFFF00, (s - 15.0f) / 25.0f);
  if (s < 70.0f) return blendCol(0xFFFF00, 0x00FF00, (s - 40.0f) / 30.0f);
  return blendCol(0x00FF00, 0x64FFFF, (s - 70.0f) / 30.0f);
}

static void drawBar(int x, int y, int w, int h, float soc, uint32_t fill, uint32_t border, uint32_t empty) {
  const int S=7, G=3, sw=(w-G*(S-1))/S;
  canvas.fillRoundRect(x-2, y-2, w+4, h+4, 4, lcd.color888(20,20,30));
  for (int i = 0; i < S; ++i) {
    int sx = x + i*(sw+G);
    canvas.drawRoundRect(sx, y, sw, h, 3, border);
    canvas.fillRoundRect(sx+1, y+1, sw-2, h-2, 2, empty);
    float f = clampf((soc-i*(100.0f/S))/(100.0f/S), 0.0f, 1.0f);
    if (f > 0.0f) {
      int fh = (int)((h-2)*f + 0.5f);
      canvas.fillRoundRect(sx+1, y+1+(h-2-fh), sw-2, fh, 2, blendCol(0xFF5032, fill, f));
    }
  }
}

static void saveState() {
  prefs.begin("sag_gauge", false);
  prefs.putUInt("magic", NVS_MAGIC);
  prefs.putUInt("version", NVS_VERSION);
  prefs.putFloat("ahOut", ahOut);
  prefs.putFloat("rInt", rInt);
  prefs.putFloat("sZeroMv", sZeroMv);
  prefs.end();
}

static void loadState() {
  prefs.begin("sag_gauge", true);
  uint32_t v = prefs.getUInt("version", 0);
  uint32_t m = prefs.getUInt("magic", 0);
  if (v == NVS_VERSION && m == NVS_MAGIC) {
    ahOut = prefs.getFloat("ahOut", 0.0f);
    rInt = prefs.getFloat("rInt", 0.060f);
    sZeroMv = prefs.getFloat("sZeroMv", 0.0f);
  }
  prefs.end();
}

// ============================================================
// Logic
// ============================================================
static void updateMeasurements() {
  static float lastIA_step = 0.0f, lastV_step = 0.0f;

  sBatMv = lp(sBatMv, (float)adcAvgMv(PIN_BAT_VOLT, 16), CFG.alpha_adc);
  sCurMv = lp(sCurMv, (float)adcAvgMv(PIN_CUR_SENS, 16), CFG.alpha_adc);
  if (sZeroMv < 1.0f) sZeroMv = sCurMv;
  if (fabsf(iA) < CFG.idle_a) sZeroMv = lp(sZeroMv, sCurMv, ALPHA_ZERO);

  vPack = (sBatMv * CFG.bat_div) / 1000.0f + BAT_V_OFFSET;
  iA = CUR_POLARITY * (sCurMv - sZeroMv) * CFG.cur_div / CFG.acs_mv_a;

  // Runic simple thermal model for temp comp
  static float tEstRunic = 25.0f;
  float pLoss = iA * iA * rInt;
  tEstRunic += (pLoss - 0.2f * (tEstRunic - 25.0f)) * (UI_REFRESH_MS / 120000.0f);

  if (vRested < 1.0f) vRested = vPack;
  if (fabsf(iA) < CFG.idle_a) vRested = lp(vRested, vPack, CFG.alpha_rest_v);
  else vRested = lp(vRested, vPack + (iA > 0 ? 1.0f : -1.0f) * fabsf(iA) * rInt * getRintSocFactor(socBlend) * getRintTempFactor(tEstRunic), CFG.alpha_load_v);

  // Advanced Rint Estimation (Step + Steady-State)
  float dI = iA - lastIA_step;
  float dV = vPack - lastV_step;
  if (vPack > 10.05f && vPack < 34.95f) {
    if (fabsf(dI) > 1.5f) {
      float rStep = -dV / dI;
      if (isfinite(rStep) && rStep > 0.01f && rStep < 0.8f) rInt = lp(rInt, rStep, CFG.alpha_rint * 3.0f);
    }
    if (fabsf(iA) > CFG.sag_min_a && fabsf(dI) < 0.1f) {
      float rEst = fabsf(vRested - vPack) / fabsf(iA);
      if (isfinite(rEst) && rEst > 0.01f && rEst < 0.8f) rInt = lp(rInt, rEst, CFG.alpha_rint);
    }
  }
  lastIA_step = iA; lastV_step = vPack;

  socOcv = socFromV(vRested / (float)CFG.cells_s);
  // Simple blend for runic - ensuring smooth transitions
  if (socBlend < 1.0f) socBlend = socOcv;
  socBlend = lp(socBlend, socOcv, 0.05f);

  if (iA > CFG.idle_a) {
    float dAh = iA * (UI_REFRESH_MS / 3600000.0f);
    float y = dAh - ahOutK; float t = ahOut + y;
    ahOutK = (t - ahOut) - y; ahOut = t;
  }
}

void renderUi() {
  char b[64];
  if (!isfinite(socBlend)) socBlend = 0;
  if (!isfinite(rInt)) rInt = 0;
  canvas.fillScreen(TFT_BLACK);
  canvas.fillRoundRect(0, 0, 160, 14, 3, lcd.color888(18, 18, 24));
  canvas.setTextSize(1); canvas.setTextColor(socCol(socBlend));
  canvas.setCursor(4, 4);
  snprintf(b, sizeof(b), "%dS Li-ion  %.1fV", CFG.cells_s, vPack);
  canvas.print(b);
  canvas.setCursor(120, 4); canvas.print(iA > CFG.idle_a ? "DISCH" : (iA < -CFG.idle_a ? "CHG" : "IDLE"));

  canvas.setTextSize(2); canvas.setCursor(4, 18);
  if (socBlend < 15.0f && (millis() / 500) % 2 == 0) canvas.print("LOW BAT!");
  else {
    snprintf(b, sizeof(b), "%.2fV/c", vRested / CFG.cells_s);
    canvas.print(b);
  }

  canvas.setTextSize(1); canvas.setTextColor(0xB4B4C0);
  canvas.setCursor(96, 18); snprintf(b, sizeof(b), "SOC %.0f%%", socBlend); canvas.print(b);
  canvas.setCursor(96, 29); snprintf(b, sizeof(b), "I %.1fA", iA); canvas.print(b);
  canvas.setCursor(96, 40); snprintf(b, sizeof(b), "R %.0fmO", rInt * 1000.0f); canvas.print(b);

  drawBar(4, 48, 152, 22, socBlend, socCol(socBlend), 0xB4B4C0, 0);

  canvas.setCursor(4, 72); canvas.setTextColor(0xFFFFFF);
  snprintf(b, sizeof(b), "Ah:%.2f  Sag:%.0fmV", ahOut, fabsf(vRested - vPack) * 1000.0f);
  canvas.print(b);

  canvas.pushSprite(&lcd, 0, 0);
}

void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);

  analogReadResolution(12);
  lcd.init(); lcd.setRotation(0); lcd.setBrightness(200);
  canvas.createSprite(160, 80);

  loadState();

  delay(300);
  float sCurMvBoot = (float)adcAvgMv(PIN_CUR_SENS, 64);
  if (sZeroMv < 100.0f) sZeroMv = sCurMvBoot;
  sCurMv = sCurMvBoot;

  vPack = vRested = (adcAvgMv(PIN_BAT_VOLT, 32) * CFG.bat_div) / 1000.0f;
  socBlend = socFromV(vRested / (float)CFG.cells_s);
}

static uint32_t lastSaveMs = 0;

void loop() {
  uint32_t now = millis();
  esp_task_wdt_reset();

  if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "SETZERO") {
          if (fabsf(iA) < 1.0f) { sZeroMv = sCurMv; saveState(); Serial.println("Zero CAL OK"); }
          else Serial.println("ERR: Current too high for zero cal");
      }
      else if (cmd == "RESET") { ahOut = 0; ahOutK = 0; saveState(); Serial.println("Session RESET"); }
  }

  if (now - lastUiMs >= UI_REFRESH_MS) {
    lastUiMs = now;
    updateMeasurements();
    renderUi();

    if (now - lastSaveMs > 60000) {
      lastSaveMs = now;
      saveState();
    }

    Serial.printf("V=%.2f I=%.1f R=%.0f SOC=%.0f%%\n", vPack, iA, rInt*1000.0f, socBlend);
  }
}
