#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <algorithm>
#include <cstring>
#include <Preferences.h>

#ifndef clamp
#define clamp(v,lo,hi) (((v)<(lo))?(lo):((v)>(hi))?(hi):(v))
#endif

// ============================================================
// User hardware configuration
// ============================================================
static constexpr int TFT_SCK  = 18, TFT_MOSI = 23, TFT_MISO = -1;
static constexpr int TFT_DC   = 16, TFT_CS   = 5,  TFT_RST  = 17, TFT_BL = 4;
static constexpr int PIN_BAT_VOLT = 34, PIN_CUR_SENS = 35, PIN_BUTTON = 0;

static constexpr float CUR_POLARITY = 1.0f;
static constexpr float BAT_V_OFFSET = 0.0f;

struct MonitorConfig {
    int   cells_s;
    float cap_ah;
    float nom_v_cell;
    float rint_fresh_mo;
    float rint_dead_mo;
    float bat_div;
    float cur_div;
    float acs_mv_a;
    float alpha_adc;
    float alpha_rint;
    float alpha_rest_v;
    float alpha_load_v;
    float idle_a;
    float sag_min_a;
    float charge_eff;
    uint32_t dim_ms;
};

static constexpr MonitorConfig CFG = {
    7, 20.0f, 3.60f, 60.0f, 300.0f,
    (110000.0f + 10000.0f) / 10000.0f, 2.0f, 185.0f,
    0.08f, 0.01f, 0.12f, 0.002f,
    0.20f, 1.50f, 0.99f, 30000
};

static constexpr uint32_t UI_REFRESH_MS = 100;
static constexpr float ALPHA_ZERO = 0.005f;

// ============================================================
// LovyanGFX custom device
// ============================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7735 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = VSPI_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = TFT_SCK; c.pin_mosi = TFT_MOSI;
      c.pin_miso = TFT_MISO; c.pin_dc = TFT_DC;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = TFT_CS; c.pin_rst = TFT_RST;
      c.panel_width = 160; c.panel_height = 80;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = TFT_BL; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

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
static constexpr uint32_t NVS_VERSION = 2;

// ============================================================
// Helpers
// ============================================================
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static inline float lp(float p, float in, float a) { return p + a * (in - p); }
static uint32_t adcAvgMv(int pin, int n) {
  uint32_t s = 0; for (int i = 0; i < n; ++i) s += analogReadMilliVolts(pin);
  return s / (uint32_t)n;
}

static float socFromV(float v) {
  static constexpr struct { float v, s; } T[] = {
    {4.20f,100},{4.10f,95},{4.00f,88},{3.95f,84},{3.90f,78},{3.85f,70},
    {3.80f,62},{3.75f,54},{3.70f,45},{3.65f,36},{3.60f,28},{3.50f,16},
    {3.40f,8},{3.20f,0}
  };
  if (v >= T[0].v) return 100.0f;
  if (v <= T[13].v) return 0.0f;
  for (int i = 0; i < 13; ++i)
    if (v <= T[i].v && v >= T[i+1].v) {
      float t = (v-T[i+1].v)/(T[i].v-T[i+1].v);
      return T[i+1].s + t*(T[i].s-T[i+1].s);
    }
  return 0.0f;
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
  prefs.putUInt("version", NVS_VERSION);
  prefs.putFloat("ahOut", ahOut);
  prefs.putFloat("rInt", rInt);
  prefs.putFloat("sZeroMv", sZeroMv);
  prefs.end();
}

static void loadState() {
  prefs.begin("sag_gauge", true);
  uint32_t v = prefs.getUInt("version", 0);
  if (v == NVS_VERSION) {
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
  sBatMv = lp(sBatMv, (float)adcAvgMv(PIN_BAT_VOLT, 16), CFG.alpha_adc);
  sCurMv = lp(sCurMv, (float)adcAvgMv(PIN_CUR_SENS, 16), CFG.alpha_adc);
  if (sZeroMv < 1.0f) sZeroMv = sCurMv;
  if (fabsf(iA) < CFG.idle_a) sZeroMv = lp(sZeroMv, sCurMv, ALPHA_ZERO);

  vPack = (sBatMv * CFG.bat_div) / 1000.0f + BAT_V_OFFSET;
  iA = CUR_POLARITY * (sCurMv - sZeroMv) * CFG.cur_div / CFG.acs_mv_a;

  if (vRested < 1.0f) vRested = vPack;
  if (fabsf(iA) < CFG.idle_a) vRested = lp(vRested, vPack, CFG.alpha_rest_v);
  else vRested = lp(vRested, vPack + (iA > 0 ? 1.0f : -1.0f) * fabsf(iA) * rInt, CFG.alpha_load_v);

  float sag = fabsf(vRested - vPack);
  if (fabsf(iA) > CFG.sag_min_a) {
    float rEst = sag / fabsf(iA);
    if (isfinite(rEst) && rEst > 0.01f && rEst < 0.8f) rInt = lp(rInt, rEst, CFG.alpha_rint);
  }

  socOcv = socFromV(vRested / (float)CFG.cells_s);
  // Simple blend for runic
  socBlend = lp(socBlend, socOcv, 0.05f);

  if (iA > CFG.idle_a) {
    float dAh = iA * (UI_REFRESH_MS / 3600000.0f);
    float y = dAh - ahOutK; float t = ahOut + y;
    ahOutK = (t - ahOut) - y; ahOut = t;
  }
}

void renderUi() {
  char b[64];
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

  if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "SETZERO") { sZeroMv = sCurMv; saveState(); }
      else if (cmd == "RESET") { ahOut = 0; ahOutK = 0; saveState(); }
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
