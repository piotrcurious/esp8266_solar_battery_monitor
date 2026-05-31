#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <algorithm>
#include <cstring>
#include <Preferences.h>

#ifndef clamp
#define clamp(v,lo,hi) (((v)<(lo))?(lo):((v)>(hi))?(hi):(v))
#endif

// ================================================================
//  Hardware
// ================================================================
static constexpr int TFT_SCK  = 18, TFT_MOSI = 23, TFT_MISO = -1;
static constexpr int TFT_DC   = 16, TFT_CS   = 5,  TFT_RST  = 17, TFT_BL = 4;
static constexpr int PIN_BAT_VOLT = 34, PIN_CUR_SENS = 35, PIN_BUTTON = 0;

static constexpr float CUR_POLARITY = 1.0f;   // flip to -1 if sense is backwards

// Calibration
static constexpr float BAT_V_OFFSET = 0.0f;   // add to measured pack voltage (V)

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

static constexpr MonitorConfig CFG = {
    7, 20.0f, 3.60f, 60.0f, 300.0f,
    (110000.0f + 10000.0f) / 10000.0f, 2.0f, 185.0f, 10.0f,
    0.08f, 0.01f, 0.12f, 0.002f,
    0.20f, 1.50f, 0.99f, 30000, 0.05f, 0.2f
};

// Derived tuning
static constexpr uint32_t UI_MS         = 100;
static constexpr uint32_t PAGE_MS       = 5000;
static constexpr float ALPHA_ZERO       = 0.005f;
static constexpr float ALPHA_LOAD_W     = 0.04f;
static constexpr float ALPHA_WDECAY     = 0.003f;
static constexpr float COULOMB_DEADBAND_A = 0.05f;
static constexpr uint32_t IDLE_STABLE_MS = 2000;
static constexpr float WMAX_COUL        = 0.80f;
static constexpr float WINC             = 0.0002f;

static float getRintSocFactor(float soc) {
  if (soc > 80.0f) return 1.0f;
  if (soc < 10.0f) return 1.5f;
  return 1.0f + (80.0f - soc) * (0.5f / 70.0f);
}

// dV/dt buffer: DVDT_N × UI_MS ms window
static constexpr int DVDT_N = 30;  // 3-second window

// Rint median filter buffer
static constexpr int RINT_MED_N = 7;

// History buffers
static constexpr int HIST_N = 40;
static float vHist[HIST_N] = {};
static float iHist[HIST_N] = {};
static int histIdx = 0;
static uint32_t lastHistMs = 0;

// ================================================================
//  LovyanGFX
// ================================================================
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
      c.pin_cs = TFT_CS; c.pin_rst = TFT_RST; c.pin_busy = -1;
      c.panel_width = 160; c.panel_height = 80;
      c.offset_x = 0; c.offset_y = 0; c.offset_rotation = 0;
      c.dummy_read_pixel = 8; c.dummy_read_bits = 1;
      c.readable = true; c.invert = false;
      c.rgb_order = false; c.dlen_16bit = false; c.bus_shared = false;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = TFT_BL; c.invert = false;
      c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

static LGFX        lcd;
static LGFX_Sprite canvas(&lcd);

// Forward declarations
static float getEffectiveCapAh();

// ================================================================
//  State
// ================================================================
enum class PackState : uint8_t { IDLE, CHARGING, DISCHARGING };

// Filtered ADC-side millivolts (at the ADC pin, after the divider)
static float sBatMv  = 0.0f;
static float sCurMv  = 0.0f;
static float sZeroMv = 0.0f;  // ACS712 zero-point (ADC-side mV)
static uint32_t idleMs = 0;

// Core electrical
static float vPack   = 0.0f;  // loaded terminal voltage (V)
static float iA      = 0.0f;  // current (A) – positive = discharge
static float tEst    = 25.0f; // estimated internal temp (C)
static float vRested = 0.0f;  // estimated open-circuit voltage (V)
static float vSag    = 0.0f;  // vRested − vPack (V)
static float rInt    = CFG.rint_fresh_mo / 1000.0f;
static float pW      = 0.0f;  // instantaneous power (W)
static PackState pState = PackState::IDLE;

// Per-cell
static float vCellLoad = 0.0f;
static float vCellRest = 0.0f;

// SOC trio
static float socOcv   = 0.0f;  // OCV-curve SOC from sag-corrected voltage
static float socCoul  = 0.0f;  // Coulomb-counted SOC (%)
static float socBlend = 0.0f;  // displayed blended SOC (%)
static float wCoul    = 0.0f;  // Coulomb blend weight [0 … WMAX_COUL]
static bool  socInit  = false; // one-shot Coulomb initialisation flag

static uint32_t sessionStartMs = 0;
static float    sessionStartSoc = 0.0f;
static float    sessionStartVoc = 0.0f;

// Session energy
static float ahOut = 0.0f, ahIn = 0.0f;
static float sessionMaxDod = 0.0f;
static float sessionMaxSag = 0.0f;
static float whOut = 0.0f, whIn = 0.0f;
static float whCruise = 0.0f, whActive = 0.0f, whBurst = 0.0f;
static float ahTotal = 0.0f; // Lifetime Ah for cycle counting
static float rtEff = 100.0f;
static float ahOutK = 0.0f, ahInK = 0.0f; // Kahan summation compensators
static float whOutK = 0.0f, whInK = 0.0f;

// Analytics
static float sohPct   = 100.0f; // state of health from R_int (%)
static float dvdtVps  = 0.0f;   // pack dV/dt (V/s)
static float avgLoadW = 0.0f;   // smoothed discharge power for ETA
static float peakIA   = 0.0f;   // peak discharge current (A)
static float peakPW   = 0.0f;   // peak discharge power (W)
static int   etaMin   = -1;     // estimated minutes remaining (-1 = unknown)

// dV/dt ring buffer (sampled every UI_MS ms)
static float vRing[DVDT_N] = {};
static int   vRingHead = 0, vRingCount = 0;

// Rint median buffer
static float rMedBuf[RINT_MED_N] = {};
static int   rMedHead = 0, rMedCount = 0;

// Timing
static uint32_t lastMeasUs = 0;
static uint32_t lastUiMs   = 0;
static uint32_t lastSaveMs = 0;
static uint32_t lastActMs  = 0;
static uint32_t pageMs     = 0;
static int      uiPage     = 0;
static bool     isDimmed   = false;
static bool     useImperial = false;

static Preferences prefs;

// ================================================================
//  Helpers
// ================================================================
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : x > hi ? hi : x;
}
static inline float lp(float p, float in, float a) {
  return p + a * (in - p);
}
static uint32_t adcAvgMv(int pin, int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; ++i) s += analogReadMilliVolts(pin);
  return s / (uint32_t)n;
}

// Piecewise-linear Li-ion OCV → SOC
static float socFromV(float v) {
  static constexpr struct { float v, s; } T[] = {
    {4.20f,100},{4.10f,95},{4.00f,88},{3.95f,84},{3.90f,78},
    {3.85f,70},{3.80f,62},{3.75f,54},{3.70f,45},{3.65f,36},
    {3.60f,28},{3.50f,16},{3.40f,8},{3.20f,0}
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

// Signed-safe colour blend (original had uint8_t wrap-around on dark channels)
static uint32_t blendCol(uint32_t a, uint32_t b, float t) {
  t = clampf(t, 0.0f, 1.0f);
  int ar=(a>>16)&0xFF, ag=(a>>8)&0xFF, ab=a&0xFF;
  int br=(b>>16)&0xFF, bg=(b>>8)&0xFF, bb=b&0xFF;
  return lcd.color888(
    (uint8_t)(ar + (int)((br-ar)*t + 0.5f)),
    (uint8_t)(ag + (int)((bg-ag)*t + 0.5f)),
    (uint8_t)(ab + (int)((bb-ab)*t + 0.5f)));
}
static uint32_t socCol(float s) {
  if (s < 15.0f) return blendCol(lcd.color888(255, 0, 0), lcd.color888(255, 100, 0), s / 15.0f);
  if (s < 40.0f) return blendCol(lcd.color888(255, 100, 0), lcd.color888(255, 255, 0), (s - 15.0f) / 25.0f);
  if (s < 70.0f) return blendCol(lcd.color888(255, 255, 0), lcd.color888(0, 255, 0), (s - 40.0f) / 30.0f);
  return blendCol(lcd.color888(0, 255, 0), lcd.color888(100, 255, 255), (s - 70.0f) / 30.0f);
}
static uint32_t dvdtCol(float d) {
  if (d < -0.050f) return lcd.color888(255, 60, 60);  // fast drain
  if (d < -0.008f) return lcd.color888(255,160, 40);  // normal drain
  if (d >  0.008f) return lcd.color888( 80,200,255);  // charging
  return                  lcd.color888(130,140,160);   // stable
}
static const char* dvdtLabel(float d) {
  if (d < -0.050f) return "DROP!";
  if (d < -0.008f) return "FALL ";
  if (d >  0.008f) return "RISE ";
  return                   "FLAT ";
}
static const char* stateStr() {
  switch (pState) {
    case PackState::DISCHARGING: return "DISCH";
    case PackState::CHARGING:    return "CHG  ";
    default:                     return "IDLE ";
  }
}

static void formatUnit(char* buf, size_t sz, float val, const char* unit, const char* milliUnit) {
    if (fabsf(val) < 1.0f) snprintf(buf, sz, "%.0f%s", val * 1000.0f, milliUnit);
    else                  snprintf(buf, sz, "%.2f%s", val, unit);
}

// ================================================================
//  Drawing primitives
// ================================================================
static void drawSegBar(int x, int y, int w, int h,
                       float soc, uint32_t fill, uint32_t border, uint32_t empty) {
  const int S=7, G=3, sw=(w-G*(S-1))/S;
  const float ps = 100.0f/S;

  // Background container
  canvas.fillRoundRect(x-2, y-2, w+4, h+4, 4, lcd.color888(20,20,30));

  for (int i = 0; i < S; ++i) {
    int sx = x + i*(sw+G);
    canvas.drawRoundRect(sx, y, sw, h, 3, border);
    canvas.fillRoundRect(sx+1, y+1, sw-2, h-2, 2, empty);

    float f = clampf((soc-i*ps)/ps, 0.0f, 1.0f);
    if (f > 0.0f) {
      int fh = (int)((h-2)*f + 0.5f);
      // Vertical fill
      uint32_t c = blendCol(lcd.color888(255,80,60), fill, f);
      canvas.fillRoundRect(sx+1, y+1+(h-2-fh), sw-2, fh, 2, c);

      // Top gloss/highlight for the segment if full
      if (f >= 1.0f) {
          canvas.fillRect(sx+2, y+2, sw-4, 2, blendCol(c, 0xFFFFFF, 0.3f));
      }

      // Charging animation: scan-line highlight
      if (pState == PackState::CHARGING) {
          int scanX = (millis() / 50) % (w + 20) - 10;
          if (scanX > i*(sw+G) && scanX < (i+1)*(sw+G)) {
              canvas.fillRect(sx+1, y+1, sw-2, h-2, blendCol(c, 0xFFFFFF, 0.4f));
          }
      }
    }
  }
}

static void drawSparkline(int x, int y, int w, int h, float* data, int n, uint32_t color, const char* unit = "") {
  if (n < 2) return;
  float minV = 999, maxV = -999;
  for (int i=0; i<n; i++) {
    if (data[i] < minV) minV = data[i];
    if (data[i] > maxV) maxV = data[i];
  }
  float span = maxV - minV;
  if (span < 0.01f) span = 1.0f;
  minV -= span * 0.1f; maxV += span * 0.1f; // 10% padding

  canvas.drawRect(x, y, w, h, lcd.color888(40,40,55));

  // Peak indicator
  int peakY = y + h - 1 - (int)((maxV - minV) / (maxV - minV) * (h-1)); // Should be y
  peakY = y;
  canvas.drawPixel(x+w-2, peakY, 0xFFFFFF);
  canvas.drawPixel(x+w-3, peakY, 0xFFFFFF);

  // Mid-line reference
  int midY = y + h/2;
  for (int i=x; i<x+w; i+=4) canvas.drawPixel(i, midY, lcd.color888(60,60,70));

  // Health color bands (left edge 2px)
  canvas.fillRect(x, y, 2, h, lcd.color888(255, 40, 40));
  canvas.fillRect(x, y, 2, (int)(h*0.7f), lcd.color888(255, 170, 30));
  canvas.fillRect(x, y, 2, (int)(h*0.3f), lcd.color888(60, 255, 90));

  // Range labels
  char lbl[16];
  canvas.setTextSize(0); canvas.setTextColor(lcd.color888(120,120,140));
  snprintf(lbl, sizeof(lbl), "%.1f%s", maxV, unit);
  canvas.setCursor(x+2, y+1); canvas.print(lbl);
  snprintf(lbl, sizeof(lbl), "%.1f%s", minV, unit);
  canvas.setCursor(x+2, y+h-7); canvas.print(lbl);
  canvas.setTextSize(1);

  for (int i=0; i<n-1; i++) {
    int x0 = x + (i * w) / (n-1);
    int x1 = x + ((i+1) * w) / (n-1);
    int y0 = y + h - 1 - (int)((data[i] - minV) / (maxV - minV) * (h-1));
    int y1 = y + h - 1 - (int)((data[i+1] - minV) / (maxV - minV) * (h-1));
    y0 = clamp(y0, y, y+h-1); y1 = clamp(y1, y, y+h-1);

    // Area fill (faint)
    uint32_t fillCol = blendCol(color, TFT_BLACK, 0.7f);
    canvas.drawLine(x0, y+h-1, x0, y0, fillCol);

    canvas.drawLine(x0, y0, x1, y1, color);
  }
}

static void drawPageDots(int x, int y, int total, int cur) {
  for (int i = 0; i < total; ++i) {
    int px = x + i*7;
    if (i==cur) canvas.fillRoundRect(px, y, 5, 2, 1, lcd.color888(200,200,240));
    else        canvas.drawRoundRect(px, y, 5, 2, 1, lcd.color888( 70, 70, 90));
  }
}

// ================================================================
//  Page 0 – Main gauges  (160×80)
//
//  y= 0-12   header band
//  y=14-29   big cell OCV (sz2) | I and P right column (sz1)
//  y=31-38   sag / R_int / dV/dt label
//  y=40-61   blended SOC bar (h=22)
//  y=63-70   SOC% / SoH% / ETA
//  y=72-79   session Ah / sag pill / page dots
// ================================================================
static void renderPage0() {
  char b[40];
  const uint32_t H = lcd.color888(14,14,24);

  // Header
  canvas.fillRoundRect(0,0,160,13,2,H);
  canvas.setTextSize(1);
  snprintf(b,sizeof(b),"%dS %.1fV", CFG.cells_s, vPack);
  canvas.setTextColor(socCol(socBlend), H);
  canvas.setCursor(3,3); canvas.print(b);

  snprintf(b,sizeof(b),"Eff:%2.0f%%", rtEff);
  canvas.setTextColor(lcd.color888(180,200,255), H);
  canvas.setCursor(66,3); canvas.print(b);

  snprintf(b,sizeof(b),"%s", stateStr());
  canvas.setTextColor(pState == PackState::IDLE ? lcd.color888(150,150,150) :
                     (pState == PackState::CHARGING ? lcd.color888(100,255,100) : lcd.color888(255,150,100)), H);
  canvas.setCursor(126,3); canvas.print(b);

  // Big cell rested voltage + right column
  canvas.setTextSize(2);
  canvas.setTextColor(socCol(socBlend), TFT_BLACK);
  snprintf(b,sizeof(b),"%.2fV", vCellRest);
  canvas.setCursor(3,14); canvas.print(b);

  // Load bar relative to ~20A
  float loadPct = clampf(fabsf(iA) / 20.0f, 0.0f, 1.0f);
  canvas.drawRect(3, 27, 72, 3, lcd.color888(40,40,60));
  canvas.fillRect(3, 27, (int)(72 * loadPct), 3, pState == PackState::CHARGING ? lcd.color888(100,255,100) : lcd.color888(255,100,50));

  // Peak load ghost marker
  float peakLoadPct = clampf(fabsf(peakIA) / 20.0f, 0.0f, 1.0f);
  canvas.fillRect(3 + (int)(72 * peakLoadPct) - 1, 27, 2, 3, lcd.color888(150,150,150));

  canvas.setTextSize(1);
  uint32_t loadCol = (fabsf(iA) > 15.0f) ? lcd.color888(255, 100, 50) : lcd.color888(180, 180, 200);
  canvas.setTextColor(loadCol, TFT_BLACK);
  snprintf(b,sizeof(b),"I %+.1fA", iA);
  canvas.setCursor(88,15); canvas.print(b);
  snprintf(b,sizeof(b),"P %5.0fW", fabsf(pW));
  canvas.setCursor(88,24); canvas.print(b);

  canvas.setTextSize(0);
  canvas.setTextColor(dvdtCol(dvdtVps), TFT_BLACK);
  canvas.setCursor(120,15); canvas.print(dvdtLabel(dvdtVps));
  canvas.setTextColor(lcd.color888(100,100,120), TFT_BLACK);
  snprintf(b,sizeof(b),"Pk:%.0fW", peakPW);
  canvas.setCursor(120,24); canvas.print(b);
  canvas.setTextSize(1);

  // Sag / R_int / dV/dt label
  char sS[16], sR[16];
  formatUnit(sS, sizeof(sS), vSag, "V", "mV");
  formatUnit(sR, sizeof(sR), rInt, "O", "mO");
  snprintf(b,sizeof(b),"%s %s", sS, sR);

  if (fabsf(dvdtVps) > 0.5f) {
      canvas.fillRect(2, 31, 64, 10, lcd.color888(60, 20, 20));
      canvas.setTextColor(lcd.color888(255,150,150));
  } else {
      canvas.setTextColor(lcd.color888(140,140,160), TFT_BLACK);
  }
  canvas.setCursor(3,32); canvas.print(b);

  // Blended SOC bar (single clean bar replaces original double-overlay)
  drawSegBar(3,44,154,18, socBlend,
    socCol(socBlend), lcd.color888(65,65,85), lcd.color888(12,12,18));

  // SOC / SoH / ETA
  canvas.setTextSize(1);
  snprintf(b,sizeof(b),"SOC:%.0f%%", socBlend);
  canvas.setTextColor(socCol(socBlend), TFT_BLACK);
  canvas.setCursor(3,63); canvas.print(b);

  snprintf(b,sizeof(b),"SoH:%.0f%%", sohPct);
  canvas.setTextColor(sohPct>70 ? lcd.color888(60,230,80) : lcd.color888(255
    ,150,30), TFT_BLACK);
  canvas.setCursor(57,63); canvas.print(b);

  if (etaMin >= 0) {
    int h=etaMin/60, m=etaMin%60;
    if (h>0) snprintf(b,sizeof(b),"%dh%dm",h,m);
    else     snprintf(b,sizeof(b),"%dm",m);
  } else snprintf(b,sizeof(b),"---");
  canvas.setTextColor(lcd.color888(100,190,255), TFT_BLACK);
  canvas.setCursor(108,63); canvas.print(b);

  float remWh = (socBlend/100.0f) * getEffectiveCapAh() * (float)CFG.cells_s * CFG.nom_v_cell;
  float dist = remWh * CFG.km_per_wh;
  if (useImperial) snprintf(b, sizeof(b), "%.1fmi", dist * 0.621371f);
  else             snprintf(b, sizeof(b), "%.1fkm", dist);
  canvas.setCursor(108, 72); canvas.setTextColor(lcd.color888(150, 255, 150)); canvas.print(b);

  // Sag Prediction (at reference load)
  float vPredRef = vRested - (CFG.ref_load_a * rInt * getRintSocFactor(socBlend));
  snprintf(b, sizeof(b), "Est @ %.0fA: %.1fV", CFG.ref_load_a, vPredRef);
  canvas.setTextColor(lcd.color888(120,120,140), TFT_BLACK);
  canvas.setCursor(68, 72);
  // Let's reorganize Page 0 a bit.

  // Ah / sag pill / page dots
  bool critical = (vCellLoad < 3.1f);
  if (critical && (millis() / 250) % 2 == 0) {
    snprintf(b,sizeof(b),"CRITICAL!");
    canvas.setTextColor(lcd.color888(255,255,255), lcd.color888(255,0,0));
  } else if (socBlend < 15.0f && (millis() / 500) % 2 == 0) {
    snprintf(b,sizeof(b),"LOW BAT!");
    canvas.setTextColor(lcd.color888(255,50,50), TFT_BLACK);
  } else {
    snprintf(b,sizeof(b),"Ah:%.2f", ahOut);
    canvas.setTextColor(lcd.color888(140,140,165), TFT_BLACK);
  }
  canvas.setCursor(3,72); canvas.print(b);

  uint32_t pillC = vSag>1.2f ? lcd.color888(255,40,40) :
                   vSag>0.5f ? lcd.color888(255,160,40) :
                               lcd.color888(60,230,80);
  canvas.fillRoundRect(126,71,16,8,4,pillC); // rounded pill
  if (vSag > 1.2f && (millis()/250)%2==0) canvas.drawRoundRect(126,71,16,8,4,lcd.color888(255,255,255));
  else                                     canvas.drawRoundRect(126,71,16,8,4,lcd.color888(100,100,100));
  drawPageDots(146, 75, 3, 0);
}

// ================================================================
//  Page 1 – Analytics
//
//  y= 0-12   header
//  y=14-29   ETA large (sz2)
//  y=31-38   @avg load + dV/dt
//  y=40-47   Ah out / in
//  y=49-56   Wh out / in
//  y=58-65   SoH / R_int / SOC
//  y=67-74   peak I and P
//  y=76-79   page dots
// ================================================================
static void renderPage1() {
  char b[48];
  const uint32_t H = lcd.color888(14,14,24);

  // Header
  canvas.fillRoundRect(0,0,160,13,2,H);
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(130,175,255), H);
  canvas.setCursor(3,3); canvas.print("Trends");
  snprintf(b,sizeof(b),"%.1fV %+.1fA", vPack, iA);
  canvas.setTextColor(lcd.color888(155,155,175), H);
  canvas.setCursor(80,3); canvas.print(b);

  // Voltage Sparkline
  float vPlot[HIST_N];
  for (int i=0; i<HIST_N; i++) vPlot[i] = vHist[(histIdx + i) % HIST_N];
  drawSparkline(3, 15, 75, 25, vPlot, HIST_N, lcd.color888(100, 200, 255), "V");
  canvas.setCursor(30, 41); canvas.setTextColor(lcd.color888(100, 150, 200)); canvas.print("Voltage");

  // Current Sparkline
  float iPlot[HIST_N];
  for (int i=0; i<HIST_N; i++) iPlot[i] = iHist[(histIdx + i) % HIST_N];
  drawSparkline(82, 15, 75, 25, iPlot, HIST_N, lcd.color888(255, 150, 100), "A");
  canvas.setCursor(105, 41); canvas.setTextColor(lcd.color888(200, 100, 50)); canvas.print("Current");

  // ETA and Load Info
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(180, 180, 200), TFT_BLACK);
  canvas.setCursor(3, 54);
  if (etaMin >= 0) {
    int h=etaMin/60, m=etaMin%60;
    if (h>0) snprintf(b,sizeof(b),"ETA %dh%02dm", h, m);
    else     snprintf(b,sizeof(b),"ETA %dm", m);
  } else snprintf(b,sizeof(b),"ETA stable");
  canvas.print(b);

  snprintf(b,sizeof(b),"Avg %.0fW", fabsf(avgLoadW));
  canvas.setCursor(100, 54); canvas.print(b);

  // Health summary
  canvas.setCursor(3, 65);
  canvas.setTextColor(lcd.color888(140, 140, 160), TFT_BLACK);
  canvas.print("Health ");
  canvas.setTextColor(sohPct>70?lcd.color888(60,230,80):lcd.color888(255,150,30), TFT_BLACK);
  snprintf(b,sizeof(b),"%.0f%% R:%.0fmO", sohPct, rInt*1000.0f);
  canvas.print(b);

  drawPageDots(146, 75, 3, 1);
}

static void renderPage4() {
  char b[48];
  const uint32_t H = lcd.color888(14,14,24);
  canvas.fillRoundRect(0,0,160,13,2,H);
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(150,255,150), H);
  canvas.setCursor(3,3); canvas.print("Battery Health Detail");

  canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK);
  canvas.setCursor(3, 18); canvas.print("SOH (Rint):");
  float sohRint = 100.0f * (CFG.rint_dead_mo - rInt * 1000.0f) / (CFG.rint_dead_mo - CFG.rint_fresh_mo);
  snprintf(b, sizeof(b), "%.0f%%", sohRint);
  canvas.setCursor(90, 18); canvas.setTextColor(lcd.color888(100,200,255)); canvas.print(b);

  canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK);
  canvas.setCursor(3, 28); canvas.print("SOH (Age) :");
  float cycles  = ahTotal / (CFG.cap_ah * 2.0f);
  float sohCycle = 100.0f - (cycles / 5.0f);
  snprintf(b, sizeof(b), "%.0f%%", sohCycle);
  canvas.setCursor(90, 28); canvas.setTextColor(lcd.color888(200,150,255)); canvas.print(b);

  canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK);
  canvas.setCursor(3, 42); canvas.print("Nominal Cap:");
  snprintf(b, sizeof(b), "%.1fAh", CFG.cap_ah);
  canvas.setCursor(90, 42); canvas.print(b);

  canvas.setCursor(3, 52); canvas.print("Effect. Cap:");
  snprintf(b, sizeof(b), "%.1fAh", getEffectiveCapAh());
  canvas.setCursor(90, 52); canvas.setTextColor(lcd.color888(100,255,100)); canvas.print(b);

  canvas.setCursor(3, 62); canvas.setTextColor(lcd.color888(180,180,200)); canvas.print("Max Sag:");
  formatUnit(b, sizeof(b), sessionMaxSag, "V", "mV");
  canvas.setCursor(90, 62); canvas.setTextColor(lcd.color888(255,150,50)); canvas.print(b);

  canvas.setCursor(3, 72); canvas.setTextColor(lcd.color888(180,180,200)); canvas.print("Est Temp:");
  snprintf(b, sizeof(b), "%.1f C", tEst);
  canvas.setCursor(90, 72); canvas.setTextColor(tEst > 50.0f ? lcd.color888(255,100,50) : lcd.color888(100,200,255)); canvas.print(b);

  drawPageDots(138, 75, 5, 4);
}

static void renderPage3() {
  char b[48];
  const uint32_t H = lcd.color888(14,14,24);
  canvas.fillRoundRect(0,0,160,13,2,H);
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(100,255,200), H);
  canvas.setCursor(3,3); canvas.print("Energy Distribution");

  auto drawBar = [](int y, const char* lbl, float wh, float total, uint32_t c) {
    char b[32];
    canvas.setTextColor(lcd.color888(180,180,200));
    canvas.setCursor(3, y); canvas.print(lbl);
    float pct = (total > 0.01f) ? (wh / total) : 0;
    canvas.drawRect(50, y, 60, 7, lcd.color888(40,40,50));
    canvas.fillRect(50, y, (int)(60*pct), 7, c);
    snprintf(b, sizeof(b), "%.1fWh", wh);
    canvas.setCursor(115, y); canvas.print(b);
  };

  float total = whCruise + whActive + whBurst;
  drawBar(18, "Cruise", whCruise, total, lcd.color888(100,200,255));
  drawBar(30, "Active", whActive, total, lcd.color888(100,255,100));
  drawBar(42, "Burst",  whBurst,  total, lcd.color888(255,100,50));

  canvas.setCursor(3, 56); canvas.setTextColor(lcd.color888(150,150,160));
  snprintf(b, sizeof(b), "Total Out: %.1f Wh", whOut);
  canvas.print(b);

  float distKm = ahOut * CFG.nom_v_cell * CFG.cells_s * CFG.km_per_wh;
  if (useImperial) snprintf(b, sizeof(b), "Avg: %.1f Wh/mi", (distKm > 0.1f) ? (whOut / (distKm * 0.621371f)) : 0);
  else             snprintf(b, sizeof(b), "Avg: %.1f Wh/km", (distKm > 0.1f) ? (whOut / distKm) : 0);
  canvas.setCursor(3, 65); canvas.print(b);

  drawPageDots(142, 75, 4, 3);
}

static void renderPage2() {
  char b[48];
  const uint32_t H = lcd.color888(14,14,24);

  // Header
  canvas.fillRoundRect(0,0,160,13,2,H);
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(255,200,100), H);
  canvas.setCursor(3,3); canvas.print("Session Summary");

  uint32_t sec = (millis() - sessionStartMs) / 1000;
  int h = sec / 3600, m = (sec / 60) % 60, s = sec % 60;
  snprintf(b, sizeof(b), "%02d:%02d:%02d", h, m, s);
  canvas.setTextColor(lcd.color888(150,150,150), H);
  canvas.setCursor(110, 3); canvas.print(b);

  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK);

  canvas.setCursor(3,18); canvas.print("Ah Out:");
  snprintf(b,sizeof(b),"%.2f Ah", ahOut);
  canvas.setCursor(80,18); canvas.setTextColor(lcd.color888(255,100,100), TFT_BLACK); canvas.print(b);

  canvas.setCursor(3,28); canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK); canvas.print("Ah In:");
  snprintf(b,sizeof(b),"%.2f Ah", ahIn);
  canvas.setCursor(80,28); canvas.setTextColor(lcd.color888(100,255,100), TFT_BLACK); canvas.print(b);

  canvas.setCursor(3,40); canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK); canvas.print("Wh Out:");
  snprintf(b,sizeof(b),"%.1f Wh", whOut);
  canvas.setCursor(80,40); canvas.setTextColor(lcd.color888(255,100,100), TFT_BLACK); canvas.print(b);

  canvas.setCursor(3,50); canvas.setTextColor(lcd.color888(180,180,200), TFT_BLACK); canvas.print("Wh In:");
  snprintf(b,sizeof(b),"%.1f Wh", whIn);
  canvas.setCursor(80,50); canvas.setTextColor(lcd.color888(100,255,100), TFT_BLACK); canvas.print(b);

  canvas.setCursor(3,65); canvas.setTextColor(lcd.color888(200,200,220), TFT_BLACK); canvas.print("Efficiency:");
  snprintf(b,sizeof(b),"%.1f%%", rtEff);
  canvas.setCursor(80,65); canvas.setTextColor(lcd.color888(100,200,255), TFT_BLACK); canvas.print(b);

  canvas.setCursor(3,73); canvas.setTextColor(lcd.color888(150,150,170), TFT_BLACK); canvas.print("Life Cycles:");
  float cycles = ahTotal / (CFG.cap_ah * 2.0f);
  snprintf(b,sizeof(b),"%.1f  (%+.1f%%)", cycles, socBlend - sessionStartSoc);
  canvas.setCursor(80,73); canvas.setTextColor(lcd.color888(200,150,255), TFT_BLACK); canvas.print(b);

  drawPageDots(146, 75, 3, 2);
}

// ================================================================
//  Measurement + inference  (called every UI_MS ms)
// ================================================================

static void sampleSensors(float dt) {
  // Adaptive ADC alpha: faster response during high load
  float aAdc = CFG.alpha_adc * (1.0f + clampf(fabsf(iA) / 10.0f, 0.0f, 2.0f));

  // ── ADC (16-point hardware average per pin)
  sBatMv = lp(sBatMv, (float)adcAvgMv(PIN_BAT_VOLT, 16), aAdc);
  sCurMv = lp(sCurMv, (float)adcAvgMv(PIN_CUR_SENS, 16), aAdc);

  // ACS712 zero-point: track only while effectively idle and stable
  if (sZeroMv < 1.0f) sZeroMv = sCurMv;
  if (fabsf(iA) < CFG.idle_a && fabsf(dvdtVps) < 0.02f) {
    idleMs += (uint32_t)(dt * 1000.0f);
    if (idleMs > IDLE_STABLE_MS) {
      sZeroMv = lp(sZeroMv, sCurMv, ALPHA_ZERO);
    }
  } else {
    idleMs = 0;
  }
}

// Forward declarations
static void saveState();
static float getEffectiveCapAh();

static void calculateElectrical() {
  vPack = (sBatMv * CFG.bat_div) / 1000.0f + BAT_V_OFFSET;

  // ADC Sanity checks
  if (vPack < 10.0f || vPack > 35.0f) vPack = clampf(vPack, 10.0f, 35.0f);

  iA    = CUR_POLARITY * (sCurMv - sZeroMv) * CFG.cur_div / CFG.acs_mv_a;
  pW    = vPack * iA;

  PackState oldState = pState;
  if      (iA >  CFG.idle_a) pState = PackState::DISCHARGING;
  else if (iA < -CFG.idle_a) pState = PackState::CHARGING;
  else                    pState = PackState::IDLE;

  if (pState != oldState && oldState != PackState::IDLE) {
    saveState(); // Save immediately when a session ends
  }

  vCellLoad = vPack / (float)CFG.cells_s;
}

static void updateVocEstimator() {
  if (vRested < 1.0f) vRested = vPack;
  if (pState == PackState::IDLE) {
    vRested = lp(vRested, vPack, CFG.alpha_rest_v);
  } else {
    float iSign = (pState == PackState::DISCHARGING) ? 1.0f : -1.0f;
    // Account for SOC-dependent Rint increase in the Voc estimation path
    float rCurrent = rInt * getRintSocFactor(socBlend);
    vRested = lp(vRested, vPack + iSign * fabsf(iA) * rCurrent, CFG.alpha_load_v);
  }
  vSag = fabsf(vRested - vPack);
  vCellRest = vRested / (float)CFG.cells_s;
  socOcv = socFromV(vCellRest);
}

static float getEffectiveCapAh() {
  return CFG.cap_ah * (0.7f + 0.3f * (sohPct / 100.0f));
}

static void updateThermal(float dt) {
    float powerLost = iA * iA * rInt;
    float deltaT = (powerLost - CFG.thermal_k * (tEst - 25.0f)) * (dt / 120.0f);
    tEst += deltaT;
}

static void updateRintEstimator() {
  static float lastIA_Rint = 0.0f;
  float dI = fabsf(iA - lastIA_Rint);
  lastIA_Rint = iA;

  if (fabsf(iA) > CFG.sag_min_a && dI < 0.2f) {
    float rEstRaw = vSag / fabsf(iA);
    // Normalize measured Rint to 100% SOC equivalent using the factor
    float rEst = rEstRaw / getRintSocFactor(socBlend);

    if (isfinite(rEst) && rEst > 0.010f && rEst < 0.800f) {
      if (fabsf(dvdtVps) < 0.1f) {
        rMedBuf[rMedHead] = rEst;
        rMedHead = (rMedHead + 1) % RINT_MED_N;
        if (rMedCount < RINT_MED_N) ++rMedCount;
        if (rMedCount >= 3) {
          float tmp[RINT_MED_N];
          memcpy(tmp, rMedBuf, rMedCount * sizeof(float));
          std::sort(tmp, tmp + rMedCount);

          // Adaptive Rint alpha: faster updates at higher currents (higher SNR)
          float aRint = CFG.alpha_rint * (1.0f + clampf(fabsf(iA) / 5.0f, 0.0f, 3.0f));
          rInt = lp(rInt, tmp[rMedCount / 2], aRint);
        }
      }
    }
  }

  // Refined SOH: Weighted combination of Rint health and cycle-based aging.
  float sohRint = 100.0f * (CFG.rint_dead_mo - rInt * 1000.0f) / (CFG.rint_dead_mo - CFG.rint_fresh_mo);
  float cycles  = ahTotal / (CFG.cap_ah * 2.0f);
  float sohCycle = 100.0f - (cycles / 5.0f); // Assume ~500 cycles EOL (simplified aging model)

  sohPct = clampf(0.8f * sohRint + 0.2f * sohCycle, 0.0f, 100.0f);
}

static float learnedCapAh = CFG.cap_ah;

static void resetSession() {
  // Capacity learning if session was deep enough
  if (sessionMaxDod > 50.0f && ahOut > 1.0f) {
      float socDelta = socFromV(sessionStartVoc / CFG.cells_s) - socFromV(vRested / CFG.cells_s);
      if (socDelta > 20.0f) {
          float estFullCap = ahOut / (socDelta / 100.0f);
          if (estFullCap > CFG.cap_ah * 0.5f && estFullCap < CFG.cap_ah * 1.5f) {
              learnedCapAh = lp(learnedCapAh, estFullCap, 0.1f);
          }
      }
  }

  ahOut = ahIn = whOut = whIn = 0.0f;
  whCruise = whActive = whBurst = 0.0f;
  sessionMaxDod = 0.0f;
  sessionMaxSag = 0.0f;
  ahOutK = ahInK = whOutK = whInK = 0.0f;
  peakIA = peakPW = 0.0f;
  sessionStartMs = millis();
  sessionStartSoc = socBlend;
  sessionStartVoc = vRested;
  saveState();
}

static void updateSoc(float dt) {
  if (!socInit) { socCoul = socOcv; wCoul = 0.0f; socInit = true; }
  float iInt = (fabsf(iA) < COULOMB_DEADBAND_A) ? 0.0f : iA;

  // Apply charge efficiency factor
  float eff = (iInt < 0.0f) ? CFG.charge_eff : 1.0f;

  socCoul -= (iInt * eff * dt / 3600.0f / getEffectiveCapAh()) * 100.0f;
  socCoul  = clampf(socCoul, 0.0f, 100.0f);
  if (pState == PackState::IDLE && fabsf(dvdtVps) < 0.01f) {
    socCoul = lp(socCoul, socOcv, 0.001f);
  }
  // Hard-sync to 100% when voltage is very high and charging is finished
  if (pState == PackState::CHARGING && vCellRest > 4.15f && fabsf(iA) < 0.3f) {
    socCoul = lp(socCoul, 100.0f, 0.01f);
    // Auto-reset session if we are full and significant discharge occurred
    if (socCoul > 99.5f && sessionMaxDod > 20.0f) {
      resetSession();
    }
  }
  wCoul = (pState != PackState::IDLE) ? clampf(wCoul + WINC, 0.0f, WMAX_COUL) : lp(wCoul, 0.0f, ALPHA_WDECAY);
  socBlend = (1.0f - wCoul) * socOcv + wCoul * socCoul;

  float currentDod = 100.0f - socBlend;
  if (currentDod > sessionMaxDod) sessionMaxDod = currentDod;
}

static void saveState() {
  prefs.begin("sag_gauge", false);
  prefs.putFloat("ahOut", ahOut);
  prefs.putFloat("ahIn", ahIn);
  prefs.putFloat("whOut", whOut);
  prefs.putFloat("whIn", whIn);
  prefs.putFloat("ahTotal", ahTotal);
  prefs.putFloat("learnedCapAh", learnedCapAh);
  prefs.putFloat("rInt", rInt);
  prefs.putFloat("sZeroMv", sZeroMv);
  prefs.end();
}

static void loadState() {
  prefs.begin("sag_gauge", true);
  ahOut = prefs.getFloat("ahOut", 0.0f);
  ahIn = prefs.getFloat("ahIn", 0.0f);
  whOut = prefs.getFloat("whOut", 0.0f);
  whIn = prefs.getFloat("whIn", 0.0f);
  ahTotal = prefs.getFloat("ahTotal", 0.0f);
  learnedCapAh = prefs.getFloat("learnedCapAh", CFG.cap_ah);
  rInt = prefs.getFloat("rInt", CFG.rint_fresh_mo / 1000.0f);
  sZeroMv = prefs.getFloat("sZeroMv", 0.0f);
  prefs.end();
}

static void integrateEnergy(float dt) {
  float dAh = fabsf(iA) * (dt / 3600.0f);
  float dWh = fabsf(pW) * (dt / 3600.0f);

  static float ahTotalK = 0.0f;
  auto kahanAdd = [](float &sum, float &c, float input) {
    float y = input - c; float t = sum + y;
    c = (t - sum) - y; sum = t;
  };
  if (pState == PackState::DISCHARGING) {
    kahanAdd(ahOut, ahOutK, dAh); kahanAdd(whOut, whOutK, dWh);
    kahanAdd(ahTotal, ahTotalK, dAh);

    // Distribution
    if (pW < 50.0f) whCruise += dWh;
    else if (pW < 200.0f) whActive += dWh;
    else whBurst += dWh;

    if (iA > peakIA) peakIA = iA; if (pW > peakPW) peakPW = pW;
  } else if (pState == PackState::CHARGING) {
    kahanAdd(ahIn, ahInK, dAh); kahanAdd(whIn, whInK, dWh);
    kahanAdd(ahTotal, ahTotalK, dAh);
  }
  if (whIn > 0.1f) rtEff = clampf(whOut / whIn * 100.0f, 0.0f, 100.0f);
  avgLoadW = lp(avgLoadW, pW, ALPHA_LOAD_W);
  if (pState == PackState::DISCHARGING && avgLoadW > 2.0f) {
    float remWh = (socBlend/100.0f) * getEffectiveCapAh() * (float)CFG.cells_s * CFG.nom_v_cell;
    etaMin = (int)clampf(remWh / avgLoadW * 60.0f, 0.0f, 9999.0f);
  } else if (pState == PackState::CHARGING && avgLoadW < -2.0f) {
    float needWh = (1.0f - socBlend/100.0f) * getEffectiveCapAh() * (float)CFG.cells_s * CFG.nom_v_cell;
    float chgW = fabsf(avgLoadW);
    // CV phase compensation: current tapers, so ETA is longer than linear.
    if (vCellRest > 4.10f) chgW *= 0.5f;
    etaMin = (int)clampf(needWh / chgW * 60.0f, 0.0f, 9999.0f);
  } else {
    etaMin = -1;
    if (pState == PackState::IDLE) avgLoadW = lp(avgLoadW, 0.0f, 0.01f);
  }
}

static void updateDvdt() {
  vRing[vRingHead] = vPack;
  vRingHead = (vRingHead + 1) % DVDT_N;
  if (vRingCount < DVDT_N) ++vRingCount;
  if (vRingCount >= 2) {
    int oldest = (vRingHead - vRingCount + DVDT_N) % DVDT_N;
    float ws   = (float)(vRingCount - 1) * (UI_MS / 1000.0f);
    dvdtVps    = (vRing[(vRingHead-1+DVDT_N)%DVDT_N] - vRing[oldest]) / ws;
  }
}

static void updateMeasurements() {
  uint32_t nowUs = micros();
  float dt = (lastMeasUs == 0) ? (UI_MS / 1000.0f) : clampf((float)(nowUs - lastMeasUs) * 1e-6f, 0.001f, 1.0f);
  lastMeasUs = nowUs;

  sampleSensors(dt);
  calculateElectrical();
  updateVocEstimator();
  updateRintEstimator();
  updateSoc(dt);
  integrateEnergy(dt);
  updateThermal(dt);
  updateDvdt();

  // Update history buffers every 5 seconds
  uint32_t now = millis();
  if (lastHistMs == 0 || now - lastHistMs > 5000) {
    lastHistMs = now;
    vHist[histIdx] = vPack;
    iHist[histIdx] = iA;
    histIdx = (histIdx + 1) % HIST_N;
  }
}

// ================================================================
//  Setup
// ================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_VOLT, ADC_11db);
  analogSetPinAttenuation(PIN_CUR_SENS, ADC_11db);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  lcd.init();
  lcd.setRotation(0);
  lcd.setBrightness(200);
  lcd.setColorDepth(16);

  canvas.setColorDepth(16);
  canvas.createSprite(160, 80);
  canvas.fillScreen(TFT_BLACK);

  loadState();

  // Boot calibration – stable 64-sample average before any load
  delay(300);
  sBatMv  = (float)adcAvgMv(PIN_BAT_VOLT, 32);
  float sCurMvBoot = (float)adcAvgMv(PIN_CUR_SENS, 64);

  // Use loaded sZeroMv if it looks valid, otherwise calibrate from boot
  if (sZeroMv < 100.0f) sZeroMv = sCurMvBoot;
  sCurMv = sCurMvBoot;

  vPack     = (sBatMv * CFG.bat_div) / 1000.0f;
  vRested   = vPack;
  vCellRest = vPack / (float)CFG.cells_s;
  socOcv    = socFromV(vCellRest);
  socBlend  = socOcv;
  // socInit = false  →  Coulomb SOC initialises from OCV on first tick

  for (int i=0; i<HIST_N; i++) vHist[i] = vPack;
  sessionStartMs = millis();
  sessionStartSoc = socBlend;
  sessionStartVoc = vRested;
  lastUiMs = pageMs = millis();
}

// ================================================================
//  Loop – all work gated to UI_MS cadence
// ================================================================
void loop() {
  uint32_t now = millis();

  // Button handling
  static bool lastBtn = true;
  static uint32_t btnDownMs = 0;
  bool btn = digitalRead(PIN_BUTTON);
  if (!btn && lastBtn) { btnDownMs = now; }
  if (btn && !lastBtn) {
    uint32_t dur = now - btnDownMs;
    lastActMs = now;
    if (isDimmed) {
        lcd.setBrightness(200);
        isDimmed = false;
    } else {
        if (dur > 5000) {
            prefs.begin("sag_gauge", false);
            prefs.clear();
            prefs.end();
            ESP.restart();
        }
        else if (dur > 2000) resetSession();
        else if (dur > 50) { uiPage = (uiPage + 1) % 5; pageMs = now; }
    }
  }
  lastBtn = btn;

  // Auto-dimming
  if (!isDimmed && now - lastActMs > CFG.dim_ms && pState == PackState::IDLE) {
      if (now - lastActMs > CFG.dim_ms * 10) lcd.setBrightness(0); // Auto-off after 10x dim period
      else lcd.setBrightness(20);
      isDimmed = true;
  }
  if (isDimmed && pState != PackState::IDLE) {
      lcd.setBrightness(200);
      isDimmed = false;
      lastActMs = now;
  }

  // Non-blocking auto page flip (only if button not used recently)
  if (!isDimmed && now - pageMs >= PAGE_MS) {
    uiPage = (uiPage + 1) % 5;
    pageMs  = now;
  }

  // Serial command parser
  if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "RESET") resetSession();
      else if (cmd == "SETZERO") { sZeroMv = sCurMv; saveState(); }
      else if (cmd == "UNITS") useImperial = !useImperial;
      else if (cmd.startsWith("SETCAP ")) {
          float newCap = cmd.substring(7).toFloat();
          if (newCap > 1.0f) {
              // This requires a non-const config, but for now we just log it
              Serial.println("Manual cap update not yet supported via serial");
          }
      }
  }

  // Sample + render at fixed cadence (fixes Coulomb integration rate and dV/dt)
  if (now - lastUiMs >= UI_MS) {
    lastUiMs = now;
    updateMeasurements();

    canvas.fillScreen(TFT_BLACK);
    if      (uiPage == 0) renderPage0();
    else if (uiPage == 1) renderPage1();
    else if (uiPage == 2) renderPage2();
    else if (uiPage == 3) renderPage3();
    else                  renderPage4();
    canvas.pushSprite(&lcd, 0, 0);

  // Periodic save (1 min)
  if (now - lastSaveMs > 60000) {
    lastSaveMs = now;
    saveState();
  }

    Serial.printf(
      "TELE:%.2f,%+.2f,%.1f,%.1f,%.1f,%.0f,%d\n",
      vPack, iA, pW, socBlend, rInt*1000.0f, sohPct, etaMin);

    Serial.printf(
      "STAT: soc=%.0f%%(ocv=%.0f%%,cc=%.0f%%) | "
      "soh=%.0f%% R=%.0fmO sag=%.0fmV | "
      "eta=%dm Ah=%.2f/%.2f Wh=%.1f/%.1f | "
      "dV/dt=%+.1fmV/s peak=%.1fA/%.0fW\n",
      socBlend, socOcv, socCoul,
      sohPct, rInt*1000.0f, vSag*1000.0f,
      etaMin, ahOut, ahIn, whOut, whIn,
      dvdtVps*1000.0f, peakIA, peakPW);

  }
}


//A few tuning notes:
//
//- **`CAP_AH`** is the most impactful parameter — size it correctly for accurate ETA and Coulomb SOC.
//- **`RINT_FRESH_MOHM`** should be measured from a freshly charged pack at room temperature (vSag ÷ iA at a known steady load).
//- **`WINC`** controls how quickly the display shifts from trusting OCV to trusting coulomb counting. At 10 Hz it currently ramps to full Coulomb trust in ~7 minutes of continuous load.
//- **`ALPHA_WDECAY`** controls the reverse — how quickly it snaps back to OCV trust after a rest. Currently ~2.5 minutes to essentially full OCV mode.
