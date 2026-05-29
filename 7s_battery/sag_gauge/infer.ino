#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>

// ================================================================
//  Hardware
// ================================================================
static constexpr int TFT_SCK  = 18, TFT_MOSI = 23, TFT_MISO = -1;
static constexpr int TFT_DC   = 16, TFT_CS   = 5,  TFT_RST  = 17, TFT_BL = 4;
static constexpr int PIN_BAT_VOLT = 34, PIN_CUR_SENS = 35;

// Voltage divider (110 k / 10 k → ×12.0; 29.4 V → 2.45 V at ADC)
static constexpr float BAT_R_TOP = 110000.0f, BAT_R_BOT = 10000.0f;
static constexpr float BAT_DIV   = (BAT_R_TOP + BAT_R_BOT) / BAT_R_BOT;

// ACS712 half-divider (10 k / 10 k, keeps 5 V sensor output safe for ESP32 ADC)
static constexpr float CUR_R_TOP = 10000.0f, CUR_R_BOT = 10000.0f;
static constexpr float CUR_DIV   = (CUR_R_TOP + CUR_R_BOT) / CUR_R_BOT;

// ACS712 variant: 5 A → 185 mV/A | 20 A → 100 mV/A | 30 A → 66 mV/A
static constexpr float ACS712_MV_A  = 185.0f;
static constexpr float CUR_POLARITY = 1.0f;   // flip to -1 if sense is backwards

// ================================================================
//  Pack parameters  ← configure for your battery
// ================================================================
static constexpr int   CELLS_S         = 7;       // series cell count
static constexpr float CAP_AH          = 20.0f;   // nominal capacity (Ah)
static constexpr float NOM_V_CELL      = 3.60f;   // nominal per-cell voltage (V)
static constexpr float RINT_FRESH_MOHM = 60.0f;   // R_int, new pack  (mΩ)
static constexpr float RINT_DEAD_MOHM  = 300.0f;  // R_int, end-of-life (mΩ)

// ================================================================
//  Tuning
// ================================================================
static constexpr uint32_t UI_MS         = 100;    // sample + render period (ms)
static constexpr uint32_t PAGE_MS       = 5000;   // auto page-flip period (ms)

static constexpr float ALPHA_ADC        = 0.08f;
static constexpr float ALPHA_ZERO       = 0.005f; // ACS zero-point drift
static constexpr float ALPHA_REST_V     = 0.12f;  // vRested convergence at idle
static constexpr float ALPHA_LOAD_V     = 0.002f; // vRested drift under load
static constexpr float ALPHA_RINT       = 0.01f;
static constexpr float ALPHA_LOAD_W     = 0.04f;  // ETA smoother (~25 s τ)
static constexpr float ALPHA_WDECAY     = 0.003f; // Coulomb blend decay at rest

static constexpr float IDLE_A           = 0.20f;  // |I| ≤ this → IDLE
static constexpr float SAG_MIN_A        = 0.70f;  // |I| ≥ this → update R_int

static constexpr float WMAX_COUL        = 0.80f;  // max Coulomb SOC weight
static constexpr float WINC             = 0.0002f;// blend weight ramp per tick (~7 min to max)

// dV/dt buffer: DVDT_N × UI_MS ms window
static constexpr int DVDT_N = 30;  // 3-second window

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

// ================================================================
//  State
// ================================================================
enum class PackState : uint8_t { IDLE, CHARGING, DISCHARGING };

// Filtered ADC-side millivolts (at the ADC pin, after the divider)
static float sBatMv  = 0.0f;
static float sCurMv  = 0.0f;
static float sZeroMv = 0.0f;  // ACS712 zero-point (ADC-side mV)

// Core electrical
static float vPack   = 0.0f;  // loaded terminal voltage (V)
static float iA      = 0.0f;  // current (A) – positive = discharge
static float vRested = 0.0f;  // estimated open-circuit voltage (V)
static float vSag    = 0.0f;  // vRested − vPack (V)
static float rInt    = RINT_FRESH_MOHM / 1000.0f;
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

// Session energy
static float ahOut = 0.0f, ahIn = 0.0f;
static float whOut = 0.0f, whIn = 0.0f;
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

// Timing
static uint32_t lastMeasUs = 0;
static uint32_t lastUiMs   = 0;
static uint32_t pageMs     = 0;
static int      uiPage     = 0;

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
  if (s<20) return lcd.color888(255, 40, 40);
  if (s<50) return lcd.color888(255,170, 30);
  if (s<80) return lcd.color888(180,220, 40);
  return           lcd.color888( 60,255, 90);
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

// ================================================================
//  Drawing primitives
// ================================================================
static void drawSegBar(int x, int y, int w, int h,
                       float soc, uint32_t fill, uint32_t border, uint32_t empty) {
  const int S=7, G=2, sw=(w-G*(S-1))/S;
  const float ps = 100.0f/S;
  for (int i = 0; i < S; ++i) {
    int sx = x + i*(sw+G);
    canvas.drawRoundRect(sx, y, sw, h, 2, border);
    canvas.fillRoundRect(sx+1, y+1, sw-2, h-2, 2, empty);
    float f = clampf((soc-i*ps)/ps, 0.0f, 1.0f);
    if (f > 0.0f) {
      int fh = (int)((h-2)*f + 0.5f);
      canvas.fillRoundRect(sx+1, y+1+(h-2-fh), sw-2, fh, 2,
        blendCol(lcd.color888(255,60,40), fill, f));
    }
  }
}

static void drawPageDots(int x0, int x1, int y, int cur) {
  for (int i = 0; i < 2; ++i) {
    int px = (i==0)?x0:x1;
    if (i==cur) canvas.fillCircle(px, y, 2, lcd.color888(200,200,240));
    else        canvas.drawCircle(px, y, 2, lcd.color888( 70, 70, 90));
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
  snprintf(b,sizeof(b),"%dS  %.1fV", CELLS_S, vPack);
  canvas.setTextColor(socCol(socBlend), H);
  canvas.setCursor(3,3); canvas.print(b);
  snprintf(b,sizeof(b),"%.0fW  %s", fabsf(pW), stateStr());
  canvas.setTextColor(lcd.color888(195,195,215), H);
  canvas.setCursor(93,3); canvas.print(b);

  // Big cell rested voltage + right column
  canvas.setTextSize(2);
  canvas.setTextColor(socCol(socBlend), TFT_BLACK);
  snprintf(b,sizeof(b),"%.2fV", vCellRest);
  canvas.setCursor(3,14); canvas.print(b);

  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(175,175,195), TFT_BLACK);
  snprintf(b,sizeof(b),"I:%+.1fA", iA);
  canvas.setCursor(84,15); canvas.print(b);
  snprintf(b,sizeof(b),"P: %.1fW", fabsf(pW));
  canvas.setCursor(84,24); canvas.print(b);

  // Sag / R_int / dV/dt label
  snprintf(b,sizeof(b),"Sag:%.0fmV  R:%.0fmO ", vSag*1000.0f, rInt*1000.0f);
  canvas.setTextColor(lcd.color888(140,140,160), TFT_BLACK);
  canvas.setCursor(3,31); canvas.print(b);
  canvas.setTextColor(dvdtCol(dvdtVps), TFT_BLACK);
  canvas.print(dvdtLabel(dvdtVps));

  // Blended SOC bar (single clean bar replaces original double-overlay)
  drawSegBar(3,40,154,22, socBlend,
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
    if (h>0) snprintf(b,sizeof(b),"~%dh%dm",h,m);
    else     snprintf(b,sizeof(b),"~%dm",m);
  } else snprintf(b,sizeof(b),"~---");
  canvas.setTextColor(lcd.color888(100,190,255), TFT_BLACK);
  canvas.setCursor(108,63); canvas.print(b);

  // Ah / sag pill / page dots
  snprintf(b,sizeof(b),"Ah:%.2f", ahOut);
  canvas.setTextColor(lcd.color888(140,140,165), TFT_BLACK);
  canvas.setCursor(3,72); canvas.print(b);

  uint32_t pillC = vSag>0.8f ? lcd.color888(255,60,60) :
                   vSag>0.3f ? lcd.color888(255,160,40) :
                               lcd.color888(60,230,80);
  canvas.fillRoundRect(126,71,16,8,3,pillC);
  drawPageDots(149,156,75, 0);
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
  canvas.setCursor(3,3); canvas.print("Analytics");
  snprintf(b,sizeof(b),"%.1fV %s", vPack, stateStr());
  canvas.setTextColor(lcd.color888(155,155,175), H);
  canvas.setCursor(88,3); canvas.print(b);

  // ETA (large)
  canvas.setTextSize(2);
  if (etaMin >= 0) {
    int h=etaMin/60, m=etaMin%60;
    if (h>0) snprintf(b,sizeof(b),"%dh%02dm",h,m);
    else     snprintf(b,sizeof(b),"%dm left",m);
    canvas.setTextColor(lcd.color888(90,210,255), TFT_BLACK);
  } else {
    snprintf(b,sizeof(b),"-- left");
    canvas.setTextColor(lcd.color888(65,65,85), TFT_BLACK);
  }
  canvas.setCursor(3,14); canvas.print(b);

  // @avg load / dV/dt
  canvas.setTextSize(1);
  snprintf(b,sizeof(b),"@%.1fW avg  dV:%.0fmV/s %s",
           avgLoadW, dvdtVps*1000.0f, dvdtLabel(dvdtVps));
  canvas.setTextColor(dvdtCol(dvdtVps), TFT_BLACK);
  canvas.setCursor(3,31); canvas.print(b);

  // Ah out / in
  canvas.setTextColor(lcd.color888(100,100,120), TFT_BLACK);
  canvas.setCursor(3,40); canvas.print("Ah ");
  snprintf(b,sizeof(b),"%.2f out", ahOut);
  canvas.setTextColor(lcd.color888(255,120,80), TFT_BLACK); canvas.print(b);
  snprintf(b,sizeof(b),"  %.2f in", ahIn);
  canvas.setTextColor(lcd.color888(80,200,255), TFT_BLACK); canvas.print(b);

  // Wh out / in
  canvas.setTextColor(lcd.color888(100,100,120), TFT_BLACK);
  canvas.setCursor(3,49); canvas.print("Wh ");
  snprintf(b,sizeof(b),"%.1f out", whOut);
  canvas.setTextColor(lcd.color888(255,120,80), TFT_BLACK); canvas.print(b);
  snprintf(b,sizeof(b),"  %.1f in", whIn);
  canvas.setTextColor(lcd.color888(80,200,255), TFT_BLACK); canvas.print(b);

  // SoH / R_int
  canvas.setCursor(3,58);
  canvas.setTextColor(lcd.color888(100,100,120), TFT_BLACK);
  canvas.print("SoH:");
  canvas.setTextColor(sohPct>70?lcd.color888(60,230,80):lcd.color888(255,150,30), TFT_BLACK);
  snprintf(b,sizeof(b),"%.0f%%  R:%.0fmO  SOC:%.0f%%",
           sohPct, rInt*1000.0f, socBlend);
  canvas.print(b);

  // Peaks
  canvas.setCursor(3,67);
  canvas.setTextColor(lcd.color888(100,100,120), TFT_BLACK);
  canvas.print("Peak ");
  snprintf(b,sizeof(b),"%.1fA  %.0fW", peakIA, peakPW);
  canvas.setTextColor(lcd.color888(210,170,255), TFT_BLACK); canvas.print(b);

  drawPageDots(149,156,76, 1);
}

// ================================================================
//  Measurement + inference  (called every UI_MS ms)
// ================================================================
static void updateMeasurements() {
  // Accurate dt from micros() – not a fixed constant
  uint32_t nowUs = micros();
  float dt = (lastMeasUs == 0)
    ? (UI_MS / 1000.0f)
    : clampf((float)(nowUs - lastMeasUs) * 1e-6f, 0.001f, 1.0f);
  lastMeasUs = nowUs;

  // ── ADC (16-point hardware average per pin)
  sBatMv = lp(sBatMv, (float)adcAvgMv(PIN_BAT_VOLT, 16), ALPHA_ADC);
  sCurMv = lp(sCurMv, (float)adcAvgMv(PIN_CUR_SENS, 16), ALPHA_ADC);

  // ACS712 zero-point: track only while effectively idle
  if (sZeroMv < 1.0f) sZeroMv = sCurMv;
  if (fabsf(iA) < IDLE_A)
    sZeroMv = lp(sZeroMv, sCurMv, ALPHA_ZERO);

  // ── Electrical conversion
  vPack = (sBatMv * BAT_DIV) / 1000.0f;
  iA    = CUR_POLARITY * (sCurMv - sZeroMv) * CUR_DIV / ACS712_MV_A;
  pW    = vPack * iA;

  // ── Pack state
  if      (iA >  IDLE_A) pState = PackState::DISCHARGING;
  else if (iA < -IDLE_A) pState = PackState::CHARGING;
  else                    pState = PackState::IDLE;

  // ── Rested-voltage estimator
  //    Under load: Voc ≈ Vterminal + sign(I)·|I|·R_int
  //    Sign flips for charging so the estimator stays consistent.
  if (vRested < 1.0f) vRested = vPack;
  if (pState == PackState::IDLE) {
    vRested = lp(vRested, vPack, ALPHA_REST_V);
  } else {
    float iSign = (pState == PackState::DISCHARGING) ? 1.0f : -1.0f;
    vRested = lp(vRested, vPack + iSign * fabsf(iA) * rInt, ALPHA_LOAD_V);
  }

  // ── Sag and R_int update
  // vSag is absolute deviation from rested voltage
  vSag = fabsf(vRested - vPack);
  if (fabsf(iA) > SAG_MIN_A) {
    float rEst = vSag / fabsf(iA);
    if (isfinite(rEst) && rEst > 0.010f && rEst < 0.800f) {
      // Only update Rint if the voltage has stabilized somewhat (dV/dt small)
      // and we have a significant current to avoid noise-floor issues.
      if (fabsf(dvdtVps) < 0.1f && fabsf(iA) > 1.5f) {
        rInt = lp(rInt, rEst, ALPHA_RINT);
      }
    }
  }

  // ── Per-cell
  vCellLoad = vPack   / (float)CELLS_S;
  vCellRest = vRested / (float)CELLS_S;
  socOcv    = socFromV(vCellRest);

  // ── Coulomb-counting SOC
  //    Initialised once from OCV; re-anchored toward OCV while resting.
  if (!socInit) { socCoul = socOcv; wCoul = 0.0f; socInit = true; }
  socCoul -= (iA * dt / 3600.0f / CAP_AH) * 100.0f;   // +I = discharge
  socCoul  = clampf(socCoul, 0.0f, 100.0f);
  if (pState == PackState::IDLE) {
    // slow OCV re-anchor only if voltage is stable
    if (fabsf(dvdtVps) < 0.01f) {
      socCoul = lp(socCoul, socOcv, 0.001f);
    }
  }

  // Blend weight: ramp during active use, decay back to OCV at rest
  wCoul = (pState != PackState::IDLE)
    ? clampf(wCoul + WINC, 0.0f, WMAX_COUL)
    : lp(wCoul, 0.0f, ALPHA_WDECAY);

  socBlend = (1.0f - wCoul)*socOcv + wCoul*socCoul;

  // ── Energy accounting (Kahan Summation)
  float dAh = fabsf(iA) * (dt / 3600.0f);
  float dWh = fabsf(pW) * (dt / 3600.0f);

  auto kahanAdd = [](float &sum, float &c, float input) {
    float y = input - c;
    float t = sum + y;
    c = (t - sum) - y;
    sum = t;
  };

  if (pState == PackState::DISCHARGING) {
    kahanAdd(ahOut, ahOutK, dAh);
    kahanAdd(whOut, whOutK, dWh);
    if (iA > peakIA) peakIA = iA;
    if (pW > peakPW) peakPW = pW;
  } else if (pState == PackState::CHARGING) {
    kahanAdd(ahIn, ahInK, dAh);
    kahanAdd(whIn, whInK, dWh);
  }

  // ── Smoothed load power (stabilises ETA against transient spikes)
  avgLoadW = (pState == PackState::DISCHARGING)
    ? lp(avgLoadW, pW, ALPHA_LOAD_W)
    : lp(avgLoadW, 0.0f, 0.01f);

  // ── Estimated time remaining
  if (avgLoadW > 5.0f) {
    float remWh = (socBlend/100.0f) * CAP_AH * (float)CELLS_S * NOM_V_CELL;
    etaMin = (int)clampf(remWh / avgLoadW * 60.0f, 0.0f, 9999.0f);
  } else {
    etaMin = -1;
  }

  // ── State of Health (linear R_int degradation model)
  //    SoH = 100% at RINT_FRESH, 0% at RINT_DEAD
  float rMo  = rInt * 1000.0f;
  sohPct = clampf(100.0f*(RINT_DEAD_MOHM-rMo)/(RINT_DEAD_MOHM-RINT_FRESH_MOHM),
                  0.0f, 100.0f);

  // ── dV/dt  (rolling window, UI_MS-spaced samples)
  vRing[vRingHead] = vPack;
  vRingHead = (vRingHead + 1) % DVDT_N;
  if (vRingCount < DVDT_N) ++vRingCount;
  if (vRingCount >= 2) {
    int oldest = (vRingHead - vRingCount + DVDT_N) % DVDT_N;
    float ws   = (float)(vRingCount - 1) * (UI_MS / 1000.0f);
    dvdtVps    = (vRing[(vRingHead-1+DVDT_N)%DVDT_N] - vRing[oldest]) / ws;
  }
}

// ================================================================
//  Setup
// ================================================================
void setup() {
  Serial.begin(115200);
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

  // Boot calibration – stable 64-sample average before any load
  delay(300);
  sBatMv  = (float)adcAvgMv(PIN_BAT_VOLT, 32);
  sCurMv  = (float)adcAvgMv(PIN_CUR_SENS, 64);
  sZeroMv = sCurMv;

  vPack     = (sBatMv * BAT_DIV) / 1000.0f;
  vRested   = vPack;
  vCellRest = vPack / (float)CELLS_S;
  socOcv    = socFromV(vCellRest);
  socBlend  = socOcv;
  // socInit = false  →  Coulomb SOC initialises from OCV on first tick

  lastUiMs = pageMs = millis();
}

// ================================================================
//  Loop – all work gated to UI_MS cadence
// ================================================================
void loop() {
  uint32_t now = millis();

  // Non-blocking page flip
  if (now - pageMs >= PAGE_MS) {
    uiPage = (uiPage + 1) % 2;
    pageMs  = now;
  }

  // Sample + render at fixed cadence (fixes Coulomb integration rate and dV/dt)
  if (now - lastUiMs >= UI_MS) {
    lastUiMs = now;
    updateMeasurements();

    canvas.fillScreen(TFT_BLACK);
    if (uiPage == 0) renderPage0();
    else             renderPage1();
    canvas.pushSprite(&lcd, 0, 0);

    Serial.printf(
      "V=%.2f I=%+.2fA P=%.1fW | "
      "soc=%.0f%%(ocv=%.0f%%,cc=%.0f%%) | "
      "soh=%.0f%% R=%.0fmO sag=%.0fmV | "
      "eta=%dm Ah=%.2f/%.2f Wh=%.1f/%.1f | "
      "dV/dt=%+.1fmV/s peak=%.1fA/%.0fW\n",
      vPack, iA, pW,
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
