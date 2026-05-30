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

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7735 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
  lgfx::Light_PWM    _light_instance;

public:
  LGFX(void)
  {
    { // SPI bus
      auto cfg = _bus_instance.config();
      cfg.spi_host   = VSPI_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.pin_sclk    = TFT_SCK;
      cfg.pin_mosi    = TFT_MOSI;
      cfg.pin_miso    = TFT_MISO;
      cfg.pin_dc      = TFT_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // panel
      auto cfg = _panel_instance.config();
      cfg.pin_cs            = TFT_CS;
      cfg.pin_rst           = TFT_RST;
      cfg.pin_busy          = -1;
      cfg.panel_width       = 160;
      cfg.panel_height      = 80;
      cfg.offset_x          = 0;   // adjust if your module is shifted
      cfg.offset_y          = 0;   // adjust if your module is shifted
      cfg.offset_rotation   = 0;   // adjust if rotation is offset on your panel
      cfg.dummy_read_pixel  = 8;
      cfg.dummy_read_bits   = 1;
      cfg.readable          = true;
      cfg.invert            = false;
      cfg.rgb_order         = false; // set true if red/blue are swapped
      cfg.dlen_16bit        = false;
      cfg.bus_shared        = false;

      // Uncomment only if your specific panel needs it.
      // cfg.memory_width  = 160;
      // cfg.memory_height = 80;

      _panel_instance.config(cfg);
    }

    { // backlight
      auto cfg = _light_instance.config();
      cfg.pin_bl       = TFT_BL;
      cfg.invert       = false;
      cfg.freq         = 44100;
      cfg.pwm_channel  = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

static LGFX lcd;
static LGFX_Sprite canvas(&lcd);

// ============================================================
// State
// ============================================================

static float adcBatMvFilt   = 0.0f;
static float adcCurMvFilt   = 0.0f;
static float acsZeroMvFilt   = 0.0f;

static float restedPackV    = 0.0f;
static float rIntOhm        = 0.060f;
static float currentA       = 0.0f;
static float packV          = 0.0f;
static float packVRested    = 0.0f;
static float sagV           = 0.0f;
static float packCellVLoad   = 0.0f;
static float packCellVRested = 0.0f;
static float socLoadPct      = 0.0f;
static float socRestPct      = 0.0f;

static uint32_t lastUiMs = 0;

// ============================================================
// Helpers
// ============================================================

static float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float lowpass(float prev, float input, float alpha)
{
  return prev + alpha * (input - prev);
}

static uint32_t readAdcAverageMv(int pin, int samples)
{
  uint32_t sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogReadMilliVolts(pin);
  }
  return sum / (uint32_t)samples;
}

// Approximate Li-ion SOC from cell OCV.
// This is intentionally smooth, not “perfect chemistry truth”.
// Uses a coarse discharge curve shape.
static float socFromCellVoltage(float v)
{
  struct Pt { float v; float soc; };
  static constexpr Pt lut[] = {
    {4.20f, 100.0f},
    {4.10f,  95.0f},
    {4.00f,  88.0f},
    {3.95f,  84.0f},
    {3.90f,  78.0f},
    {3.85f,  70.0f},
    {3.80f,  62.0f},
    {3.75f,  54.0f},
    {3.70f,  45.0f},
    {3.65f,  36.0f},
    {3.60f,  28.0f},
    {3.50f,  16.0f},
    {3.40f,   8.0f},
    {3.20f,   0.0f},
  };

  if (v >= lut[0].v) return 100.0f;
  if (v <= lut[sizeof(lut) / sizeof(lut[0]) - 1].v) return 0.0f;

  for (size_t i = 0; i + 1 < sizeof(lut) / sizeof(lut[0]); ++i) {
    const Pt &a = lut[i];
    const Pt &b = lut[i + 1];
    if (v <= a.v && v >= b.v) {
      float t = (v - b.v) / (a.v - b.v);
      return b.soc + t * (a.soc - b.soc);
    }
  }
  return 0.0f;
}

static uint32_t colorBlend(uint32_t a, uint32_t b, float t)
{
  t = clampf(t, 0.0f, 1.0f);
  uint8_t ar = (a >> 16) & 0xFF, ag = (a >> 8) & 0xFF, ab = a & 0xFF;
  uint8_t br = (b >> 16) & 0xFF, bg = (b >> 8) & 0xFF, bb = b & 0xFF;
  uint8_t r = (uint8_t)(ar + (br - ar) * t);
  uint8_t g = (uint8_t)(ag + (bg - ag) * t);
  uint8_t bl = (uint8_t)(ab + (bb - ab) * t);
  return lcd.color888(r, g, bl);
}

static uint32_t colorBlend(uint32_t a, uint32_t b, float t)
{
  t = (t < 0.0f) ? 0.0f : (t > 1.0f) ? 1.0f : t;
  uint8_t ar = (a >> 16) & 0xFF, ag = (a >> 8) & 0xFF, ab = a & 0xFF;
  uint8_t br = (b >> 16) & 0xFF, bg = (b >> 8) & 0xFF, bb = b & 0xFF;
  return lcd.color888(ar + (int)((br-ar)*t), ag + (int)((bg-ag)*t), ab + (int)((bb-ab)*t));
}

static uint32_t socColor(float s)
{
  if (s < 15.0f) return colorBlend(lcd.color888(255, 0, 0), lcd.color888(255, 100, 0), s / 15.0f);
  if (s < 40.0f) return colorBlend(lcd.color888(255, 100, 0), lcd.color888(255, 255, 0), (s - 15.0f) / 25.0f);
  if (s < 70.0f) return colorBlend(lcd.color888(255, 255, 0), lcd.color888(0, 255, 0), (s - 40.0f) / 30.0f);
  return colorBlend(lcd.color888(0, 255, 0), lcd.color888(100, 255, 255), (s - 70.0f) / 30.0f);
}

static void drawBarSegmented(int x, int y, int w, int h, float soc, uint32_t fillColor, uint32_t outlineColor, uint32_t emptyColor)
{
  const int segments = 7;
  const int gap = 3;
  const int segW = (w - gap * (segments - 1)) / segments;
  const float perSeg = 100.0f / segments;

  // Background box
  canvas.fillRoundRect(x-2, y-2, w+4, h+4, 4, lcd.color888(20,20,25));

  for (int i = 0; i < segments; ++i) {
    int sx = x + i * (segW + gap);

    canvas.drawRoundRect(sx, y, segW, h, 3, outlineColor);
    canvas.fillRoundRect(sx + 1, y + 1, segW - 2, h - 2, 2, emptyColor);

    float segFill = clampf((soc - i * perSeg) / perSeg, 0.0f, 1.0f);
    if (segFill > 0.0f) {
      int fh = (int)((h - 2) * segFill + 0.5f);
      int fy = y + 1 + (h - 2 - fh);
      uint32_t c0 = colorBlend(lcd.color888(255, 70, 50), fillColor, segFill);
      canvas.fillRoundRect(sx + 1, fy, segW - 2, fh, 2, c0);

      if (segFill >= 1.0f) {
        canvas.fillRect(sx+2, y+2, segW-4, 2, colorBlend(c0, 0xFFFFFF, 0.3f));
      }
    }
  }
}

static void drawLabelValue(int x, int y, const char *label, float value, const char *unit, int decimals, uint32_t color)
{
  canvas.setTextColor(color, TFT_BLACK);
  canvas.setCursor(x, y);
  canvas.print(label);
  canvas.print(value, decimals);
  canvas.print(unit);
}

// ============================================================
// Sampling + estimation
// ============================================================

static void updateMeasurements()
{
  const uint32_t batMvRaw = readAdcAverageMv(PIN_BAT_VOLT, 16);
  const uint32_t curMvRaw = readAdcAverageMv(PIN_CUR_SENS, 16);

  adcBatMvFilt = (adcBatMvFilt <= 1.0f) ? (float)batMvRaw : lowpass(adcBatMvFilt, (float)batMvRaw, CFG.alpha_adc);
  adcCurMvFilt = (adcCurMvFilt <= 1.0f) ? (float)curMvRaw : lowpass(adcCurMvFilt, (float)curMvRaw, CFG.alpha_adc);

  if (acsZeroMvFilt <= 1.0f) acsZeroMvFilt = adcCurMvFilt;
  if (fabsf(currentA) < CFG.idle_a) acsZeroMvFilt = lowpass(acsZeroMvFilt, adcCurMvFilt, ALPHA_ZERO);

  packV = (adcBatMvFilt * CFG.bat_div) / 1000.0f + BAT_V_OFFSET;
  currentA = CURRENT_POLARITY * ((adcCurMvFilt * CFG.cur_div - acsZeroMvFilt * CFG.cur_div) / CFG.acs_mv_a);

  if (restedPackV <= 1.0f) restedPackV = packV;
  if (fabsf(currentA) < CFG.idle_a) restedPackV = lowpass(restedPackV, packV, CFG.alpha_rest_v);
  else restedPackV = lowpass(restedPackV, packV + ((currentA > 0) ? 1.0f : -1.0f) * fabsf(currentA) * rIntOhm, CFG.alpha_load_v);

  packVRested = restedPackV;
  sagV = fabsf(packVRested - packV);
  if (fabsf(currentA) > CFG.sag_min_a) {
    float rEst = sagV / fabsf(currentA);
    if (isfinite(rEst) && rEst > 0.0f && rEst < 1.0f) rIntOhm = lowpass(rIntOhm, rEst, CFG.alpha_rint);
  }

  packCellVLoad = packV / (float)CFG.cells_s;
  packCellVRested = packVRested / (float)CFG.cells_s;
  socLoadPct = socFromCellVoltage(packCellVLoad);
  socRestPct = socFromCellVoltage(packCellVRested);
}

// ============================================================
// UI
// ============================================================

static void renderUi()
{
  canvas.fillScreen(TFT_BLACK);

  // Header band
  uint32_t headerColor = socColor(socRestPct);
  canvas.fillRoundRect(0, 0, 160, 14, 3, lcd.color888(18, 18, 24));
  canvas.setTextSize(1);
  canvas.setTextColor(headerColor, lcd.color888(18, 18, 24));
  canvas.setCursor(4, 4);
  canvas.print("7S Li-ion  ");
  canvas.print(packV, 1);
  canvas.print("V");

  // Status text in header
  canvas.setCursor(120, 4);
  if (currentA > REST_CURRENT_A) canvas.print("DISCH");
  else if (currentA < -REST_CURRENT_A) canvas.print("CHG");
  else canvas.print("IDLE");

  // Big left value
  canvas.setTextSize(2);
  canvas.setTextColor(socColor(socRestPct), TFT_BLACK);
  canvas.setCursor(4, 18);
  // Flash if low battery
  if (socRestPct < 15.0f && (millis() / 500) % 2 == 0) {
      canvas.setTextColor(lcd.color888(255, 40, 40), TFT_BLACK);
      canvas.print("LOW BAT!");
  } else {
      canvas.print(packCellVRested, 2);
      canvas.print("V/c");
  }

  // Small right status
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(180, 180, 190), TFT_BLACK);

  canvas.setCursor(96, 18);
  canvas.print("SOC ");
  canvas.print(socRestPct, 0);
  canvas.print("%");

  canvas.setCursor(96, 29);
  canvas.print("I ");
  canvas.print(currentA, 1);
  canvas.print("A");

  canvas.setCursor(96, 40);
  canvas.print("S ");
  if (sagV < 1.0f) { canvas.print(sagV * 1000.0f, 0); canvas.print("mV"); }
  else { canvas.print(sagV, 1); canvas.print("V"); }

  // Main gauge: bright estimate with loaded shadow beneath
  const int gx = 4;
  const int gy = 48;
  const int gw = 152;
  const int gh = 22;

  // Shadow gauge = loaded value
  drawBarSegmented(gx, gy, gw, gh, socLoadPct,
                   lcd.color888(70, 120, 255),
                   lcd.color888(40, 40, 60),
                   lcd.color888(12, 12, 16));

  // Bright overlay = sag-corrected estimate
  drawBarSegmented(gx, gy, gw, gh, socRestPct,
                   socColor(socRestPct),
                   lcd.color888(180, 180, 200),
                   lcd.color888(0, 0, 0));

  // Bottom stats
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(220, 220, 220), TFT_BLACK);

  canvas.setCursor(4, 72);
  canvas.print("SOC ");
  canvas.print(socRestPct, 0);
  canvas.print("%");

  canvas.setCursor(54, 72);
  canvas.print("R ");
  canvas.print(rIntOhm * 1000.0f, 0);
  canvas.print("m");

  canvas.setCursor(96, 72);
  canvas.print("L ");
  canvas.print(socLoadPct, 0);
  canvas.print("%");

  // Tiny hint bar when sag is high
  if (sagV > 0.8f) {
    canvas.fillRect(145, 70, 11, 6, lcd.color888(255, 60, 60));
  } else if (sagV > 0.3f) {
    canvas.fillRect(145, 70, 11, 6, lcd.color888(255, 160, 40));
  } else {
    canvas.fillRect(145, 70, 11, 6, lcd.color888(60, 255, 90));
  }

  canvas.pushSprite(&lcd, 0, 0);
}

// ============================================================
// Arduino
// ============================================================

void setup()
{
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

  // Initial calibration
  delay(300);
  uint32_t curZero = readAdcAverageMv(PIN_CUR_SENS, 64);
  uint32_t bat0    = readAdcAverageMv(PIN_BAT_VOLT, 32);

  acsZeroMvFilt = (float)curZero;
  adcBatMvFilt  = (float)bat0;
  adcCurMvFilt  = (float)curZero;

  packV = (adcBatMvFilt * CFG.bat_div) / 1000.0f;
  restedPackV = packV;

  renderUi();
  lastUiMs = millis();
}

void loop()
{
  uint32_t now = millis();

  // Button handling
  static bool lastBtn = true;
  static uint32_t btnDownMs = 0;
  bool btn = digitalRead(PIN_BUTTON);
  if (!btn && lastBtn) btnDownMs = now;
  if (btn && !lastBtn) {
    uint32_t dur = now - btnDownMs;
    if (dur > 5000) { prefs.begin("sag_gauge", false); prefs.clear(); prefs.end(); ESP.restart(); }
    else if (dur > 50) { /* Page flip or other? Runic is single page for now */ }
  }
  lastBtn = btn;

  updateMeasurements();

  if (now - lastUiMs >= UI_REFRESH_MS) {
    lastUiMs = now;

    renderUi();

    // optional serial debug
    Serial.print("V=");
    Serial.print(packV, 2);
    Serial.print("  I=");
    Serial.print(currentA, 2);
    Serial.print("A  sag=");
    Serial.print(sagV, 3);
    Serial.print("V  Rint=");
    Serial.print(rIntOhm * 1000.0f, 1);
    Serial.println("mOhm");
  }
}
