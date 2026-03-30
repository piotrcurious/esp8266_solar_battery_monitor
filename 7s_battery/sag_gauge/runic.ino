#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <math.h>

// ============================================================
// User hardware configuration
// ============================================================

// ----- TFT (ST7735 / 160x80) -----
static constexpr int TFT_SCK  = 18;
static constexpr int TFT_MOSI = 23;
static constexpr int TFT_MISO = -1;
static constexpr int TFT_DC   = 16;
static constexpr int TFT_CS   = 5;
static constexpr int TFT_RST  = 17;
static constexpr int TFT_BL   = 4;

// ----- ADC pins (ADC1 preferred on ESP32) -----
static constexpr int PIN_BAT_VOLT = 34;  // battery divider -> ADC
static constexpr int PIN_CUR_SENS = 35;  // ACS712 divider -> ADC

// ----- Battery divider -----
// Choose values that keep 29.4V safely below ADC range.
// 110k / 10k => ratio 12.0, so 29.4V -> 2.45V at ADC.
static constexpr float BAT_R_TOP = 110000.0f;
static constexpr float BAT_R_BOT = 10000.0f;
static constexpr float BAT_DIV_RATIO = (BAT_R_TOP + BAT_R_BOT) / BAT_R_BOT;

// ----- ACS712 divider -----
// 10k / 10k halves the output, keeping a 5V-powered ACS712 safe for ESP32 ADC.
static constexpr float CUR_R_TOP = 10000.0f;
static constexpr float CUR_R_BOT = 10000.0f;
static constexpr float CUR_DIV_RATIO = (CUR_R_TOP + CUR_R_BOT) / CUR_R_BOT;

// ACS712 variant sensitivity.
// 5A  -> 185 mV/A
// 20A -> 100 mV/A
// 30A ->  66 mV/A
static constexpr float ACS712_SENS_MV_PER_A = 185.0f;

// Flip sign if your sensor orientation is reversed.
static constexpr float CURRENT_POLARITY = 1.0f;

// ----- Display update -----
static constexpr uint32_t UI_REFRESH_MS = 100;

// ----- Filter / estimator tuning -----
static constexpr float ADC_ALPHA_FAST = 0.20f;
static constexpr float ADC_ALPHA_SLOW = 0.03f;
static constexpr float REST_CURRENT_A = 0.20f;
static constexpr float SAG_UPDATE_CURRENT_A = 0.70f;
static constexpr float RINT_ALPHA = 0.03f;

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

static float adcBatMvFilt   = 0.0f;  // ADC-side mV after divider
static float adcCurMvFilt   = 0.0f;  // ADC-side mV after divider
static float acsZeroMvFilt   = 0.0f;  // ACS712 zero at ADC-side after divider

static float restedPackV    = 0.0f;   // slow estimate of unloaded pack voltage
static float rIntOhm        = 0.060f; // estimated internal resistance (ohm)
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

static uint32_t socColor(float soc)
{
  // red -> amber -> green
  if (soc < 20.0f) return lcd.color888(255, 40, 40);
  if (soc < 50.0f) return lcd.color888(255, 170, 30);
  if (soc < 80.0f) return lcd.color888(180, 220, 40);
  return lcd.color888(60, 255, 90);
}

static void drawBarSegmented(int x, int y, int w, int h, float soc, uint32_t fillColor, uint32_t outlineColor, uint32_t emptyColor)
{
  const int segments = 7;
  const int gap = 2;
  const int segW = (w - gap * (segments - 1)) / segments;
  const float perSeg = 100.0f / segments;

  for (int i = 0; i < segments; ++i) {
    int sx = x + i * (segW + gap);

    canvas.drawRoundRect(sx, y, segW, h, 2, outlineColor);
    canvas.fillRoundRect(sx + 1, y + 1, segW - 2, h - 2, 2, emptyColor);

    float segFill = clampf((soc - i * perSeg) / perSeg, 0.0f, 1.0f);
    if (segFill > 0.0f) {
      int fh = (int)((h - 2) * segFill + 0.5f);
      int fy = y + 1 + (h - 2 - fh);
      uint32_t c0 = colorBlend(lcd.color888(255, 60, 40), fillColor, segFill);
      canvas.fillRoundRect(sx + 1, fy, segW - 2, fh, 2, c0);
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
  // Read ADC-side voltages after divider
  const uint32_t batMvRaw = readAdcAverageMv(PIN_BAT_VOLT, 16);
  const uint32_t curMvRaw = readAdcAverageMv(PIN_CUR_SENS, 16);

  if (adcBatMvFilt <= 1.0f) adcBatMvFilt = (float)batMvRaw;
  else adcBatMvFilt = lowpass(adcBatMvFilt, (float)batMvRaw, ADC_ALPHA_SLOW);

  if (adcCurMvFilt <= 1.0f) adcCurMvFilt = (float)curMvRaw;
  else adcCurMvFilt = lowpass(adcCurMvFilt, (float)curMvRaw, ADC_ALPHA_SLOW);

  // Calibrate ACS712 zero at boot and gently track drift when idle.
  if (acsZeroMvFilt <= 1.0f) acsZeroMvFilt = adcCurMvFilt;
  if (fabsf(currentA) < REST_CURRENT_A) {
    acsZeroMvFilt = lowpass(acsZeroMvFilt, adcCurMvFilt, 0.01f);
  }

  // Convert ADC-side to real-side values.
  packV = (adcBatMvFilt * BAT_DIV_RATIO) / 1000.0f;

  float sensorMv = adcCurMvFilt * CUR_DIV_RATIO;
  float zeroMv   = acsZeroMvFilt * CUR_DIV_RATIO;
  currentA = CURRENT_POLARITY * ((sensorMv - zeroMv) / ACS712_SENS_MV_PER_A);

  // Track resting voltage estimate.
  if (restedPackV <= 1.0f) restedPackV = packV;
  if (fabsf(currentA) < REST_CURRENT_A) {
    restedPackV = lowpass(restedPackV, packV, 0.12f);
  } else {
    // Under load, keep the estimate mostly stable, but allow a tiny drift.
    restedPackV = lowpass(restedPackV, packV + fabsf(currentA) * rIntOhm, 0.002f);
  }

  packVRested = restedPackV;

  // Sag and internal resistance estimate only make sense in discharge/load.
  if (currentA > SAG_UPDATE_CURRENT_A) {
    sagV = packVRested - packV;
    if (sagV < 0.0f) sagV = 0.0f;

    float rEst = sagV / currentA;
    if (isfinite(rEst) && rEst > 0.0f && rEst < 1.0f) {
      rIntOhm = lowpass(rIntOhm, rEst, RINT_ALPHA);
    }
  } else {
    // still show measured sag from current resting estimate
    sagV = packVRested - packV;
    if (sagV < 0.0f) sagV = 0.0f;
  }

  packCellVLoad   = packV / 7.0f;
  packCellVRested = packVRested / 7.0f;

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

  // Big left value
  canvas.setTextSize(2);
  canvas.setTextColor(socColor(socRestPct), TFT_BLACK);
  canvas.setCursor(4, 18);
  canvas.print(packCellVRested, 2);
  canvas.print("V/c");

  // Small right status
  canvas.setTextSize(1);
  canvas.setTextColor(lcd.color888(180, 180, 190), TFT_BLACK);

  canvas.setCursor(96, 18);
  if (currentA > REST_CURRENT_A) {
    canvas.print("DISCH");
  } else if (currentA < -REST_CURRENT_A) {
    canvas.print("CHG");
  } else {
    canvas.print("IDLE");
  }

  canvas.setCursor(96, 29);
  canvas.print("I ");
  canvas.print(currentA, 1);
  canvas.print("A");

  canvas.setCursor(96, 40);
  canvas.print("SAG ");
  canvas.print(sagV * 1000.0f, 0);
  canvas.print("mV");

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

  packV = (adcBatMvFilt * BAT_DIV_RATIO) / 1000.0f;
  restedPackV = packV;

  renderUi();
  lastUiMs = millis();
}

void loop()
{
  updateMeasurements();

  uint32_t now = millis();
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
