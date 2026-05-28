/*
 * SSD1306 multicast receiver
 * Cooperative version: no interrupts, no timer ISR framework.
 *
 * Features:
 * - WiFi SSID/password from config
 * - Multicast UDP telemetry reception
 * - Fuzzy graphics / motion to reduce burn-in
 * - Idle-triggered screensaver with multiple animation modes
 * - Simple display refresh scheduling via millis()
 * - Hooks for future encoder/buttons/input handling
 */

// -----------------------------------------------------------------------------
// Core ESP8266 / network includes
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// -----------------------------------------------------------------------------
// Project headers
// -----------------------------------------------------------------------------
#include "wifi_settings.h"       // ssid, password, AP config, multicast IP/port
#include "telemetry_frame.h"     // telemetry_frame struct must match sender
#include "display_settings.h"    // display object + screen constants + timeout_period

// -----------------------------------------------------------------------------
// UDP object
// -----------------------------------------------------------------------------
WiFiUDP Udp;

// -----------------------------------------------------------------------------
// Application state
// -----------------------------------------------------------------------------
telemetry_frame tframe {};

enum class ScreenMode : uint8_t
{
  VoltageText  = 1,
  VoltageGauge  = 2
};

static ScreenMode screen = ScreenMode::VoltageGauge;

static uint8_t display_brightness = 255;

static uint32_t last_packet_millis         = 0;
static uint32_t last_display_refresh_millis = 0;
static uint32_t last_wifi_bars_millis      = 0;
static uint32_t last_activity_millis       = 0;
static uint32_t last_decay_millis          = 0;
static uint32_t last_motion_millis         = 0;
static uint32_t last_saver_step_millis     = 0;

static bool display_dirty = true;
static bool screensaver_active = false;

static uint32_t UDP_packets = 0;
static uint32_t UDP_packets_lost = 0;
static uint32_t UDP_packets_lost_total = 0;

// -----------------------------------------------------------------------------
// Gauge history
// -----------------------------------------------------------------------------
static constexpr size_t BUFFER_SIZE = 128;
static constexpr float  ABS_MIN = 6.0f;
static constexpr float  ABS_MAX = 17.0f;

static float  buffer[BUFFER_SIZE] = {0.0f};
static size_t  gauge_index = 0;
static size_t  gauge_count = 0;
static float   gauge_min = ABS_MIN;
static float   gauge_max = ABS_MAX;

// -----------------------------------------------------------------------------
// Tuning knobs
// -----------------------------------------------------------------------------
static constexpr uint32_t DISPLAY_REFRESH_PERIOD_MS = 25;    // ~40 FPS
static constexpr uint32_t BRIGHTNESS_DECAY_MS       = 12;
static constexpr uint32_t WIFI_BARS_PERIOD_MS       = 1000;
static constexpr uint32_t SCREEN_SAVER_AFTER_MS     = 30000;
static constexpr uint32_t SCREEN_SAVER_STEP_MS      = 70;
static constexpr uint32_t CONTENT_MOTION_MS         = 250;

static constexpr uint8_t  SCREEN_SAVER_MODES        = 4;
static constexpr uint8_t  NORMAL_FUZZ_PIXELS        = 3;
static constexpr uint8_t  SAVER_FUZZ_PIXELS         = 18;

static constexpr int16_t  NORMAL_X_DRIFT_MAX       = 2;
static constexpr int16_t  NORMAL_Y_DRIFT_MAX       = 2;

// -----------------------------------------------------------------------------
// Motion state for active screens
// -----------------------------------------------------------------------------
static int16_t content_x = 0;
static int16_t content_y = 0;
static int16_t content_dx = 1;
static int16_t content_dy = 1;

// -----------------------------------------------------------------------------
// Screensaver state
// -----------------------------------------------------------------------------
static uint8_t saver_mode = 0;
static uint8_t saver_phase = 0;
static int16_t saver_x = 0;
static int16_t saver_y = 0;
static int16_t saver_dx = 1;
static int16_t saver_dy = 1;

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------
static void markDirty()
{
  display_dirty = true;
}

static void markActivity()
{
  last_activity_millis = millis();
  if (screensaver_active)
  {
    screensaver_active = false;
    display.invertDisplay(false);
    markDirty();
  }
}

static int mapFloatToX(float value, float inMin, float inMax, int outMin, int outMax)
{
  if (inMax <= inMin) return outMin;
  const float clamped = constrain(value, inMin, inMax);
  return (int)(outMin + (clamped - inMin) * (outMax - outMin) / (inMax - inMin));
}

static void applyFuzz(uint8_t amount)
{
  for (uint8_t i = 0; i < amount; ++i)
  {
    const int x = random(SCREEN_WIDTH);
    const int y = random(SCREEN_HEIGHT);

    if (random(5) == 0)
      display.drawPixel(x, y, SSD1306_BLACK);
    else
      display.drawPixel(x, y, SSD1306_WHITE);
  }
}

static void updateGaugeMinMax()
{
  if (gauge_count == 0)
  {
    gauge_min = ABS_MIN;
    gauge_max = ABS_MAX;
    return;
  }

  float minv = ABS_MAX;
  float maxv = ABS_MIN;

  const size_t count = (gauge_count < BUFFER_SIZE) ? gauge_count : BUFFER_SIZE;
  for (size_t i = 0; i < count; ++i)
  {
    const float v = buffer[i];
    if (v <= 0.0f) continue;

    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  if (minv == ABS_MAX) minv = ABS_MIN;
  if (maxv == ABS_MIN) maxv = ABS_MAX;

  gauge_min = constrain(minv, ABS_MIN, ABS_MAX);
  gauge_max = constrain(maxv, ABS_MIN, ABS_MAX);

  if (gauge_min > gauge_max)
  {
    const float mid = tframe.voltage_ADC0;
    gauge_min = constrain(mid - 0.5f, ABS_MIN, ABS_MAX);
    gauge_max = constrain(mid + 0.5f, ABS_MIN, ABS_MAX);
  }
}

static void pushGaugeSample(float value)
{
  buffer[gauge_index] = value;
  gauge_index = (gauge_index + 1) % BUFFER_SIZE;
  if (gauge_count < BUFFER_SIZE) gauge_count++;
  updateGaugeMinMax();
}

static void updateActiveMotion(uint32_t now)
{
  if ((now - last_motion_millis) < CONTENT_MOTION_MS)
    return;

  last_motion_millis = now;

  content_x += content_dx;
  content_y += content_dy;

  if (content_x < -NORMAL_X_DRIFT_MAX)
  {
    content_x = -NORMAL_X_DRIFT_MAX;
    content_dx = 1;
  }
  else if (content_x > NORMAL_X_DRIFT_MAX)
  {
    content_x = NORMAL_X_DRIFT_MAX;
    content_dx = -1;
  }

  if (content_y < -NORMAL_Y_DRIFT_MAX)
  {
    content_y = -NORMAL_Y_DRIFT_MAX;
    content_dy = 1;
  }
  else if (content_y > NORMAL_Y_DRIFT_MAX)
  {
    content_y = NORMAL_Y_DRIFT_MAX;
    content_dy = -1;
  }

  if (random(10) == 0) content_dx = -content_dx;
  if (random(10) == 0) content_dy = -content_dy;
}

static void enterScreensaver()
{
  screensaver_active = true;
  saver_mode = random(SCREEN_SAVER_MODES);
  saver_phase = 0;

  saver_x = random(0, SCREEN_WIDTH - 24);
  saver_y = random(0, SCREEN_HEIGHT - 12);
  saver_dx = random(2) ? 1 : -1;
  saver_dy = random(2) ? 1 : -1;

  last_saver_step_millis = millis();
  markDirty();
}

static void updateScreensaverState(uint32_t now)
{
  if (!screensaver_active)
  {
    if ((now - last_activity_millis) >= SCREEN_SAVER_AFTER_MS)
      enterScreensaver();
  }
}

static void stepScreensaver()
{
  saver_x += saver_dx;
  saver_y += saver_dy;

  if (saver_x < 0)
  {
    saver_x = 0;
    saver_dx = 1;
  }
  else if (saver_x > (SCREEN_WIDTH - 24))
  {
    saver_x = SCREEN_WIDTH - 24;
    saver_dx = -1;
  }

  if (saver_y < 0)
  {
    saver_y = 0;
    saver_dy = 1;
  }
  else if (saver_y > (SCREEN_HEIGHT - 12))
  {
    saver_y = SCREEN_HEIGHT - 12;
    saver_dy = -1;
  }

  if (random(18) == 0) saver_dx = -saver_dx;
  if (random(18) == 0) saver_dy = -saver_dy;

  saver_phase++;
}

static void drawTimeoutTicker()
{
#ifdef TIMEOUT_TICKER
  if (UDP_packets_lost > 0)
  {
    const uint8_t pos = (UDP_packets_lost >= TIMEOUT_TICKER_WIDTH)
                      ? (TIMEOUT_TICKER_WIDTH - 1)
                      : UDP_packets_lost;

    display.drawPixel(TIMEOUT_TICKER_X + pos, TIMEOUT_TICKER_Y, SSD1306_WHITE);
  }
#endif
}

static void displayWifiBars()
{
#ifdef WIFI_BARS
#ifndef AP_mode_on
  const int rssi = WiFi.RSSI();
  uint8_t bars = 0;

  if (rssi > -55)         bars = 4;
  else if (rssi > -65)    bars = 4;
  else if (rssi > -75)    bars = 3;
  else if (rssi > -85)    bars = 2;
  else if (rssi > -95)    bars = 1;
  else                    bars = 0;

  display.fillRect(WIFI_BARS_X, WIFI_BARS_Y - 8, 10, 8, SSD1306_BLACK);

  for (uint8_t b = 0; b < bars; ++b)
  {
    display.fillRect(WIFI_BARS_X + (b * 2), WIFI_BARS_Y - (b * 2), 1, b * 2, SSD1306_WHITE);
  }

  if (bars == 0)
  {
    display.drawPixel(WIFI_BARS_X, WIFI_BARS_Y - 1, SSD1306_WHITE);
  }
#endif
#endif
}

// -----------------------------------------------------------------------------
// Renderers
// -----------------------------------------------------------------------------
static void renderVoltageText()
{
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

#ifdef SCREENSAVER_TRICKS
  const int16_t tx = constrain<int16_t>(content_x + 1, 0, SCREEN_WIDTH - 32);
  const int16_t ty = constrain<int16_t>(content_y + 1, 0, SCREEN_HEIGHT - 16);
  display.setCursor(tx, ty);
#else
  display.setCursor(0, 0);
#endif

  display.print(tframe.voltage_ADC0, 2);
  display.print(F("V"));
  applyFuzz(NORMAL_FUZZ_PIXELS);
}

static void renderGauge(float value)
{
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.cp437(true);

  const int ox = content_x;
  const int oy = content_y;

  display.setCursor(constrain(0 + ox, 0, SCREEN_WIDTH - 1), constrain(0 + oy, 0, SCREEN_HEIGHT - 1));
  display.print(gauge_min, 2);

  display.setCursor(constrain(SCREEN_WIDTH - 8 * 5 + ox, 0, SCREEN_WIDTH - 1), constrain(0 + oy, 0, SCREEN_HEIGHT - 1));
  display.print(gauge_max, 2);

  int x = mapFloatToX(value, gauge_min, gauge_max, 4, SCREEN_WIDTH - 4);
  x = constrain(x + ox, 1, SCREEN_WIDTH - 2);

  int min_x = 1;
  if (value < gauge_min)
  {
    min_x = mapFloatToX(value, ABS_MIN, gauge_min, 3, SCREEN_WIDTH - 4 - 8 * 5);
    min_x = constrain(min_x + ox, 1, SCREEN_WIDTH - 2);
  }

  const int y0 = constrain(10 + oy, 0, SCREEN_HEIGHT - 1);
  const int y1 = constrain(SCREEN_HEIGHT - 1 + oy, 0, SCREEN_HEIGHT - 1);

  display.drawLine(x, y0, min_x, y1, SSD1306_WHITE);
  display.drawLine(x - 2, y0 + 2, x, y0, SSD1306_WHITE);
  display.drawLine(x + 2, y0 + 2, x, y0, SSD1306_WHITE);

  display.setCursor(constrain(SCREEN_WIDTH - 40 + ox, 0, SCREEN_WIDTH - 1), constrain(56 + oy, 0, SCREEN_HEIGHT - 1));
  display.print(tframe.voltage_ADC0, 2);
  display.print(F("V"));

  applyFuzz(NORMAL_FUZZ_PIXELS);
}

static void renderNormalScreen()
{
  display.clearDisplay();

  switch (screen)
  {
    case ScreenMode::VoltageText:
      renderVoltageText();
      break;

    case ScreenMode::VoltageGauge:
    default:
      renderGauge(tframe.voltage_ADC0);
      break;
  }

  displayWifiBars();
  drawTimeoutTicker();
}

static void renderScreensaver()
{
  display.clearDisplay();

  switch (saver_mode)
  {
    case 0:
    {
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      display.setCursor(saver_x, saver_y);
      display.print(F("IDLE"));
      display.setCursor(saver_x, saver_y + 10);
      display.print(tframe.voltage_ADC0, 2);
      display.print(F("V"));
      applyFuzz(SAVER_FUZZ_PIXELS);
      break;
    }

    case 1:
    {
      for (uint8_t i = 0; i < 12; ++i)
      {
        const int x = random(SCREEN_WIDTH);
        const int y = random(SCREEN_HEIGHT);
        display.drawPixel(x, y, SSD1306_WHITE);
      }

      display.drawRect(saver_x, saver_y, 24, 10, SSD1306_WHITE);
      display.drawRect(SCREEN_WIDTH - saver_x - 24, SCREEN_HEIGHT - saver_y - 10, 24, 10, SSD1306_WHITE);
      applyFuzz(SAVER_FUZZ_PIXELS);
      break;
    }

    case 2:
    {
      for (int x = 0; x < SCREEN_WIDTH; x += 8)
      {
        const int y = (x + saver_phase) % SCREEN_HEIGHT;
        display.drawLine(x, y, x + 7, (y + 12) % SCREEN_HEIGHT, SSD1306_WHITE);
      }
      applyFuzz(SAVER_FUZZ_PIXELS);
      break;
    }

    case 3:
    default:
    {
      const int cx = SCREEN_WIDTH / 2;
      const int cy = SCREEN_HEIGHT / 2;
      const int r1 = 6 + (saver_phase % 10);
      const int r2 = 12 + (saver_phase % 14);

      display.drawCircle(cx, cy, r1, SSD1306_WHITE);
      display.drawCircle(cx, cy, r2, SSD1306_WHITE);
      display.drawLine(cx, cy, saver_x + 8, saver_y + 4, SSD1306_WHITE);
      applyFuzz(SAVER_FUZZ_PIXELS);
      break;
    }
  }

  if ((saver_phase % 64) == 0)
    display.invertDisplay((saver_phase / 64) & 1);
}

static void renderCurrentScreen(uint32_t now)
{
  if (screensaver_active)
  {
    if ((now - last_saver_step_millis) >= SCREEN_SAVER_STEP_MS)
    {
      last_saver_step_millis = now;
      stepScreensaver();
    }

    renderScreensaver();
    return;
  }

  renderNormalScreen();
}

// -----------------------------------------------------------------------------
// Packet handling
// -----------------------------------------------------------------------------
static void processTelemetryPacket()
{
  const int packetSize = Udp.parsePacket();
  if (packetSize <= 0)
    return;

  UDP_packets++;
  display_brightness = 255;
  markActivity();
  last_packet_millis = millis();

  if (packetSize != (int)sizeof(telemetry_frame))
  {
    // Ignore malformed or version-mismatched packets.
    // Drain the packet payload so the socket can continue cleanly.
    uint8_t sink[64];
    while (Udp.available() > 0)
      Udp.read(sink, sizeof(sink));
    return;
  }

  telemetry_frame incoming {};
  const int bytesRead = Udp.read(reinterpret_cast<uint8_t*>(&incoming), sizeof(incoming));
  if (bytesRead != (int)sizeof(incoming))
    return;

  tframe = incoming;

  if (screen == ScreenMode::VoltageGauge)
  {
    pushGaugeSample(tframe.voltage_ADC0);
  }

  // A new packet is a strong signal that the display should be refreshed now.
  markDirty();
}

static void updateTimeoutState(uint32_t now)
{
  if ((now - last_packet_millis) <= timeout_period)
    return;

  UDP_packets_lost_total++;
  last_packet_millis = now;

#ifdef TIMEOUT_TICKER
  if (UDP_packets_lost < TIMEOUT_TICKER_WIDTH)
    UDP_packets_lost++;

  if (UDP_packets_lost > TIMEOUT_TICKER_WIDTH)
    UDP_packets_lost = TIMEOUT_TICKER_WIDTH;
#endif

  markDirty();
}

static void updateBrightness(uint32_t now)
{
  if ((now - last_decay_millis) < BRIGHTNESS_DECAY_MS)
    return;

  last_decay_millis = now;

  if (display_brightness > 0)
    display_brightness--;

  display.setContrast(display_brightness);

  // Keep some motion alive even while fading.
  if (display_brightness <= 200)
    markDirty();
}

static void updateWifiBars(uint32_t now)
{
#ifdef WIFI_BARS
  if ((now - last_wifi_bars_millis) < WIFI_BARS_PERIOD_MS)
    return;

  last_wifi_bars_millis = now;

#ifndef AP_mode_on
  displayWifiBars();
  markDirty();
#endif
#endif
}

static void updateIdleState(uint32_t now)
{
  updateScreensaverState(now);
  if (screensaver_active)
    markDirty();
}

static void refreshDisplayIfNeeded(uint32_t now)
{
  if (!display_dirty && (now - last_display_refresh_millis) < DISPLAY_REFRESH_PERIOD_MS)
    return;

  last_display_refresh_millis = now;
  renderCurrentScreen(now);
  display.display();
  display_dirty = false;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(50);

  randomSeed(ESP.getCycleCount());

  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(150);

  display.begin(SCREEN_ADDRESS, true);
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.cp437(true);

  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  delay(200);

  WiFi.persistent(false);

#ifdef AP_mode_on
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#else
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("WiFi: "));
    display.print(millis());
    display.display();
    delay(100);
  }
  display.clearDisplay();
  display.display();
#endif

  Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort);

  const uint32_t now = millis();
  last_packet_millis = now;
  last_display_refresh_millis = now;
  last_wifi_bars_millis = now;
  last_activity_millis = now;
  last_decay_millis = now;
  last_motion_millis = now;
  last_saver_step_millis = now;

  display_brightness = 255;
  display.setContrast(display_brightness);
  markDirty();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop()
{
  const uint32_t now = millis();

  // Future encoder/button handling can hook in here:
  // - change screen mode
  // - adjust brightness
  // - wake display
  // - toggle saver styles
  //
  // pollInputs();

  processTelemetryPacket();
  updateTimeoutState(now);
  updateWifiBars(now);
  updateBrightness(now);
  updateActiveMotion(now);
  updateIdleState(now);
  refreshDisplayIfNeeded(now);
}
