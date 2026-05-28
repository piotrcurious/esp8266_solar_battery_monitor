/*
 * SSD1306 multicast receiver
 * Receives telemetry frames from multicast UDP and displays them on SSD1306.
 *
 * Refactor goals:
 * - no interrupts
 * - cooperative scheduling with millis()
 * - packet processing separated from rendering
 * - easier to extend with encoder/buttons/mode switching
 */

// -----------------------------------------------------------------------------
// WiFi / telemetry configuration
// -----------------------------------------------------------------------------
#include "wifi_settings.h"       // SSID, password, AP config, multicast config
#include "telemetry_frame.h"     // must match sender
#include "display_settings.h"     // display config / widgets

// -----------------------------------------------------------------------------
// Application state
// -----------------------------------------------------------------------------
uint8_t display_brightness = 255;

static uint32_t last_packet_millis = 0;
static uint32_t last_display_refresh_millis = 0;
static uint32_t wifi_bars_millis = 0;

static bool display_dirty = true;

#ifdef SCREENSAVER_TRICKS
static uint8_t screensaver_x_offset = 0;
static uint8_t screensaver_y_offset = 0;
#endif

enum class ScreenMode : uint8_t
{
  VoltageText = 1,
  VoltageGauge = 2
};

static ScreenMode screen = ScreenMode::VoltageGauge;
telemetry_frame tframe;

// -----------------------------------------------------------------------------
// Gauge configuration
// -----------------------------------------------------------------------------
static constexpr size_t BUFFER_SIZE = 128;
static constexpr float  ABS_MIN = 6.0f;
static constexpr float  ABS_MAX = 17.0f;

float  buffer[BUFFER_SIZE] = {0.0f};
size_t  gauge_index = 0;
size_t  gauge_count = 0;
float   gauge_min = ABS_MAX;
float   gauge_max = ABS_MIN;

// -----------------------------------------------------------------------------
// Timing configuration
// -----------------------------------------------------------------------------
static constexpr uint32_t DISPLAY_REFRESH_PERIOD_MS = 20;
static constexpr uint32_t WIFI_BARS_PERIOD_MS       = 1000;
static constexpr uint32_t BRIGHTNESS_DECAY_MS       = 10;

// keep your existing timeout_period from display_settings.h or elsewhere

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static void markDisplayDirty()
{
  display_dirty = true;
}

static int mapFloatToX(float value, float inMin, float inMax, int outMin, int outMax)
{
  if (inMax <= inMin) return outMin;
  const float clamped = constrain(value, inMin, inMax);
  return (int)(outMin + (clamped - inMin) * (outMax - outMin) / (inMax - inMin));
}

static void updateMinMax()
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

static void display_wifi_bars()
{
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
}

static void drawTimeoutTicker()
{
#ifdef TIMEOUT_TICKER
  if (UDP_packets_lost > 0)
  {
    const uint8_t pos = (UDP_packets_lost >= TIMEOUT_TICKER_WIDTH)
                      ? TIMEOUT_TICKER_WIDTH - 1
                      : UDP_packets_lost;

    display.drawPixel(TIMEOUT_TICKER_X + pos, TIMEOUT_TICKER_Y, SSD1306_WHITE);
  }
#endif
}

static void renderVoltageText()
{
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

#ifdef SCREENSAVER_TRICKS
  if (screensaver_x_offset < 8)  screensaver_x_offset += random(2);
  if (screensaver_x_offset > 0)   screensaver_x_offset -= random(2);

  if (screensaver_y_offset < 40)  screensaver_y_offset += random(2);
  if (screensaver_y_offset > 0)   screensaver_y_offset -= random(2);

  display.setCursor(screensaver_x_offset, screensaver_y_offset);
#else
  display.setCursor(0, 0);
#endif

  display.print(tframe.voltage_ADC0, 2);
  display.print(F("V"));
}

static void renderGauge(float value)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.cp437(true);

  display.setCursor(0, 0);
  display.print(gauge_min, 2);

  display.setCursor(SCREEN_WIDTH - 8 * 5, 0);
  display.print(gauge_max, 2);

  int x = mapFloatToX(value, gauge_min, gauge_max, 4, SCREEN_WIDTH - 4);
  x = constrain(x, 1, SCREEN_WIDTH - 2);

  int min_x = 1;
  if (value < gauge_min)
  {
    min_x = mapFloatToX(value, ABS_MIN, gauge_min, 3, SCREEN_WIDTH - 4 - 8 * 5);
    min_x = constrain(min_x, 1, SCREEN_WIDTH - 2);
  }

  display.drawLine(x, 10, min_x, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(x - 2, 12, x, 10, SSD1306_WHITE);
  display.drawLine(x + 2, 12, x, 10, SSD1306_WHITE);

  display.setCursor(SCREEN_WIDTH - 40, 56);
  display.print(tframe.voltage_ADC0, 2);
  display.print(F("V"));
}

static void renderCurrentScreen()
{
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

  display_wifi_bars();
  drawTimeoutTicker();
}

static void pushGaugeSample(float value)
{
  buffer[gauge_index] = value;
  gauge_index = (gauge_index + 1) % BUFFER_SIZE;
  if (gauge_count < BUFFER_SIZE) gauge_count++;

  updateMinMax();
}

static void processTelemetryPacket()
{
  const int packetSize = Udp.parsePacket();
  if (packetSize <= 0)
    return;

  UDP_packets++;
  display_brightness = 255;
  last_packet_millis = millis();

#ifdef TIMEOUT_TICKER
  if (UDP_packets_lost > 1)
  {
    UDP_packets_lost--;
    if (UDP_packets_lost >= TIMEOUT_TICKER_WIDTH) UDP_packets_lost = TIMEOUT_TICKER_WIDTH;
  }
#endif

  const int message_size = Udp.read(reinterpret_cast<uint8_t*>(&tframe), sizeof(tframe));
  if (message_size <= 0)
    return;

  if (screen == ScreenMode::VoltageGauge)
  {
    pushGaugeSample(tframe.voltage_ADC0);
  }

  markDisplayDirty();
}

static void updateBrightnessAndEffects(uint32_t now)
{
  static uint32_t last_decay_millis = 0;

  if ((now - last_decay_millis) < BRIGHTNESS_DECAY_MS)
    return;

  last_decay_millis = now;

  if (display_brightness > 0)
    display_brightness--;

#ifdef SCREENSAVER_TRICKS
  if (display_brightness <= 200)
  {
    for (uint8_t i = 0; i <= 32; ++i)
    {
      display.drawPixel(random(SCREEN_WIDTH), random(SCREEN_HEIGHT), SSD1306_BLACK);
    }
    markDisplayDirty();
  }
#endif

  display.setContrast(display_brightness);
}

static void updateWifiBarsIfNeeded(uint32_t now)
{
#ifdef WIFI_BARS
#ifndef AP_mode_on
  if ((now - wifi_bars_millis) >= WIFI_BARS_PERIOD_MS)
  {
    wifi_bars_millis = now;
    display_wifi_bars();
    markDisplayDirty();
  }
#endif
#endif
}

static void updateTimeoutState(uint32_t now)
{
  if ((now - last_packet_millis) > timeout_period)
  {
    UDP_packets_lost_total++;
    last_packet_millis = now;

#ifdef TIMEOUT_TICKER
    if (UDP_packets_lost < TIMEOUT_TICKER_WIDTH)
      UDP_packets_lost++;

    if (UDP_packets_lost >= TIMEOUT_TICKER_WIDTH)
      UDP_packets_lost = TIMEOUT_TICKER_WIDTH;
#endif

    markDisplayDirty();
  }
}

static void refreshDisplayIfNeeded(uint32_t now)
{
  if (!display_dirty && (now - last_display_refresh_millis) < DISPLAY_REFRESH_PERIOD_MS)
    return;

  last_display_refresh_millis = now;
  renderCurrentScreen();
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

  last_packet_millis = millis();
  wifi_bars_millis = millis();
  last_display_refresh_millis = millis();

  markDisplayDirty();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop()
{
  const uint32_t now = millis();

  processTelemetryPacket();
  updateTimeoutState(now);
  updateWifiBarsIfNeeded(now);
  updateBrightnessAndEffects(now);
  refreshDisplayIfNeeded(now);
}
