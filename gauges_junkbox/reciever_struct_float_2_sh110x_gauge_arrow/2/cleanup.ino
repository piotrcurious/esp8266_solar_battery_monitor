/*
 * SSD1306 multicast receiver
 * Receives telemetry frames from multicast UDP and displays them on SSD1306.
 *
 * Goals:
 * - get WiFi SSID/password via IR
 * - keep frames synchronized in time
 * - use interrupts for frame timing / display refresh
 * - support display modes and user feedback
 */

// -----------------------------------------------------------------------------
// WiFi / telemetry configuration
// -----------------------------------------------------------------------------
#include "wifi_settings.h"       // SSID, password, AP config, multicast config
#include "telemetry_frame.h"     // must match sender
#include "display_settings.h"     // display config / widgets

// -----------------------------------------------------------------------------
// Timer interrupt includes
// -----------------------------------------------------------------------------
#define USING_TIM_DIV1                false
#define USING_TIM_DIV16               false
#define USING_TIM_DIV256              true

#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.h"

// -----------------------------------------------------------------------------
// Timer interrupt setup
// -----------------------------------------------------------------------------
#define HW_TIMER_INTERVAL_MS          10L
#define NUMBER_ISR_TIMERS             2

ESP8266Timer ITimer;
ESP8266_ISR_Timer ISR_Timer;

void IRAM_ATTR TimerHandler()
{
  ISR_Timer.run();
}

using irqCallback = void (*)();

volatile bool display_refresh_sync = false;

void IRAM_ATTR display_interface_interrupt()
{
  display_refresh_sync = false;
}

void IRAM_ATTR UDP_interface_interrupt()
{
  // reserved for future packet timing / sync work
}

const uint32_t TimerInterval[NUMBER_ISR_TIMERS] =
{
  20L,  // OLED refresh
  10L   // UDP/network layer timing hook
};

irqCallback irqCallbackFunc[NUMBER_ISR_TIMERS] =
{
  display_interface_interrupt,
  UDP_interface_interrupt
};

// -----------------------------------------------------------------------------
// Application state
// -----------------------------------------------------------------------------
static uint32_t last_packet_millis = 0;
static uint32_t wifi_bars_millis   = 0;

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
static constexpr size_t  BUFFER_SIZE = 128;
static constexpr float   ABS_MIN = 6.0f;
static constexpr float   ABS_MAX = 17.0f;

float  buffer[BUFFER_SIZE] = {0.0f};
size_t  gauge_index = 0;
size_t  gauge_count = 0;
float   gauge_min = ABS_MAX;
float   gauge_max = ABS_MIN;

static void updateMinMax()
{
  if (gauge_count == 0)
  {
    gauge_min = ABS_MAX;
    gauge_max = ABS_MIN;
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

static int mapFloatToX(float value, float inMin, float inMax, int outMin, int outMax)
{
  if (inMax <= inMin) return outMin;
  const float clamped = constrain(value, inMin, inMax);
  return (int)(outMin + (clamped - inMin) * (outMax - outMin) / (inMax - inMin));
}

static void displayValue(float value)
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

static void draw_gauge(float value)
{
  buffer[gauge_index] = value;
  gauge_index = (gauge_index + 1) % BUFFER_SIZE;
  if (gauge_count < BUFFER_SIZE) gauge_count++;

  updateMinMax();
  displayValue(value);
}

// -----------------------------------------------------------------------------
// WiFi signal indicator
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Packet handling
// -----------------------------------------------------------------------------
static uint32_t UDP_packets = 0;
static uint32_t UDP_packets_lost = 0;
static uint32_t UDP_packets_lost_total = 0;

static void handleTelemetryPacket()
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

    for (uint8_t i = 0; i <= UDP_packets_lost; ++i)
    {
      display.drawPixel(TIMEOUT_TICKER_X + i, TIMEOUT_TICKER_Y, SSD1306_BLACK);
    }
  }

  UDP_packets_lost = (UDP_packets_lost + UDP_packets_lost) / 2;
  if (UDP_packets_lost > 0)
  {
    display.drawPixel(TIMEOUT_TICKER_X + UDP_packets_lost, TIMEOUT_TICKER_Y, SSD1306_WHITE);
  }
#endif

  // Read only up to the size of the struct to avoid overflow / malformed packets.
  const int message_size = Udp.read(reinterpret_cast<uint8_t*>(&tframe), sizeof(tframe));
  if (message_size <= 0)
    return;

  switch (screen)
  {
    case ScreenMode::VoltageText:
    {
#ifdef SCREENSAVER_TRICKS
      display.clearDisplay();

      if (screensaver_x_offset < 8)  screensaver_x_offset += random(2);
      if (screensaver_x_offset > 0)   screensaver_x_offset -= random(2);

      if (screensaver_y_offset < 40)  screensaver_y_offset += random(2);
      if (screensaver_y_offset > 0)   screensaver_y_offset -= random(2);

      display.setCursor(screensaver_x_offset, screensaver_y_offset);
#else
      display.setCursor(0, 0);
#endif
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      display.print(tframe.voltage_ADC0, 2);
      display.print(F("V"));
      break;
    }

    case ScreenMode::VoltageGauge:
    default:
      draw_gauge(tframe.voltage_ADC0);
      break;
  }
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

  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);

  for (uint16_t i = 0; i < NUMBER_ISR_TIMERS; ++i)
  {
    ISR_Timer.setInterval(TimerInterval[i], irqCallbackFunc[i]);
  }

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
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop()
{
  const uint32_t now = millis();

  if (!display_refresh_sync)
  {
    if (display_brightness > 0)
      display_brightness--;

    if (display_brightness <= 200)
    {
#ifdef SCREENSAVER_TRICKS
      for (uint8_t i = 0; i <= 32; ++i)
      {
        display.drawPixel(random(128), random(64), SSD1306_BLACK);
      }
#endif
    }

    display.setContrast(display_brightness);
    display.display();
    display_refresh_sync = true;
  }

#ifdef WIFI_BARS
  if ((now - wifi_bars_millis) > WIFI_BARS_PERIOD)
  {
#ifndef AP_mode_on
    display_wifi_bars();
#endif
    wifi_bars_millis = now;
  }
#endif

  handleTelemetryPacket();

  if ((now - last_packet_millis) > timeout_period)
  {
    UDP_packets_lost_total++;
    last_packet_millis = now;

#ifdef TIMEOUT_TICKER
    if (UDP_packets_lost < TIMEOUT_TICKER_WIDTH)
      UDP_packets_lost++;

    if (UDP_packets_lost >= TIMEOUT_TICKER_WIDTH)
      UDP_packets_lost = TIMEOUT_TICKER_WIDTH;

    if (UDP_packets_lost > 0 && UDP_packets_lost < TIMEOUT_TICKER_WIDTH)
    {
      display.drawPixel(TIMEOUT_TICKER_X + UDP_packets_lost, TIMEOUT_TICKER_Y, SSD1306_WHITE);
    }
#endif
  }
}
