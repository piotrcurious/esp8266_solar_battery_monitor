#pragma once
#include <LovyanGFX.hpp>

// ============================================================
//  Hardware Pins
// ============================================================
static constexpr int TFT_SCK  = 18, TFT_MOSI = 23, TFT_MISO = -1;
static constexpr int TFT_DC   = 16, TFT_CS   = 5,  TFT_RST  = 17, TFT_BL = 4;
static constexpr int PIN_BAT_VOLT = 34, PIN_CUR_SENS = 35, PIN_BUTTON = 0;

// ============================================================
//  LovyanGFX Custom Device
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

// ============================================================
//  Global State Enums
// ============================================================
enum class PackState : uint8_t { IDLE, CHARGING, DISCHARGING };
