#ifndef LOVYANGFX_HPP
#define LOVYANGFX_HPP

#include <stdint.h>

namespace lgfx {
    class LGFX_Device {
    public:
        void setPanel(void* p) {}
        void init() {}
        void setRotation(int r) {}
        void setBrightness(int b) {}
        void setColorDepth(int d) {}
        uint32_t color888(uint8_t r, uint8_t g, uint8_t b) {
            return (r << 16) | (g << 8) | b;
        }
        void pushSprite(void* sprite, int x, int y) {}
    };

    struct Panel_ST7735 {
        struct Config {
            int pin_cs, pin_rst, pin_busy, panel_width, panel_height, offset_x, offset_y, offset_rotation, dummy_read_pixel, dummy_read_bits;
            bool readable, invert, rgb_order, dlen_16bit, bus_shared;
        };
        Config config() { return Config(); }
        void config(const Config& c) {}
        void setBus(void* b) {}
        void setLight(void* l) {}
    };

    struct Bus_SPI {
        struct Config {
            int spi_host, spi_mode, freq_write, freq_read, pin_sclk, pin_mosi, pin_miso, pin_dc;
        };
        Config config() { return Config(); }
        void config(const Config& c) {}
    };

    struct Light_PWM {
        struct Config {
            int pin_bl, freq, pwm_channel;
            bool invert;
        };
        Config config() { return Config(); }
        void config(const Config& c) {}
    };
}

#define VSPI_HOST 0
#define TFT_BLACK 0

class LGFX_Sprite {
public:
    LGFX_Sprite(void* device) {}
    void setColorDepth(int d) {}
    void createSprite(int w, int h) {}
    void fillScreen(uint32_t c) {}
    void fillRoundRect(int x, int y, int w, int h, int r, uint32_t c) {}
    void drawRoundRect(int x, int y, int w, int h, int r, uint32_t c) {}
    void setTextSize(int s) {}
    void setTextColor(uint32_t c, uint32_t b) {}
    void setCursor(int x, int y) {}
    void print(const char* s) {}
    void print(float f, int p = 2) {}
    void drawCircle(int x, int y, int r, uint32_t c) {}
    void fillCircle(int x, int y, int r, uint32_t c) {}
    void pushSprite(void* device, int x, int y) {}
};

#endif
