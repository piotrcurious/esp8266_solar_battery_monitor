#ifndef LOVYANGFX_HPP
#define LOVYANGFX_HPP

#include <stdint.h>
#include <stdio.h>
#include <vector>

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
    int _w=0, _h=0;
    std::vector<uint32_t> _buffer;
    uint32_t _cur_color = 0xFFFFFF;
    int _cur_x=0, _cur_y=0;

public:
    LGFX_Sprite(void* device) {}
    void setColorDepth(int d) {}
    void createSprite(int w, int h) {
        _w = w; _h = h;
        _buffer.assign(w * h, 0);
    }
    void fillScreen(uint32_t c) {
        std::fill(_buffer.begin(), _buffer.end(), c);
    }
    void fillRect(int x, int y, int w, int h, uint32_t c) {
        for (int j=y; j<y+h && j<_h; j++)
            for (int i=x; i<x+w && i<_w; i++)
                if (i>=0 && j>=0) _buffer[j*_w + i] = c;
    }
    void fillRoundRect(int x, int y, int w, int h, int r, uint32_t c) {
        fillRect(x, y, w, h, c); // Simplification
    }
    void drawRoundRect(int x, int y, int w, int h, int r, uint32_t c) {
        // Just draw a frame
        for (int i=x; i<x+w && i<_w; i++) { if (i>=0 && y>=0) _buffer[y*_w + i] = c; if (i>=0 && y+h-1<_h) _buffer[(y+h-1)*_w + i] = c; }
        for (int j=y; j<y+h && j<_h; j++) { if (x>=0 && j>=0) _buffer[j*_w + x] = c; if (x+w-1<_w && j>=0) _buffer[j*_w + x+w-1] = c; }
    }
    void setTextSize(int s) {}
    void setTextColor(uint32_t c, uint32_t b) { _cur_color = c; }
    void setCursor(int x, int y) { _cur_x = x; _cur_y = y; }
    void print(const char* s) {
        // Mock: just draw a small block for each character
        int len = strlen(s);
        fillRect(_cur_x, _cur_y, len * 6, 8, _cur_color);
        _cur_x += len * 6;
    }
    void print(float f, int p = 2) {
        char buf[32]; snprintf(buf, 32, "%.*f", p, f); print(buf);
    }
    void drawCircle(int x, int y, int r, uint32_t c) {
        fillRect(x-r, y-r, r*2, r*2, c); // Simplification
    }
    void fillCircle(int x, int y, int r, uint32_t c) {
        fillRect(x-r, y-r, r*2, r*2, c); // Simplification
    }
    void pushSprite(void* device, int x, int y) {}

    void savePPM(const char* filename) {
        FILE* f = fopen(filename, "wb");
        if (!f) return;
        fprintf(f, "P6\n%d %d\n255\n", _w, _h);
        for (uint32_t c : _buffer) {
            uint8_t r = (c >> 16) & 0xFF;
            uint8_t g = (c >> 8) & 0xFF;
            uint8_t b = c & 0xFF;
            fwrite(&r, 1, 1, f);
            fwrite(&g, 1, 1, f);
            fwrite(&b, 1, 1, f);
        }
        fclose(f);
    }
};

#endif
