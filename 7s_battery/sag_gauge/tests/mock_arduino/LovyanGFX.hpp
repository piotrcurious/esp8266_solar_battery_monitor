#pragma once
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include <cmath>

#ifdef UI_AUDIT
extern "C" void audit_draw_call(const char* type, int x, int y, int w, int h);
#else
#define audit_draw_call(type, x, y, w, h) ((void)0)
#endif

namespace lgfx {
    class LGFX_Device {
        int _brightness = 0;
    public:
        void setPanel(void* p) {}
        void init() {}
        void setRotation(int r) {}
        void setBrightness(int b) {
            _brightness = b;
            printf("LGFX:setBrightness:%d\n", b);
        }
        int getBrightness() { return _brightness; }
        void setColorDepth(int d) {}
        uint32_t color888(uint8_t r, uint8_t g, uint8_t b) {
            return (r << 16) | (g << 8) | b;
        }
        void pushSprite(void* sprite, int x, int y) {
            printf("LGFX:pushSprite:x=%d,y=%d\n", x, y);
        }
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
    int _text_size = 1;

public:
    LGFX_Sprite(void* device) {}
    void setColorDepth(int d) {}
    void createSprite(int w, int h) {
        _w = w; _h = h;
        _buffer.assign(w * h, 0);
        printf("SPRITE:createSprite:w=%d,h=%d\n", w, h);
    }
    void fillScreen(uint32_t c) {
        if (_buffer.empty()) return;
        std::fill(_buffer.begin(), _buffer.end(), c);
        printf("SPRITE:fillScreen:color=0x%06X\n", c);
    }
    void fillRect(int x, int y, int w, int h, uint32_t c) {
        audit_draw_call("fillRect", x, y, w, h);
        if (_buffer.empty()) return;
        for (int j=y; j<y+h && j<_h; j++)
            for (int i=x; i<x+w && i<_w; i++)
                if (i>=0 && j>=0) _buffer[j*_w + i] = c;
        printf("SPRITE:fillRect:x=%d,y=%d,w=%d,h=%d,color=0x%06X\n", x, y, w, h, c);
    }
    void drawRect(int x, int y, int w, int h, uint32_t c) {
        audit_draw_call("drawRect", x, y, w, h);
        if (_buffer.empty()) return;
        printf("SPRITE:drawRect:x=%d,y=%d,w=%d,h=%d,color=0x%06X\n", x, y, w, h, c);
    }
    void drawPixel(int x, int y, uint32_t c) {
        audit_draw_call("drawPixel", x, y, 1, 1);
        if (_buffer.empty()) return;
        if (x>=0 && x<_w && y>=0 && y<_h) _buffer[y*_w + x] = c;
    }
    void fillRoundRect(int x, int y, int w, int h, int r, uint32_t c) {
        fillRect(x, y, w, h, c);
    }
    void drawRoundRect(int x, int y, int w, int h, int r, uint32_t c) {
        drawRect(x, y, w, h, c);
    }
    void drawLine(int x0, int y0, int x1, int y1, uint32_t c) {
        int w = abs(x1-x0)+1, h = abs(y1-y0)+1;
        audit_draw_call("drawLine", std::min(x0,x1), std::min(y0,y1), w, h);
        if (_buffer.empty()) return;
        printf("SPRITE:drawLine:x0=%d,y0=%d,x1=%d,y1=%d,color=0x%06X\n", x0, y0, x1, y1, c);
    }
    void setTextSize(int s) { _text_size = s; }
    void setTextColor(uint32_t c, uint32_t b = 0) { _cur_color = c; }
    void setCursor(int x, int y) { _cur_x = x; _cur_y = y; }
    void print(const char* s) {
        int len = strlen(s);
        int cw = 6 * _text_size;
        int ch = 8 * _text_size;
        audit_draw_call("print", _cur_x, _cur_y, len*cw, ch);
        if (_buffer.empty()) return;
        printf("SPRITE:print:x=%d,y=%d,size=%d,color=0x%06X,text=\"%s\"\n", _cur_x, _cur_y, _text_size, _cur_color, s);
        _cur_x += len * cw;
    }
    void print(float f, int p = 2) {
        char buf[32]; snprintf(buf, 32, "%.*f", p, f); print(buf);
    }
    void drawCircle(int x, int y, int r, uint32_t c) {
        audit_draw_call("drawCircle", x-r, y-r, 2*r, 2*r);
        if (_buffer.empty()) return;
    }
    void fillCircle(int x, int y, int r, uint32_t c) {
        audit_draw_call("fillCircle", x-r, y-r, 2*r, 2*r);
        if (_buffer.empty()) return;
    }
    void pushSprite(void* device, int x, int y) {
        printf("SPRITE:pushSprite:x=%d,y=%d\n", x, y);
    }

    void savePPM(const char* filename) {
        if (_buffer.empty()) return;
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
