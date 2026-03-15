#ifndef TFT_ESPI_H
#define TFT_ESPI_H

#include <stdint.h>

class TFT_eSPI {
public:
    void init() {}
    void setRotation(uint8_t r) {}
    void fillScreen(uint32_t color) {}
    void setTextColor(uint32_t color) {}
    void setTextSize(uint8_t size) {}
    void drawString(const char* s, int x, int y, int f) {}
    void drawRect(int x, int y, int w, int h, uint32_t color) {}
    void drawPixel(int x, int y, uint32_t color) {}
    void drawLine(int x1, int y1, int x2, int y2, uint32_t color) {}
    void setCursor(int x, int y) {}
    // Add other functions as needed
};

#endif
