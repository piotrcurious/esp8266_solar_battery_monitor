/*
 * ESP32 BMS Monitor with Nuklear GUI
 * Demonstrates Nuklear immediate-mode GUI with custom ESP32 backend
 * 
 * Libraries needed:
 * - Nuklear (single header: nuklear.h)
 * - TFT_eSPI or similar display driver
 * - Wire.h (for I2C communication)
 * 
 * Note: Place nuklear.h in your project directory
 */

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_IMPLEMENTATION
#define NK_BUTTON_TRIGGER_ON_RELEASE

#include "nuklear.h"
#include <TFT_eSPI.h>
#include <Wire.h>

// Display configuration
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define MAX_VERTEX_MEMORY 512 * 1024
#define MAX_ELEMENT_MEMORY 128 * 1024

// TFT Display
TFT_eSPI tft = TFT_eSPI();

// BMS Data Structure
struct BMSData {
  float cellVoltages[8];
  float packVoltage;
  float current;
  float temperature[4];
  float soc;
  float soh;
  uint32_t cycleCount;
  bool balancing[8];
  bool charging;
  bool error;
  float maxCellV;
  float minCellV;
  int maxCellIdx;
  int minCellIdx;
};

BMSData bms;

// Nuklear context and buffers
struct nk_context ctx;
struct nk_user_font font;
struct nk_buffer cmds;
struct nk_draw_null_texture null_texture;

// GUI state
static int activeTab = 0;
static float voltageHistory[50];
static int historyIndex = 0;
static unsigned long lastUpdate = 0;
static bool showSettings = false;
static float targetVoltage = 4.1f;
static float targetCurrent = 2.0f;

// Color conversion helper
uint16_t nk_color_to_rgb565(struct nk_color col) {
  uint16_t r = (col.r >> 3) & 0x1F;
  uint16_t g = (col.g >> 2) & 0x3F;
  uint16_t b = (col.b >> 3) & 0x1F;
  return (r << 11) | (g << 5) | b;
}

// Nuklear rendering backend for TFT_eSPI
void nk_tft_render(struct nk_context *ctx) {
  const struct nk_command *cmd;
  
  nk_foreach(cmd, ctx) {
    switch (cmd->type) {
      case NK_COMMAND_NOP: break;
      
      case NK_COMMAND_SCISSOR: {
        const struct nk_command_scissor *s = (const struct nk_command_scissor*)cmd;
        tft.setViewport(s->x, s->y, s->w, s->h);
      } break;
      
      case NK_COMMAND_LINE: {
        const struct nk_command_line *l = (const struct nk_command_line*)cmd;
        uint16_t color = nk_color_to_rgb565(l->color);
        tft.drawLine(l->begin.x, l->begin.y, l->end.x, l->end.y, color);
      } break;
      
      case NK_COMMAND_RECT: {
        const struct nk_command_rect *r = (const struct nk_command_rect*)cmd;
        uint16_t color = nk_color_to_rgb565(r->color);
        tft.drawRect(r->x, r->y, r->w, r->h, color);
      } break;
      
      case NK_COMMAND_RECT_FILLED: {
        const struct nk_command_rect_filled *r = (const struct nk_command_rect_filled*)cmd;
        uint16_t color = nk_color_to_rgb565(r->color);
        tft.fillRect(r->x, r->y, r->w, r->h, color);
      } break;
      
      case NK_COMMAND_CIRCLE: {
        const struct nk_command_circle *c = (const struct nk_command_circle*)cmd;
        uint16_t color = nk_color_to_rgb565(c->color);
        int radius = c->w / 2;
        tft.drawCircle(c->x + radius, c->y + radius, radius, color);
      } break;
      
      case NK_COMMAND_CIRCLE_FILLED: {
        const struct nk_command_circle_filled *c = (const struct nk_command_circle_filled*)cmd;
        uint16_t color = nk_color_to_rgb565(c->color);
        int radius = c->w / 2;
        tft.fillCircle(c->x + radius, c->y + radius, radius, color);
      } break;
      
      case NK_COMMAND_TRIANGLE: {
        const struct nk_command_triangle *t = (const struct nk_command_triangle*)cmd;
        uint16_t color = nk_color_to_rgb565(t->color);
        tft.drawTriangle(t->a.x, t->a.y, t->b.x, t->b.y, t->c.x, t->c.y, color);
      } break;
      
      case NK_COMMAND_TRIANGLE_FILLED: {
        const struct nk_command_triangle_filled *t = (const struct nk_command_triangle_filled*)cmd;
        uint16_t color = nk_color_to_rgb565(t->color);
        tft.fillTriangle(t->a.x, t->a.y, t->b.x, t->b.y, t->c.x, t->c.y, color);
      } break;
      
      case NK_COMMAND_TEXT: {
        const struct nk_command_text *t = (const struct nk_command_text*)cmd;
        uint16_t fg = nk_color_to_rgb565(t->foreground);
        uint16_t bg = nk_color_to_rgb565(t->background);
        
        tft.setTextColor(fg, bg);
        tft.setTextSize(1);
        tft.setCursor(t->x, t->y);
        
        char text[256];
        memcpy(text, t->string, t->length);
        text[t->length] = '\0';
        tft.print(text);
      } break;
      
      default: break;
    }
  }
  
  tft.resetViewport();
}

// Font width callback
float nk_tft_font_width(nk_handle handle, float height, const char *text, int len) {
  return len * 6; // Approximate character width
}

// Initialize BMS data
void initBMSData() {
  for (int i = 0; i < 8; i++) {
    bms.cellVoltages[i] = 3.65 + (random(-20, 20) / 100.0);
    bms.balancing[i] = false;
  }
  for (int i = 0; i < 4; i++) {
    bms.temperature[i] = 25.0 + (random(-5, 15) / 10.0);
  }
  bms.packVoltage = 29.2;
  bms.current = 0.5;
  bms.soc = 75.0;
  bms.soh = 98.5;
  bms.cycleCount = 127;
  bms.charging = true;
  bms.error = false;
  
  updateBMSStats();
  
  // Initialize history
  for (int i = 0; i < 50; i++) {
    voltageHistory[i] = 29.2;
  }
}

// Update BMS statistics
void updateBMSStats() {
  bms.maxCellV = bms.cellVoltages[0];
  bms.minCellV = bms.cellVoltages[0];
  bms.maxCellIdx = 0;
  bms.minCellIdx = 0;
  bms.packVoltage = 0;
  
  for (int i = 0; i < 8; i++) {
    bms.packVoltage += bms.cellVoltages[i];
    if (bms.cellVoltages[i] > bms.maxCellV) {
      bms.maxCellV = bms.cellVoltages[i];
      bms.maxCellIdx = i;
    }
    if (bms.cellVoltages[i] < bms.minCellV) {
      bms.minCellV = bms.cellVoltages[i];
      bms.minCellIdx = i;
    }
  }
  
  // Check balancing needs
  float avgVoltage = bms.packVoltage / 8;
  for (int i = 0; i < 8; i++) {
    bms.balancing[i] = (bms.cellVoltages[i] - avgVoltage) > 0.05;
  }
}

// Simulate BMS updates
void updateBMSData() {
  for (int i = 0; i < 8; i++) {
    bms.cellVoltages[i] += (random(-5, 5) / 1000.0);
    bms.cellVoltages[i] = constrain(bms.cellVoltages[i], 3.0, 4.2);
  }
  
  bms.current += (random(-10, 10) / 100.0);
  bms.current = constrain(bms.current, -5.0, 5.0);
  bms.charging = bms.current > 0;
  
  bms.soc += (bms.current * 0.001);
  bms.soc = constrain(bms.soc, 0, 100);
  
  for (int i = 0; i < 4; i++) {
    bms.temperature[i] += (random(-2, 2) / 10.0);
    bms.temperature[i] = constrain(bms.temperature[i], 20.0, 45.0);
  }
  
  updateBMSStats();
  
  // Update history
  voltageHistory[historyIndex] = bms.packVoltage;
  historyIndex = (historyIndex + 1) % 50;
}

// Custom progress bar widget
void nk_progress_bar_ex(struct nk_context *ctx, float value, float max, 
                        struct nk_color fg, struct nk_color bg, const char *label) {
  struct nk_rect bounds;
  struct nk_command_buffer *out = nk_window_get_canvas(ctx);
  
  nk_layout_row_dynamic(ctx, 20, 1);
  bounds = nk_widget_bounds(ctx);
  
  if (nk_widget(&bounds, ctx)) {
    float percent = value / max;
    
    // Background
    nk_fill_rect(out, bounds, 2, bg);
    
    // Foreground
    struct nk_rect fill = bounds;
    fill.w = bounds.w * percent;
    nk_fill_rect(out, fill, 2, fg);
    
    // Border
    nk_stroke_rect(out, bounds, 2, 1, nk_rgb(100, 100, 100));
    
    // Label
    if (label) {
      char text[64];
      snprintf(text, sizeof(text), "%s: %.1f%%", label, percent * 100.0f);
      nk_draw_text(out, bounds, text, strlen(text), &font, 
                   nk_rgba(0,0,0,0), nk_rgb(255, 255, 255));
    }
  }
}

// Draw overview tab
void drawOverviewTab(struct nk_context *ctx) {
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Pack Voltage:", NK_TEXT_LEFT);
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f V", bms.packVoltage);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, nk_rgb(100, 255, 100));
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Current:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.2f A", bms.current);
  struct nk_color cur_col = bms.charging ? nk_rgb(100, 255, 100) : nk_rgb(255, 200, 100);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, cur_col);
  
  // State of Charge with custom progress bar
  nk_layout_row_dynamic(ctx, 5, 1);
  nk_label(ctx, "", NK_TEXT_LEFT); // Spacer
  
  struct nk_color soc_fg = bms.soc > 30 ? nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
  struct nk_color soc_bg = nk_rgb(50, 50, 50);
  nk_progress_bar_ex(ctx, bms.soc, 100.0f, soc_fg, soc_bg, "SOC");
  
  // State of Health
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "State of Health:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.1f%%", bms.soh);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, nk_rgb(100, 200, 255));
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Cycle Count:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%u", bms.cycleCount);
  nk_label(ctx, buf, NK_TEXT_RIGHT);
  
  // Average temperature
  float avgTemp = 0;
  for (int i = 0; i < 4; i++) avgTemp += bms.temperature[i];
  avgTemp /= 4;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Avg Temperature:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.1f C", avgTemp);
  struct nk_color temp_col = avgTemp < 40 ? nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, temp_col);
  
  // Status
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Status:", NK_TEXT_LEFT);
  const char *status = bms.charging ? "CHARGING" : "DISCHARGING";
  nk_label_colored(ctx, status, NK_TEXT_RIGHT, nk_rgb(100, 255, 100));
  
  // Cell voltage delta
  nk_layout_row_dynamic(ctx, 5, 1);
  nk_label(ctx, "", NK_TEXT_LEFT); // Spacer
  
  nk_layout_row_dynamic(ctx, 20, 1);
  snprintf(buf, sizeof(buf), "Cell Delta: %.3f V (Min: Cell %d, Max: Cell %d)", 
           bms.maxCellV - bms.minCellV, bms.minCellIdx + 1, bms.maxCellIdx + 1);
  struct nk_color delta_col = (bms.maxCellV - bms.minCellV) < 0.1 ? 
                               nk_rgb(100, 255, 100) : nk_rgb(255, 200, 100);
  nk_label_colored(ctx, buf, NK_TEXT_CENTERED, delta_col);
}

// Draw cells tab
void drawCellsTab(struct nk_context *ctx) {
  nk_layout_row_begin(ctx, NK_STATIC, 20, 4);
  nk_layout_row_push(ctx, 40);
  nk_label(ctx, "Cell", NK_TEXT_CENTERED);
  nk_layout_row_push(ctx, 70);
  nk_label(ctx, "Voltage", NK_TEXT_CENTERED);
  nk_layout_row_push(ctx, 130);
  nk_label(ctx, "Bar", NK_TEXT_CENTERED);
  nk_layout_row_push(ctx, 50);
  nk_label(ctx, "Bal", NK_TEXT_CENTERED);
  nk_layout_row_end(ctx);
  
  for (int i = 0; i < 8; i++) {
    nk_layout_row_begin(ctx, NK_STATIC, 25, 4);
    
    // Cell number
    nk_layout_row_push(ctx, 40);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", i + 1);
    nk_label(ctx, buf, NK_TEXT_CENTERED);
    
    // Voltage
    nk_layout_row_push(ctx, 70);
    snprintf(buf, sizeof(buf), "%.3f V", bms.cellVoltages[i]);
    nk_label(ctx, buf, NK_TEXT_RIGHT);
    
    // Progress bar for voltage (3.0 - 4.2V range)
    nk_layout_row_push(ctx, 130);
    float normalized = (bms.cellVoltages[i] - 3.0f) / 1.2f;
    struct nk_rect bounds = nk_widget_bounds(ctx);
    if (nk_widget(&bounds, ctx)) {
      struct nk_command_buffer *out = nk_window_get_canvas(ctx);
      nk_fill_rect(out, bounds, 2, nk_rgb(40, 40, 40));
      struct nk_rect fill = bounds;
      fill.w = bounds.w * normalized;
      nk_fill_rect(out, fill, 2, nk_rgb(100, 255, 100));
      nk_stroke_rect(out, bounds, 2, 1, nk_rgb(150, 150, 150));
    }
    
    // Balancing indicator
    nk_layout_row_push(ctx, 50);
    if (bms.balancing[i]) {
      nk_label_colored(ctx, "BAL", NK_TEXT_CENTERED, nk_rgb(255, 200, 100));
    } else {
      nk_label(ctx, "-", NK_TEXT_CENTERED);
    }
    
    nk_layout_row_end(ctx);
  }
}

// Draw temperature tab
void drawTempTab(struct nk_context *ctx) {
  for (int i = 0; i < 4; i++) {
    nk_layout_row_dynamic(ctx, 20, 2);
    
    char buf[32];
    snprintf(buf, sizeof(buf), "Sensor %d:", i + 1);
    nk_label(ctx, buf, NK_TEXT_LEFT);
    
    snprintf(buf, sizeof(buf), "%.1f C", bms.temperature[i]);
    struct nk_color temp_col = bms.temperature[i] < 40 ? 
                               nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
    nk_label_colored(ctx, buf, NK_TEXT_RIGHT, temp_col);
    
    // Temperature bar (20-45°C range)
    nk_layout_row_dynamic(ctx, 15, 1);
    float normalized = (bms.temperature[i] - 20.0f) / 25.0f;
    struct nk_rect bounds = nk_widget_bounds(ctx);
    if (nk_widget(&bounds, ctx)) {
      struct nk_command_buffer *out = nk_window_get_canvas(ctx);
      nk_fill_rect(out, bounds, 2, nk_rgb(40, 40, 40));
      struct nk_rect fill = bounds;
      fill.w = bounds.w * normalized;
      nk_fill_rect(out, fill, 2, temp_col);
      nk_stroke_rect(out, bounds, 2, 1, nk_rgb(150, 150, 150));
    }
    
    nk_layout_row_dynamic(ctx, 5, 1);
    nk_label(ctx, "", NK_TEXT_LEFT); // Spacer
  }
  
  // Check for high temperature warning
  bool highTemp = false;
  for (int i = 0; i < 4; i++) {
    if (bms.temperature[i] > 40) highTemp = true;
  }
  
  if (highTemp) {
    nk_layout_row_dynamic(ctx, 30, 1);
    struct nk_rect bounds = nk_widget_bounds(ctx);
    if (nk_widget(&bounds, ctx)) {
      struct nk_command_buffer *out = nk_window_get_canvas(ctx);
      nk_fill_rect(out, bounds, 4, nk_rgb(255, 100, 100));
      nk_draw_text(out, bounds, "HIGH TEMPERATURE WARNING", 26, &font,
                   nk_rgb(255, 100, 100), nk_rgb(0, 0, 0));
    }
  }
}

// Draw history tab with chart
void drawHistoryTab(struct nk_context *ctx) {
  nk_layout_row_dynamic(ctx, 20, 1);
  nk_label(ctx, "Voltage History (Last 50 samples)", NK_TEXT_CENTERED);
  
  nk_layout_row_dynamic(ctx, 100, 1);
  struct nk_rect bounds = nk_widget_bounds(ctx);
  
  if (nk_widget(&bounds, ctx)) {
    struct nk_command_buffer *out = nk_window_get_canvas(ctx);
    
    // Draw chart background
    nk_fill_rect(out, bounds, 2, nk_rgb(30, 30, 40));
    nk_stroke_rect(out, bounds, 2, 1, nk_rgb(100, 100, 100));
    
    // Find min/max for scaling
    float minV = voltageHistory[0], maxV = voltageHistory[0];
    for (int i = 1; i < 50; i++) {
      if (voltageHistory[i] < minV) minV = voltageHistory[i];
      if (voltageHistory[i] > maxV) maxV = voltageHistory[i];
    }
    float range = maxV - minV;
    if (range < 0.1) range = 0.1;
    
    // Draw voltage line
    for (int i = 0; i < 49; i++) {
      int idx1 = (historyIndex + i) % 50;
      int idx2 = (historyIndex + i + 1) % 50;
      
      float x1 = bounds.x + (bounds.w * i / 49.0f);
      float y1 = bounds.y + bounds.h - ((voltageHistory[idx1] - minV) / range) * bounds.h;
      float x2 = bounds.x + (bounds.w * (i + 1) / 49.0f);
      float y2 = bounds.y + bounds.h - ((voltageHistory[idx2] - minV) / range) * bounds.h;
      
      nk_stroke_line(out, x1, y1, x2, y2, 2, nk_rgb(100, 255, 100));
    }
    
    // Draw grid lines
    for (int i = 1; i < 4; i++) {
      float y = bounds.y + (bounds.h * i / 4.0f);
      nk_stroke_line(out, bounds.x, y, bounds.x + bounds.w, y, 
                    1, nk_rgb(60, 60, 70));
    }
  }
  
  // Statistics
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Total Cycles:", NK_TEXT_LEFT);
  char buf[32];
  snprintf(buf, sizeof(buf), "%u", bms.cycleCount);
  nk_label(ctx, buf, NK_TEXT_RIGHT);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Peak Discharge:", NK_TEXT_LEFT);
  nk_label(ctx, "8.2 A", NK_TEXT_RIGHT);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Peak Charge:", NK_TEXT_LEFT);
  nk_label(ctx, "5.5 A", NK_TEXT_RIGHT);
}

// Draw settings tab
void drawSettingsTab(struct nk_context *ctx) {
  nk_layout_row_dynamic(ctx, 20, 1);
  nk_label(ctx, "Charge Settings", NK_TEXT_CENTERED);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Target Voltage:", NK_TEXT_LEFT);
  nk_layout_row_dynamic(ctx, 25, 1);
  nk_property_float(ctx, "V:", 3.0f, &targetVoltage, 4.2f, 0.01f, 0.01f);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Target Current:", NK_TEXT_LEFT);
  nk_layout_row_dynamic(ctx, 25, 1);
  nk_property_float(ctx, "A:", 0.1f, &targetCurrent, 5.0f, 0.1f, 0.1f);
  
  nk_layout_row_dynamic(ctx, 10, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  nk_layout_row_dynamic(ctx, 30, 1);
  if (nk_button_label(ctx, "Apply Settings")) {
    // Apply settings logic here
  }
  
  nk_layout_row_dynamic(ctx, 10, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  nk_layout_row_dynamic(ctx, 20, 1);
  nk_label(ctx, "System Info", NK_TEXT_CENTERED);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "Firmware:", NK_TEXT_LEFT);
  nk_label(ctx, "v1.0.3", NK_TEXT_RIGHT);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  nk_label(ctx, "BMS Type:", NK_TEXT_LEFT);
  nk_label(ctx, "8S Li-Ion", NK_TEXT_RIGHT);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize TFT
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);
  
  // Initialize Nuklear
  nk_init_default(&ctx, &font);
  
  // Setup font
  font.userdata.ptr = NULL;
  font.height = 12;
  font.width = nk_tft_font_width;
  
  // Initialize BMS data
  initBMSData();
  
  Serial.println("Nuklear BMS Monitor initialized");
}

void loop() {
  // Update BMS data periodically
  if (millis() - lastUpdate > 500) {
    updateBMSData();
    lastUpdate = millis();
  }
  
  // Start Nuklear frame
  nk_input_begin(&ctx);
  // TODO: Handle touch input here
  nk_input_end(&ctx);
  
  // Main window
  if (nk_begin(&ctx, "BMS Monitor", nk_rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT),
               NK_WINDOW_BORDER | NK_WINDOW_TITLE)) {
    
    // Tab bar
    nk_layout_row_static(&ctx, 25, 60, 5);
    if (nk_button_label(&ctx, "Overview")) activeTab = 0;
    if (nk_button_label(&ctx, "Cells")) activeTab = 1;
    if (nk_button_label(&ctx, "Temp")) activeTab = 2;
    if (nk_button_label(&ctx, "History")) activeTab = 3;
    if (nk_button_label(&ctx, "Settings")) activeTab = 4;
    
    // Status indicator
    nk_layout_row_begin(&ctx, NK_STATIC, 10, 2);
    nk_layout_row_push(&ctx, DISPLAY_WIDTH - 60);
    nk_label(&ctx, "", NK_TEXT_LEFT);
    nk_layout_row_push(&ctx, 50);
    struct nk_rect indicator = nk_widget_bounds(&ctx);
    if (nk_widget(&indicator, &ctx)) {
      struct nk_command_buffer *out = nk_window_get_canvas(&ctx);
      struct nk_color status_col = bms.error ? nk_rgb(255, 100, 100) : nk_rgb(100, 255, 100);
      nk_fill_circle(out, nk_rect(indicator.x + 15, indicator.y - 5, 20, 20), status_col);
    }
    nk_layout_row_end(&ctx);
    
    // Separator
    nk_layout_row_dynamic(&ctx, 5, 1);
    nk_label(&ctx, "", NK_TEXT_LEFT);
    
    // Draw active tab content
    switch(activeTab) {
      case 0: drawOverviewTab(&ctx); break;
      case 1: drawCellsTab(&ctx); break;
      case 2: drawTempTab(&ctx); break;
      case 3: drawHistoryTab(&ctx); break;
      case 4: drawSettingsTab(&ctx); break;
    }
  }
  nk_end(&ctx);
  
  // Render
  tft.fillScreen(TFT_BLACK);
  nk_tft_render(&ctx);
  nk_clear(&ctx);
  
  delay(50);
}

/*
 * ADVANCED NUKLEAR FEATURES DEMONSTRATED:
 * 
 * 1. IMMEDIATE MODE GUI
 *    - No retained state, UI rebuilds each frame
 *    - Simple and flexible design pattern
 * 
 * 2. CUSTOM RENDERING BACKEND
 *    - Implemented nk_tft_render() for TFT_eSPI
 *    - Handles all Nuklear draw commands
 *    - Converts Nuklear colors to RGB565
 * 
 * 3. WIDGETS USED
 *    - Labels with color support
 *    - Buttons with callbacks
 *    - Property editors (sliders)
 *    - Custom progress bars
 *    - Custom charts
 * 
 * 4. LAYOUT SYSTEM
 *    - Dynamic row layouts
 *    - Static layouts for precise control
 *    - Row begin/end for complex layouts
 * 
 * 5. CUSTOM WIDGETS
 *    - nk_progress_bar_ex() - Enhanced progress bar
 *    - Custom voltage chart rendering
 *    - Cell voltage bars with manual drawing
 * 
 * 6. STYLING
 *    - Color-coded status indicators
 *    - Conditional coloring based on values
 *    - Professional color scheme
 * 
 * 7. MULTI-SCREEN NAVIGATION
 *    - Tab-based interface
 *    - State management
 *    - Screen-specific rendering
 * 
 * TOUCH INPUT INTEGRATION (To be implemented):
 * 
 * In nk_input_begin/end section, add:
 * 
 * uint16_t touchX, touchY;
 * bool touched = getTouchCoordinates(&touchX, &touchY);
 * 
 * if (touched) {
 *   nk_input_motion(&ctx, touchX, touchY);
 *   nk_input_button(&ctx, NK_BUTTON_LEFT, touchX, touchY, 1);
 * } else {
 *   nk_input_button(&ctx, NK_BUTTON_LEFT, 0, 0, 0);
 * }
 * 
 * OPTIMIZATION TIPS:
 * 
 * 1. Dirty Rectangle Tracking
 *    - Only redraw changed regions
 *    - Significant performance boost
 * 
 * 2. Command Buffer Management
 *    - Allocate sufficient buffer size
 *    - Monitor memory usage
 * 
 * 3. Frame Rate Control
 *    - Adjust delay based on needs
 *    - Balance responsiveness vs power
 * 
 * 4. Selective Updates
 *    - Update BMS data less frequently
 *    - Only redraw when data changes
 * 
 * NUKLEAR VS TRADITIONAL GUI:
 * 
 * Advantages:
 * - No complex state management
 * - Easy to add/remove elements
 * - Highly customizable
 * - Single-header library
 * - Cross-platform
 * 
 * Considerations:
 * - Requires custom rendering backend
 * - Higher CPU usage (redraws each frame)
 * - Memory for command buffers
 * 
 * EXTENSION IDEAS:
 * 
 * 1. Add touch gesture support
 * 2. Implement scrollable panels
 * 3. Add data logging to SD card
 * 4. WiFi/Bluetooth configuration UI
 * 5. Alarm threshold settings
 * 6. Real-time current/voltage graphs
 * 7. Cell balancing control
 * 8. Export data functionality
 * 
 * HARDWARE CONNECTIONS:
 * 
 * For real BMS integration:
 * - I2C/SPI to BMS IC (e.g., BQ76940, LTC6811)
 * - Temperature sensors (NTC, DS18B20)
 * - Current shunt amplifier (INA219)
 * - Display (ILI9341, ST7789)
 * - Touch controller (XPT2046, FT6236)
 * 
 * CALIBRATION:
 * 
 * Add calibration screens for:
 * - Voltage offset correction
 * - Current sensor calibration
 * - Temperature compensation
 * - SOC curve adjustment
 */
