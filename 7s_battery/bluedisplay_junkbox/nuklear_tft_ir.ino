/*
 * ESP32 BMS Monitor with Nuklear GUI and RC5 IR Remote Control
 * Demonstrates Nuklear immediate-mode GUI with IR remote navigation
 * 
 * Libraries needed:
 * - Nuklear (single header: nuklear.h)
 * - TFT_eSPI or similar display driver
 * - IRremote library
 * - Wire.h (for I2C communication)
 * 
 * RC5 Remote Button Mapping:
 * - UP/DOWN: Navigate menu items
 * - LEFT/RIGHT: Adjust values
 * - OK/SELECT: Confirm selection
 * - 0-4: Quick tab switching
 * - MENU: Return to overview
 * - VOL+/VOL-: Scroll pages
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
#include <IRremote.h>
#include <Wire.h>

// IR Remote configuration
#define IR_RECEIVE_PIN 15  // GPIO pin for IR receiver
#define IR_DEBOUNCE_MS 200

// RC5 Command codes (standard RC5 remote)
#define RC5_CMD_0      0x00
#define RC5_CMD_1      0x01
#define RC5_CMD_2      0x02
#define RC5_CMD_3      0x03
#define RC5_CMD_4      0x04
#define RC5_CMD_UP     0x10
#define RC5_CMD_DOWN   0x11
#define RC5_CMD_LEFT   0x15
#define RC5_CMD_RIGHT  0x16
#define RC5_CMD_OK     0x17
#define RC5_CMD_MENU   0x12
#define RC5_CMD_VOLUP  0x10
#define RC5_CMD_VOLDN  0x11
#define RC5_CMD_CHUP   0x20
#define RC5_CMD_CHDN   0x21
#define RC5_CMD_POWER  0x0C
#define RC5_CMD_MUTE   0x0D

// Display configuration
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

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

// Nuklear context and GUI state
struct nk_context ctx;
struct nk_user_font font;

// Navigation state for IR remote
struct NavState {
  int activeTab;
  int selectedItem;
  int maxItems;
  bool editMode;
  float *editValue;
  float editMin;
  float editMax;
  float editStep;
} navState = {0, 0, 0, false, nullptr, 0, 0, 0};

// GUI state
static float voltageHistory[50];
static int historyIndex = 0;
static unsigned long lastUpdate = 0;
static unsigned long lastIRTime = 0;

// Settings
static float targetVoltage = 4.1f;
static float targetCurrent = 2.0f;
static float balanceThreshold = 0.05f;
static float tempWarningLimit = 40.0f;

// IR state
static uint8_t lastIRCommand = 0;
static bool irCommandReady = false;

// Color conversion helper
uint16_t nk_color_to_rgb565(struct nk_color col) {
  uint16_t r = (col.r >> 3) & 0x1F;
  uint16_t g = (col.g >> 2) & 0x3F;
  uint16_t b = (col.b >> 3) & 0x1F;
  return (r << 11) | (g << 5) | b;
}

// IR Remote handler
void handleIRCommand(uint8_t command) {
  unsigned long now = millis();
  if (now - lastIRTime < IR_DEBOUNCE_MS) return;
  lastIRTime = now;
  
  // Handle edit mode separately
  if (navState.editMode && navState.editValue != nullptr) {
    switch(command) {
      case RC5_CMD_RIGHT:
      case RC5_CMD_VOLUP:
        *navState.editValue += navState.editStep;
        if (*navState.editValue > navState.editMax) 
          *navState.editValue = navState.editMax;
        break;
        
      case RC5_CMD_LEFT:
      case RC5_CMD_VOLDN:
        *navState.editValue -= navState.editStep;
        if (*navState.editValue < navState.editMin) 
          *navState.editValue = navState.editMin;
        break;
        
      case RC5_CMD_OK:
        navState.editMode = false;
        navState.editValue = nullptr;
        break;
        
      case RC5_CMD_MENU:
        navState.editMode = false;
        navState.editValue = nullptr;
        navState.activeTab = 0;
        navState.selectedItem = 0;
        break;
    }
    return;
  }
  
  // Normal navigation mode
  switch(command) {
    case RC5_CMD_0:
      navState.activeTab = 0;
      navState.selectedItem = 0;
      break;
      
    case RC5_CMD_1:
      navState.activeTab = 1;
      navState.selectedItem = 0;
      break;
      
    case RC5_CMD_2:
      navState.activeTab = 2;
      navState.selectedItem = 0;
      break;
      
    case RC5_CMD_3:
      navState.activeTab = 3;
      navState.selectedItem = 0;
      break;
      
    case RC5_CMD_4:
      navState.activeTab = 4;
      navState.selectedItem = 0;
      break;
      
    case RC5_CMD_UP:
    case RC5_CMD_CHUP:
      if (navState.selectedItem > 0) navState.selectedItem--;
      break;
      
    case RC5_CMD_DOWN:
    case RC5_CMD_CHDN:
      if (navState.selectedItem < navState.maxItems - 1) navState.selectedItem++;
      break;
      
    case RC5_CMD_LEFT:
      if (navState.activeTab > 0) {
        navState.activeTab--;
        navState.selectedItem = 0;
      }
      break;
      
    case RC5_CMD_RIGHT:
      if (navState.activeTab < 4) {
        navState.activeTab++;
        navState.selectedItem = 0;
      }
      break;
      
    case RC5_CMD_OK:
      // Enter edit mode for settings
      if (navState.activeTab == 4) {
        enterEditMode();
      }
      break;
      
    case RC5_CMD_MENU:
      navState.activeTab = 0;
      navState.selectedItem = 0;
      break;
  }
}

// Enter edit mode for settings
void enterEditMode() {
  navState.editMode = true;
  
  switch(navState.selectedItem) {
    case 0: // Target Voltage
      navState.editValue = &targetVoltage;
      navState.editMin = 3.0f;
      navState.editMax = 4.2f;
      navState.editStep = 0.01f;
      break;
      
    case 1: // Target Current
      navState.editValue = &targetCurrent;
      navState.editMin = 0.1f;
      navState.editMax = 5.0f;
      navState.editStep = 0.1f;
      break;
      
    case 2: // Balance Threshold
      navState.editValue = &balanceThreshold;
      navState.editMin = 0.01f;
      navState.editMax = 0.20f;
      navState.editStep = 0.01f;
      break;
      
    case 3: // Temp Warning
      navState.editValue = &tempWarningLimit;
      navState.editMin = 30.0f;
      navState.editMax = 50.0f;
      navState.editStep = 1.0f;
      break;
      
    default:
      navState.editMode = false;
      navState.editValue = nullptr;
      break;
  }
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
  return len * 6;
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
  
  float avgVoltage = bms.packVoltage / 8;
  for (int i = 0; i < 8; i++) {
    bms.balancing[i] = (bms.cellVoltages[i] - avgVoltage) > balanceThreshold;
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
  
  voltageHistory[historyIndex] = bms.packVoltage;
  historyIndex = (historyIndex + 1) % 50;
}

// Draw selection indicator
void drawSelectionIndicator(struct nk_context *ctx, int itemIndex) {
  if (navState.selectedItem == itemIndex) {
    struct nk_rect bounds = nk_widget_bounds(ctx);
    struct nk_command_buffer *out = nk_window_get_canvas(ctx);
    
    struct nk_color highlight = navState.editMode ? 
                                nk_rgb(255, 200, 100) : nk_rgb(100, 200, 255);
    nk_stroke_rect(out, nk_rect(bounds.x - 2, bounds.y - 2, bounds.w + 4, bounds.h + 4),
                   0, 2, highlight);
  }
}

// Custom progress bar widget
void nk_progress_bar_ex(struct nk_context *ctx, float value, float max, 
                        struct nk_color fg, struct nk_color bg, const char *label, int itemIdx) {
  struct nk_rect bounds;
  struct nk_command_buffer *out = nk_window_get_canvas(ctx);
  
  nk_layout_row_dynamic(ctx, 20, 1);
  bounds = nk_widget_bounds(ctx);
  
  if (nk_widget(&bounds, ctx)) {
    float percent = value / max;
    
    nk_fill_rect(out, bounds, 2, bg);
    
    struct nk_rect fill = bounds;
    fill.w = bounds.w * percent;
    nk_fill_rect(out, fill, 2, fg);
    
    nk_stroke_rect(out, bounds, 2, 1, nk_rgb(100, 100, 100));
    
    if (label) {
      char text[64];
      snprintf(text, sizeof(text), "%s: %.1f%%", label, percent * 100.0f);
      nk_draw_text(out, bounds, text, strlen(text), &font, 
                   nk_rgba(0,0,0,0), nk_rgb(255, 255, 255));
    }
    
    drawSelectionIndicator(ctx, itemIdx);
  }
}

// Draw overview tab
void drawOverviewTab(struct nk_context *ctx) {
  int itemIdx = 0;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Pack Voltage:", NK_TEXT_LEFT);
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f V", bms.packVoltage);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, nk_rgb(100, 255, 100));
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Current:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.2f A", bms.current);
  struct nk_color cur_col = bms.charging ? nk_rgb(100, 255, 100) : nk_rgb(255, 200, 100);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, cur_col);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 5, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  struct nk_color soc_fg = bms.soc > 30 ? nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
  struct nk_color soc_bg = nk_rgb(50, 50, 50);
  nk_progress_bar_ex(ctx, bms.soc, 100.0f, soc_fg, soc_bg, "SOC", itemIdx++);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "State of Health:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.1f%%", bms.soh);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, nk_rgb(100, 200, 255));
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Cycle Count:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%u", bms.cycleCount);
  nk_label(ctx, buf, NK_TEXT_RIGHT);
  itemIdx++;
  
  float avgTemp = 0;
  for (int i = 0; i < 4; i++) avgTemp += bms.temperature[i];
  avgTemp /= 4;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Avg Temperature:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.1f C", avgTemp);
  struct nk_color temp_col = avgTemp < tempWarningLimit ? nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, temp_col);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Status:", NK_TEXT_LEFT);
  const char *status = bms.charging ? "CHARGING" : "DISCHARGING";
  nk_label_colored(ctx, status, NK_TEXT_RIGHT, nk_rgb(100, 255, 100));
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 5, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  nk_layout_row_dynamic(ctx, 20, 1);
  snprintf(buf, sizeof(buf), "Cell Delta: %.3f V (Min: #%d, Max: #%d)", 
           bms.maxCellV - bms.minCellV, bms.minCellIdx + 1, bms.maxCellIdx + 1);
  struct nk_color delta_col = (bms.maxCellV - bms.minCellV) < 0.1 ? 
                               nk_rgb(100, 255, 100) : nk_rgb(255, 200, 100);
  nk_label_colored(ctx, buf, NK_TEXT_CENTERED, delta_col);
  
  navState.maxItems = itemIdx;
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
    
    nk_layout_row_push(ctx, 40);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", i + 1);
    nk_label(ctx, buf, NK_TEXT_CENTERED);
    
    nk_layout_row_push(ctx, 70);
    snprintf(buf, sizeof(buf), "%.3f V", bms.cellVoltages[i]);
    nk_label(ctx, buf, NK_TEXT_RIGHT);
    
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
      
      if (navState.selectedItem == i) {
        struct nk_color highlight = nk_rgb(100, 200, 255);
        nk_stroke_rect(out, nk_rect(bounds.x - 2, bounds.y - 2, bounds.w + 4, bounds.h + 4),
                       0, 3, highlight);
      }
    }
    
    nk_layout_row_push(ctx, 50);
    if (bms.balancing[i]) {
      nk_label_colored(ctx, "BAL", NK_TEXT_CENTERED, nk_rgb(255, 200, 100));
    } else {
      nk_label(ctx, "-", NK_TEXT_CENTERED);
    }
    
    nk_layout_row_end(ctx);
  }
  
  navState.maxItems = 8;
}

// Draw temperature tab
void drawTempTab(struct nk_context *ctx) {
  for (int i = 0; i < 4; i++) {
    nk_layout_row_dynamic(ctx, 20, 2);
    
    if (navState.selectedItem == i) drawSelectionIndicator(ctx, i);
    
    char buf[32];
    snprintf(buf, sizeof(buf), "Sensor %d:", i + 1);
    nk_label(ctx, buf, NK_TEXT_LEFT);
    
    snprintf(buf, sizeof(buf), "%.1f C", bms.temperature[i]);
    struct nk_color temp_col = bms.temperature[i] < tempWarningLimit ? 
                               nk_rgb(100, 255, 100) : nk_rgb(255, 100, 100);
    nk_label_colored(ctx, buf, NK_TEXT_RIGHT, temp_col);
    
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
    nk_label(ctx, "", NK_TEXT_LEFT);
  }
  
  bool highTemp = false;
  for (int i = 0; i < 4; i++) {
    if (bms.temperature[i] > tempWarningLimit) highTemp = true;
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
  
  navState.maxItems = 4;
}

// Draw history tab
void drawHistoryTab(struct nk_context *ctx) {
  nk_layout_row_dynamic(ctx, 20, 1);
  nk_label(ctx, "Voltage History (Last 50 samples)", NK_TEXT_CENTERED);
  
  nk_layout_row_dynamic(ctx, 100, 1);
  struct nk_rect bounds = nk_widget_bounds(ctx);
  
  if (nk_widget(&bounds, ctx)) {
    struct nk_command_buffer *out = nk_window_get_canvas(ctx);
    
    nk_fill_rect(out, bounds, 2, nk_rgb(30, 30, 40));
    nk_stroke_rect(out, bounds, 2, 1, nk_rgb(100, 100, 100));
    
    float minV = voltageHistory[0], maxV = voltageHistory[0];
    for (int i = 1; i < 50; i++) {
      if (voltageHistory[i] < minV) minV = voltageHistory[i];
      if (voltageHistory[i] > maxV) maxV = voltageHistory[i];
    }
    float range = maxV - minV;
    if (range < 0.1) range = 0.1;
    
    for (int i = 0; i < 49; i++) {
      int idx1 = (historyIndex + i) % 50;
      int idx2 = (historyIndex + i + 1) % 50;
      
      float x1 = bounds.x + (bounds.w * i / 49.0f);
      float y1 = bounds.y + bounds.h - ((voltageHistory[idx1] - minV) / range) * bounds.h;
      float x2 = bounds.x + (bounds.w * (i + 1) / 49.0f);
      float y2 = bounds.y + bounds.h - ((voltageHistory[idx2] - minV) / range) * bounds.h;
      
      nk_stroke_line(out, x1, y1, x2, y2, 2, nk_rgb(100, 255, 100));
    }
    
    for (int i = 1; i < 4; i++) {
      float y = bounds.y + (bounds.h * i / 4.0f);
      nk_stroke_line(out, bounds.x, y, bounds.x + bounds.w, y, 
                    1, nk_rgb(60, 60, 70));
    }
  }
  
  int itemIdx = 0;
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Total Cycles:", NK_TEXT_LEFT);
  char buf[32];
  snprintf(buf, sizeof(buf), "%u", bms.cycleCount);
  nk_label(ctx, buf, NK_TEXT_RIGHT);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Peak Discharge:", NK_TEXT_LEFT);
  nk_label(ctx, "8.2 A", NK_TEXT_RIGHT);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Peak Charge:", NK_TEXT_LEFT);
  nk_label(ctx, "5.5 A", NK_TEXT_RIGHT);
  itemIdx++;
  
  navState.maxItems = itemIdx;
}

// Draw settings tab
void drawSettingsTab(struct nk_context *ctx) {
  nk_layout_row_dynamic(ctx, 25, 1);
  nk_label(ctx, "Charge Settings", NK_TEXT_CENTERED);
  
  int itemIdx = 0;
  char buf[64];
  
  // Target Voltage
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Target Voltage:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.2f V %s", targetVoltage, 
           (navState.editMode && navState.selectedItem == itemIdx) ? "<EDIT>" : "");
  struct nk_color editColor = (navState.editMode && navState.selectedItem == itemIdx) ? 
                              nk_rgb(255, 200, 100) : nk_rgb(200, 200, 200);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, editColor);
  itemIdx++;
  
  // Target Current
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Target Current:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.1f A %s", targetCurrent,
           (navState.editMode && navState.selectedItem == itemIdx) ? "<EDIT>" : "");
  editColor = (navState.editMode && navState.selectedItem == itemIdx) ? 
              nk_rgb(255, 200, 100) : nk_rgb(200, 200, 200);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, editColor);
  itemIdx++;
  
  // Balance Threshold
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Balance Threshold:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.3f V %s", balanceThreshold,
           (navState.editMode && navState.selectedItem == itemIdx) ? "<EDIT>" : "");
  editColor = (navState.editMode && navState.selectedItem == itemIdx) ? 
              nk_rgb(255, 200, 100) : nk_rgb(200, 200, 200);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, editColor);
  itemIdx++;
  
  // Temp Warning Limit
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Temp Warning:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%.0f C %s", tempWarningLimit,
           (navState.editMode && navState.selectedItem == itemIdx) ? "<EDIT>" : "");
  editColor = (navState.editMode && navState.selectedItem == itemIdx) ? 
              nk_rgb(255, 200, 100) : nk_rgb(200, 200, 200);
  nk_label_colored(ctx, buf, NK_TEXT_RIGHT, editColor);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 15, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  nk_layout_row_dynamic(ctx, 20, 1);
  nk_label(ctx, "System Info", NK_TEXT_CENTERED);
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Firmware:", NK_TEXT_LEFT);
  nk_label(ctx, "v1.0.3", NK_TEXT_RIGHT);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "BMS Type:", NK_TEXT_LEFT);
  nk_label(ctx, "8S Li-Ion", NK_TEXT_RIGHT);
  itemIdx++;
  
  nk_layout_row_dynamic(ctx, 20, 2);
  if (navState.selectedItem == itemIdx) drawSelectionIndicator(ctx, itemIdx);
  nk_label(ctx, "Uptime:", NK_TEXT_LEFT);
  snprintf(buf, sizeof(buf), "%lu min", millis() / 60000);
  nk_label(ctx, buf, NK_TEXT_RIGHT);
  itemIdx++;
  
  // Instructions
  nk_layout_row_dynamic(ctx, 15, 1);
  nk_label(ctx, "", NK_TEXT_LEFT);
  
  nk_layout_row_dynamic(ctx, 15, 1);
  nk_label_colored(ctx, "IR Remote Controls:", NK_TEXT_CENTERED, nk_rgb(100, 200, 255));
  
  nk_layout_row_dynamic(ctx, 12, 1);
  nk_label(ctx, "UP/DOWN: Navigate  LEFT/RIGHT: Adjust", NK_TEXT_CENTERED);
  nk_layout_row_dynamic(ctx, 12, 1);
  nk_label(ctx, "OK: Edit Mode  MENU: Overview", NK_TEXT_CENTERED);
  nk_layout_row_dynamic(ctx, 12, 1);
  nk_label(ctx, "0-4: Quick Tab Switch", NK_TEXT_CENTERED);
  
  navState.maxItems = itemIdx;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize TFT
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);
  
  // Initialize IR Receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  Serial.println("IR Receiver initialized on pin " + String(IR_RECEIVE_PIN));
  
  // Initialize Nuklear
  nk_init_default(&ctx, &font);
  
  font.userdata.ptr = NULL;
  font.height = 12;
  font.width = nk_tft_font_width;
  
  // Initialize BMS data
  initBMSData();
  
  Serial.println("Nuklear BMS Monitor with RC5 IR Remote initialized");
  Serial.println("RC5 Remote Commands:");
  Serial.println("  0-4: Switch tabs");
  Serial.println("  UP/DOWN: Navigate items");
  Serial.println("  LEFT/RIGHT: Previous/Next tab or adjust values");
  Serial.println("  OK: Enter edit mode (Settings tab)");
  Serial.println("  MENU: Return to overview");
}

void loop() {
  // Check for IR commands
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == RC5) {
      uint8_t command = IrReceiver.decodedIRData.command;
      
      Serial.print("RC5 Command received: 0x");
      Serial.println(command, HEX);
      
      handleIRCommand(command);
    }
    
    IrReceiver.resume();
  }
  
  // Update BMS data periodically
  if (millis() - lastUpdate > 500) {
    updateBMSData();
    lastUpdate = millis();
  }
  
  // Start Nuklear frame
  nk_input_begin(&ctx);
  nk_input_end(&ctx);
  
  // Main window
  if (nk_begin(&ctx, "BMS Monitor", nk_rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT),
               NK_WINDOW_BORDER | NK_WINDOW_TITLE)) {
    
    // Tab bar with highlighting
    nk_layout_row_static(&ctx, 25, 60, 5);
    
    struct nk_style_button style = ctx.style.button;
    
    // Overview button
    if (navState.activeTab == 0) ctx.style.button.normal.data.color = nk_rgb(100, 200, 255);
    if (nk_button_label(&ctx, "Overview")) navState.activeTab = 0;
    ctx.style.button = style;
    
    // Cells button
    if (navState.activeTab == 1) ctx.style.button.normal.data.color = nk_rgb(100, 200, 255);
    if (nk_button_label(&ctx, "Cells")) navState.activeTab = 1;
    ctx.style.button = style;
    
    // Temp button
    if (navState.activeTab == 2) ctx.style.button.normal.data.color = nk_rgb(100, 200, 255);
    if (nk_button_label(&ctx, "Temp")) navState.activeTab = 2;
    ctx.style.button = style;
    
    // History button
    if (navState.activeTab == 3) ctx.style.button.normal.data.color = nk_rgb(100, 200, 255);
    if (nk_button_label(&ctx, "History")) navState.activeTab = 3;
    ctx.style.button = style;
    
    // Settings button
    if (navState.activeTab == 4) ctx.style.button.normal.data.color = nk_rgb(100, 200, 255);
    if (nk_button_label(&ctx, "Settings")) navState.activeTab = 4;
    ctx.style.button = style;
    
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
    switch(navState.activeTab) {
      case 0: drawOverviewTab(&ctx); break;
      case 1: drawCellsTab(&ctx); break;
      case 2: drawTempTab(&ctx); break;
      case 3: drawHistoryTab(&ctx); break;
      case 4: drawSettingsTab(&ctx); break;
    }
    
    // Edit mode indicator
    if (navState.editMode) {
      nk_layout_row_dynamic(&ctx, 15, 1);
      struct nk_rect bounds = nk_widget_bounds(&ctx);
      if (nk_widget(&bounds, &ctx)) {
        struct nk_command_buffer *out = nk_window_get_canvas(&ctx);
        nk_fill_rect(out, bounds, 2, nk_rgb(255, 200, 100));
        nk_draw_text(out, bounds, "EDIT MODE - Use LEFT/RIGHT to adjust, OK to confirm", 52, 
                     &font, nk_rgb(255, 200, 100), nk_rgb(0, 0, 0));
      }
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
 * RC5 IR REMOTE INTEGRATION FEATURES:
 * 
 * 1. NAVIGATION CONTROLS
 *    - UP/DOWN: Navigate through menu items
 *    - LEFT/RIGHT: Switch tabs or adjust values
 *    - Number keys (0-4): Quick tab switching
 *    - MENU: Return to overview screen
 * 
 * 2. EDIT MODE
 *    - OK button enters edit mode for settings
 *    - LEFT/RIGHT adjusts values in edit mode
 *    - OK confirms and exits edit mode
 *    - Visual feedback with orange highlighting
 * 
 * 3. VISUAL FEEDBACK
 *    - Blue outline shows selected item
 *    - Orange outline indicates edit mode
 *    - Tab buttons highlight when active
 *    - Status indicator at top right
 * 
 * 4. IR DEBOUNCING
 *    - 200ms debounce prevents multiple triggers
 *    - Smooth navigation experience
 * 
 * 5. SUPPORTED RC5 COMMANDS
 *    - 0x00-0x04: Tab selection (0-4)
 *    - 0x10: UP navigation
 *    - 0x11: DOWN navigation
 *    - 0x15: LEFT navigation
 *    - 0x16: RIGHT navigation
 *    - 0x17: OK/SELECT button
 *    - 0x12: MENU button
 *    - 0x20/0x21: Channel UP/DOWN (alternate navigation)
 * 
 * 6. CONFIGURABLE SETTINGS VIA IR
 *    - Target Voltage (3.0-4.2V, 0.01V steps)
 *    - Target Current (0.1-5.0A, 0.1A steps)
 *    - Balance Threshold (0.01-0.20V, 0.01V steps)
 *    - Temperature Warning (30-50°C, 1°C steps)
 * 
 * HARDWARE SETUP:
 * 
 * IR Receiver Module (TSOP382, VS1838B, etc.):
 * - VCC -> 3.3V or 5V
 * - GND -> GND
 * - OUT -> GPIO 15 (configurable via IR_RECEIVE_PIN)
 * 
 * TFT Display (ILI9341, ST7789, etc.):
 * - Configure pins in TFT_eSPI User_Setup.h
 * - Typical SPI connections
 * 
 * ALTERNATIVE IR PROTOCOLS:
 * 
 * This code uses RC5, but IRremote library supports:
 * - NEC
 * - Sony SIRC
 * - Samsung
 * - LG
 * - Panasonic
 * 
 * To use different protocol, change:
 * if (IrReceiver.decodedIRData.protocol == RC5)
 * to your protocol and update command codes.
 * 
 * CUSTOMIZATION:
 * 
 * 1. Adjust IR_DEBOUNCE_MS for responsiveness
 * 2. Change IR_RECEIVE_PIN for your board
 * 3. Map different remote buttons by updating RC5_CMD_* values
 * 4. Add new settings with additional edit modes
 * 5. Implement confirmation dialogs for critical actions
 * 
 * DEBUGGING:
 * 
 * - Serial monitor shows received IR commands
 * - Use IRremote examples to decode your remote
 * - Test individual buttons before mapping
 * 
 * ADVANCED FEATURES TO ADD:
 * 
 * 1. IR learning mode (auto-detect remote codes)
 * 2. Repeat handling for continuous adjustment
 * 3. Context-sensitive help screens
 * 4. Button combination shortcuts
 * 5. Save settings to EEPROM/NVS
 * 6. Multiple user profiles
 * 7. Gesture support (long press, double press)
 * 8. Macro recording (sequence of commands)
 */
