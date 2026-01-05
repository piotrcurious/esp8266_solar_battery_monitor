 url=https://github.com/piotrcurious/esp8266_solar_battery_monitor/blob/86a305df86a5a2292a072f261ce36cf2f4c6219b/bt_AP2_multi_humidity_better_graph2/fixes/helpers.ino
#include <Arduino.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <AsyncUDP.h>
#include "wifi_settings.h"
#include <BlueDisplay.hpp>
// ... other includes (truncated in original) -- add them here if needed

// --- Safety / fallbacks for missing macros (these should be defined in your headers) ---
#ifndef NUMBER_OF_GRAPHS
#define NUMBER_OF_GRAPHS 8
#endif

#ifndef LEGEND_LABEL_FONT_WIDTH
#define LEGEND_LABEL_FONT_WIDTH 8
#endif

#ifndef LEGEND_LABEL_FONT_SIZE
#define LEGEND_LABEL_FONT_SIZE 16
#endif

#ifndef LEGEND_LABEL_FONT_CHARS
#define LEGEND_LABEL_FONT_CHARS 4
#endif

#ifndef MAX_DISPLAYED_GRAPHS
#define MAX_DISPLAYED_GRAPHS 4
#endif

#ifndef MINUTES_GRAPH_BUFFER_MAX
#define MINUTES_GRAPH_BUFFER_MAX 256
#endif

#ifndef NUMBER_OF_BUFFERS
#define NUMBER_OF_BUFFERS 5
#endif

#ifndef GRAYING_FACTOR
#define GRAYING_FACTOR 0.4f
#endif

// --- Externs for globals defined elsewhere in your project ---
extern int8_t graph_order[];            // array with graph indices or -1
extern uint8_t graph_orderSize;         // how many entries are currently used /* ... */
extern const char *graph_labels[];      // labels for graphs (defined elsewhere) /* ... */

// display globals
extern uint16_t displayWidth;
extern uint16_t displayHeight;

// graph buffers (defined elsewhere)
extern float minutes_buffer[NUMBER_OF_BUFFERS][MINUTES_GRAPH_BUFFER_MAX];
extern float minutes_buffer_min[NUMBER_OF_BUFFERS];
extern float minutes_buffer_max[NUMBER_OF_BUFFERS];

/* scheduling structures */
extern volatile uint16_t lfsr;
extern bool graphComplete;
extern int current_graph;

/* telemetry frame declared in "telemetry_frame.hpp" */
extern telemetry_frame tframe;

/* networking / packet tracking */
extern volatile bool new_packet;
extern volatile uint32_t total_packets;

/* external helper functions (implemented elsewhere in your project) */
void initSerial(const char *name = nullptr);
void checkAndHandleEvents(void);
void registerSwipeEndCallback(void (cb)(struct Swipe)); /* ... other prototypes truncated ... */

// --- Local helper buffer ---
static char sStringBuffer[64]; // increased size and made static to avoid stack issues

// Function prototypes
void initializeArray();
void addNumber(int number);
void removeNumber(int number);
bool numberExists(int number);
void printArray(uint16_t x0 = 12 * 32, uint16_t y0 = 16);

// forward declarations for drawing helpers
color16_t toGrayishColor(color16_t color, float factor);
void plotGraphSection(float *data, uint16_t data_startpoint, uint16_t graph_data_size,
                      uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
                      float graph_min, float graph_max, color16_t graphColor, bool clear_under);
void plotGraph(float *data, uint16_t dataSize,
               uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
               float graph_min, float graph_max, color16_t graphColor, bool clear_under);
void drawLabels(float *data, uint16_t dataSize,
                uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
                float graph_min, float graph_max, color16_t graphColor);

// --- Implementation ---

void initializeArray()
{
    // initialize array to -1 and reset size
    for (int i = 0; i < NUMBER_OF_GRAPHS; i++) {
        graph_order[i] = -1;
    }
    graph_orderSize = 0;
}

void addNumber(int number)
{
    if (number < 0 || number >= NUMBER_OF_BUFFERS) return; // bounds check

    if (numberExists(number)) return;

    if (graph_orderSize < NUMBER_OF_GRAPHS) {
        graph_order[graph_orderSize] = number;
        graph_orderSize++;
    } else {
        // array full - drop oldest and append new (policy: FIFO)
        for (int i = 1; i < NUMBER_OF_GRAPHS; i++) {
            graph_order[i - 1] = graph_order[i];
        }
        graph_order[NUMBER_OF_GRAPHS - 1] = number;
    }
}

void removeNumber(int number)
{
    int index = -1;
    for (int i = 0; i < graph_orderSize; i++) {
        if (graph_order[i] == number) {
            index = i;
            break;
        }
    }
    if (index == -1) return;

    for (int i = index; i < (int)graph_orderSize - 1; i++) {
        graph_order[i] = graph_order[i + 1];
    }
    // clear last slot
    if (graph_orderSize > 0) {
        graph_orderSize--;
        graph_order[graph_orderSize] = -1;
    }
}

bool numberExists(int number)
{
    for (int i = 0; i < graph_orderSize; i++) {
        if (graph_order[i] == number) return true;
    }
    return false;
}

void printArray(uint16_t x0, uint16_t y0)
{
    // prints the configured graph_order array (up to NUMBER_OF_GRAPHS)
    for (int i = 0; i < NUMBER_OF_GRAPHS; i++) {
        if (graph_order[i] > -1) {
            snprintf(sStringBuffer, sizeof(sStringBuffer), "%d", graph_order[i]);
            // draw each label on its own line (adjust spacing as needed)
            BlueDisplay1.drawText(x0, y0 + i * LEGEND_LABEL_FONT_SIZE, sStringBuffer, LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);
        } else {
            // optionally draw placeholder or skip
        }
    }
}

// Safely blend a 16-bit RGB565 color towards gray using a floating factor in [0,1]
color16_t toGrayishColor(color16_t color, float factor)
{
    if (factor <= 0.0f) return color;
    if (factor >= 1.0f) {
        // convert fully to perceived gray
        float r = (float)((color >> 11) & 0x1F) / 31.0f;
        float g = (float)((color >> 5) & 0x3F) / 63.0f;
        float b = (float)(color & 0x1F) / 31.0f;
        float grayf = r * 0.299f + g * 0.587f + b * 0.114f;
        uint8_t rr = (uint8_t)roundf(grayf * 31.0f);
        uint8_t gg = (uint8_t)roundf(grayf * 63.0f);
        uint8_t bb = (uint8_t)roundf(grayf * 31.0f);
        return (color16_t)((rr << 11) | (gg << 5) | bb);
    }

    // extract components
    float r = (float)((color >> 11) & 0x1F) / 31.0f;
    float g = (float)((color >> 5) & 0x3F) / 63.0f;
    float b = (float)(color & 0x1F) / 31.0f;

    // perceived luminance
    float grayf = r * 0.299f + g * 0.587f + b * 0.114f;

    // blend
    float nr = r * (1.0f - factor) + grayf * factor;
    float ng = g * (1.0f - factor) + grayf * factor;
    float nb = b * (1.0f - factor) + grayf * factor;

    uint8_t rr = (uint8_t)roundf(nr * 31.0f);
    uint8_t gg = (uint8_t)roundf(ng * 63.0f);
    uint8_t bb = (uint8_t)roundf(nb * 31.0f);

    return (color16_t)((rr << 11) | (gg << 5) | bb);
}

// --- Display debug info ---
void DisplayDebug()
{
    int displayW = BlueDisplay1.getDisplayWidth();
    int displayH = BlueDisplay1.getDisplayHeight();
    uint16_t x0 = 0;

    uint16_t y0 = 16;       // label
    uint16_t y1 = 16 * 2;   // resolution
    uint16_t y2 = 16 * 3;   // voltage
    uint16_t y3 = 16 * 4;   // IP
    uint16_t y4 = 16 * 5;   // total packets

    // Clear debug area and draw frame
    BlueDisplay1.fillRect(x0, 0, LEGEND_LABEL_FONT_WIDTH * 16, y3, COLOR_BACKGROUND);
    BlueDisplay1.drawRect(x0, 0, LEGEND_LABEL_FONT_WIDTH * 16, y3, COLOR_FOREGROUND, 1);

    if (graphComplete) {
        if (current_graph >= 0) {
            BlueDisplay1.drawText(x0, y0, graph_labels[current_graph], LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);
        } else {
            printArray(LEGEND_LABEL_FONT_WIDTH * 8, LEGEND_LABEL_FONT_SIZE);
        }
    } else {
        if (current_graph >= 0) {
            BlueDisplay1.drawText(x0, y0, graph_labels[current_graph], LEGEND_LABEL_FONT_SIZE, COLOR_BACKGROUND, COLOR_FOREGROUND);
        }
    }

    snprintf(sStringBuffer, sizeof(sStringBuffer), "Res: x:%u, y:%u", displayW, displayH);
    BlueDisplay1.drawText(x0, y1, sStringBuffer, LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);

    // voltage (telemetry)
    snprintf(sStringBuffer, sizeof(sStringBuffer), "Voltage: %.3f", tframe.voltage_ADC0);
    BlueDisplay1.drawText(x0, y2, sStringBuffer, LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);

    snprintf(sStringBuffer, sizeof(sStringBuffer), "IP:%s", WiFi.softAPIP().toString().c_str());
    BlueDisplay1.drawText(x0, y3, sStringBuffer, LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);

    snprintf(sStringBuffer, sizeof(sStringBuffer), "packets: %lu", (unsigned long)total_packets);
    BlueDisplay1.drawText(x0, y4, sStringBuffer, LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);
}

// --- UDP packet handler ---
IRAM_ATTR void handlePacket(AsyncUDPPacket packet)
{
    // copy safely only the bytes available up to our telemetry_frame size
    size_t copySize = packet.length() < sizeof(tframe) ? packet.length() : sizeof(tframe);
    if (copySize > 0) {
        memcpy(&tframe, packet.data(), copySize);
        new_packet = true;
        total_packets++;
    }
}

// --- graph plotting helpers ---

// draw a single segment of the graph (safe and robust)
void plotGraphSection(float *data, uint16_t data_startpoint, uint16_t graph_data_size,
                      uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
                      float graph_min, float graph_max, color16_t graphColor, bool clear_under)
{
    // bounds & safety checks
    if (graph_data_size < 2) return;
    if (data_startpoint + 1 >= graph_data_size) return; // need a next point

    // avoid divide by zero
    if (fabsf(graph_max - graph_min) < 1e-6f) graph_max = graph_min + 1e-6f;

    float currentValue = data[data_startpoint];
    float nextValue = data[data_startpoint + 1];

    // compute number of segments and width of each segment
    uint16_t segWidth = (graphWidth + graph_data_size - 1) / graph_data_size; // ceil(graphWidth/graph_data_size)
    uint16_t x0 = graphPosX + (data_startpoint * graphWidth) / graph_data_size;

    if (clear_under) {
        // clear only the segment rectangle (x,y,width,height)
        BlueDisplay1.fillRect(x0, graphPosY, segWidth + 1, graphHeight, COLOR_BACKGROUND);
    }

    if (isnan(currentValue)) return;

    float scale = (graph_max - graph_min) / (float)graphHeight;
    if (fabsf(scale) < 1e-12f) scale = 1e-12f;
    uint16_t y0 = graphPosY + graphHeight - (uint16_t)((currentValue - graph_min) / scale);

    if (isnan(nextValue)) {
        BlueDisplay1.drawPixel(x0, y0, graphColor);
    } else {
        uint16_t x1 = graphPosX + ((data_startpoint + 1) * graphWidth) / graph_data_size;
        uint16_t y1 = graphPosY + graphHeight - (uint16_t)((nextValue - graph_min) / scale);
        BlueDisplay1.drawLine(x0, y0, x1, y1, graphColor);
    }
}

void plotGraph(float *data, uint16_t dataSize,
               uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
               float graph_min, float graph_max, color16_t graphColor, bool clear_under)
{
    if (dataSize < 2) return;

    // draw sections
    for (uint16_t i = 0; i < dataSize - 1; i++) {
        plotGraphSection(data, i, dataSize, graphPosX, graphPosY, graphWidth, graphHeight, graph_min, graph_max, graphColor, clear_under);
    }
}

// draw labels for a graph; minimal bounds checking
void drawLabels(float *data, uint16_t dataSize,
                uint16_t graphPosX, uint16_t graphPosY, uint16_t graphWidth, uint16_t graphHeight,
                float graph_min, float graph_max, color16_t graphColor)
{
    if (fabsf(graph_max - graph_min) < 1e-6f) graph_max = graph_min + 1e-6f;

    float yScale = (float)graphHeight / (graph_max - graph_min);
    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR);

    // bottom label
    snprintf(sStringBuffer, sizeof(sStringBuffer), "%.2f", graph_min);
    BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_WIDTH * LEGEND_LABEL_FONT_CHARS),
                          graphPosY + graphHeight - 8,
                          sStringBuffer, LEGEND_LABEL_FONT_SIZE, graphColor, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(graphPosX, graphPosY + graphHeight - 1, graphPosX + graphWidth, graphPosY + graphHeight - 1, COLOR16_RED);

    // top label
    snprintf(sStringBuffer, sizeof(sStringBuffer), "%.2f", graph_max);
    BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_WIDTH * LEGEND_LABEL_FONT_CHARS),
                          graphPosY + LEGEND_LABEL_FONT_SIZE,
                          sStringBuffer, LEGEND_LABEL_FONT_SIZE, graphColor, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(graphPosX, graphPosY, graphPosX + graphWidth, graphPosY, COLOR16_RED);

    // last value
    uint16_t lastIdx = dataSize > 1 ? (dataSize - 1) : 0;
    float lastVal = data[lastIdx];
    if (!isnan(lastVal)) {
        uint16_t lastDataY = graphPosY + graphHeight - (uint16_t)((lastVal - graph_min) * yScale);
        snprintf(sStringBuffer, sizeof(sStringBuffer), "%.2f", lastVal);
        BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_WIDTH * LEGEND_LABEL_FONT_CHARS),
                              lastDataY,
                              sStringBuffer, LEGEND_LABEL_FONT_SIZE, graphColor, COLOR_BACKGROUND);
        BlueDisplay1.drawLine(graphPosX, lastDataY, graphPosX + graphWidth, lastDataY, grayishColor);
    }
}

// --- Initialization & setup tweaks ---

void disableUnnecessaryBtFeatures()
{
    esp_bt_gap_cancel_discovery();
    esp_bt_sleep_disable();
}

void configureCoexistence()
{
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
}

bool btStartMinimal()
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    bt_cfg.bt_max_acl_conn = 1;
    bt_cfg.bt_max_sync_conn = 0;

    // initialize and enable controller in the requested mode
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        return false;
    }
    if (esp_bt_controller_enable(bt_cfg.mode) != ESP_OK) {
        return false;
    }
    // additional minimal configuration could be done here
    return true;
}

// --- NOTE: setup(), loop(), drawGui(), initDisplay(), handleSwipe() and scheduler functions remain the same
// because they depend heavily on your project's global variables and layout structures. Keep them in your main file.

// End of improved file
