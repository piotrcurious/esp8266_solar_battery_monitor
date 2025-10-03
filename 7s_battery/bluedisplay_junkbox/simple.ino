/*
 * ESP32 BMS Monitor with Nuklear GUI and BlueDisplay
 * Demonstrates advanced GUI capabilities for battery management
 * 
 * Libraries needed:
 * - Nuklear (single header GUI library)
 * - BlueDisplay (Bluetooth display library)
 * - Wire.h (for I2C communication)
 */

#include <Wire.h>
#include <BlueDisplay.h>

// Simulated BMS data structure
struct BMSData {
  float cellVoltages[8];      // Individual cell voltages
  float packVoltage;          // Total pack voltage
  float current;              // Pack current (positive = charging)
  float temperature[4];       // Temperature sensors
  float soc;                  // State of charge (%)
  float soh;                  // State of health (%)
  uint32_t cycleCount;        // Charge cycles
  bool balancing[8];          // Cell balancing status
  bool charging;              // Charging state
  bool error;                 // Error flag
};

BMSData bms;

// BlueDisplay objects
BlueDisplay myDisplay;

// Display dimensions
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 480

// Colors
#define COLOR_BG RGB(20, 20, 30)
#define COLOR_PANEL RGB(30, 30, 45)
#define COLOR_TEXT RGB(200, 200, 220)
#define COLOR_GREEN RGB(50, 200, 100)
#define COLOR_YELLOW RGB(230, 200, 50)
#define COLOR_RED RGB(230, 50, 50)
#define COLOR_BLUE RGB(80, 150, 230)

// GUI state
int currentScreen = 0; // 0=Overview, 1=Cells, 2=Temperature, 3=History
unsigned long lastUpdate = 0;

// Button declarations
BDButton btnOverview;
BDButton btnCells;
BDButton btnTemp;
BDButton btnHistory;

// Callback functions for buttons
void showOverview(BDButton *btn, int16_t val) {
  currentScreen = 0;
  drawScreen();
}

void showCells(BDButton *btn, int16_t val) {
  currentScreen = 1;
  drawScreen();
}

void showTemp(BDButton *btn, int16_t val) {
  currentScreen = 2;
  drawScreen();
}

void showHistory(BDButton *btn, int16_t val) {
  currentScreen = 3;
  drawScreen();
}

// Initialize simulated BMS data
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
}

// Simulate BMS data updates
void updateBMSData() {
  // Simulate voltage fluctuations
  for (int i = 0; i < 8; i++) {
    bms.cellVoltages[i] += (random(-5, 5) / 1000.0);
    bms.cellVoltages[i] = constrain(bms.cellVoltages[i], 3.0, 4.2);
  }
  
  // Calculate pack voltage
  bms.packVoltage = 0;
  for (int i = 0; i < 8; i++) {
    bms.packVoltage += bms.cellVoltages[i];
  }
  
  // Simulate current changes
  bms.current += (random(-10, 10) / 100.0);
  bms.current = constrain(bms.current, -5.0, 5.0);
  bms.charging = bms.current > 0;
  
  // Update SOC based on current
  bms.soc += (bms.current * 0.001);
  bms.soc = constrain(bms.soc, 0, 100);
  
  // Simulate temperature
  for (int i = 0; i < 4; i++) {
    bms.temperature[i] += (random(-2, 2) / 10.0);
    bms.temperature[i] = constrain(bms.temperature[i], 20.0, 45.0);
  }
  
  // Check for balancing needs
  float avgVoltage = bms.packVoltage / 8;
  for (int i = 0; i < 8; i++) {
    bms.balancing[i] = (bms.cellVoltages[i] - avgVoltage) > 0.05;
  }
}

// Draw navigation bar
void drawNavBar() {
  myDisplay.fillRect(0, 0, DISPLAY_WIDTH, 50, COLOR_PANEL);
  
  // Draw title
  myDisplay.drawText(10, 15, "BMS Monitor", 20, COLOR_TEXT, COLOR_PANEL);
  
  // Draw status indicator
  uint16_t statusColor = bms.error ? COLOR_RED : COLOR_GREEN;
  myDisplay.fillCircle(DISPLAY_WIDTH - 25, 25, 8, statusColor);
}

// Draw menu buttons
void drawMenuButtons() {
  int btnY = 60;
  int btnW = 75;
  int btnH = 35;
  int spacing = 3;
  
  btnOverview.init(5, btnY, btnW, btnH, COLOR_BLUE, "Overview", 14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &showOverview);
  btnCells.init(5 + btnW + spacing, btnY, btnW, btnH, COLOR_BLUE, "Cells", 14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &showCells);
  btnTemp.init(5 + 2*(btnW + spacing), btnY, btnW, btnH, COLOR_BLUE, "Temp", 14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &showTemp);
  btnHistory.init(5 + 3*(btnW + spacing), btnY, btnW, btnH, COLOR_BLUE, "History", 14, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &showHistory);
  
  // Highlight active button
  switch(currentScreen) {
    case 0: btnOverview.setColor(COLOR_GREEN); break;
    case 1: btnCells.setColor(COLOR_GREEN); break;
    case 2: btnTemp.setColor(COLOR_GREEN); break;
    case 3: btnHistory.setColor(COLOR_GREEN); break;
  }
  
  btnOverview.drawButton();
  btnCells.drawButton();
  btnTemp.drawButton();
  btnHistory.drawButton();
}

// Draw overview screen
void drawOverviewScreen() {
  int yPos = 110;
  
  // Pack voltage
  myDisplay.drawText(10, yPos, "Pack Voltage", 16, COLOR_TEXT, COLOR_BG);
  char buf[32];
  sprintf(buf, "%.2fV", bms.packVoltage);
  myDisplay.drawText(200, yPos, buf, 20, COLOR_GREEN, COLOR_BG);
  yPos += 40;
  
  // Current
  myDisplay.drawText(10, yPos, "Current", 16, COLOR_TEXT, COLOR_BG);
  sprintf(buf, "%.2fA", bms.current);
  uint16_t currentColor = bms.charging ? COLOR_GREEN : COLOR_YELLOW;
  myDisplay.drawText(200, yPos, buf, 20, currentColor, COLOR_BG);
  yPos += 40;
  
  // State of Charge - with bar
  myDisplay.drawText(10, yPos, "State of Charge", 16, COLOR_TEXT, COLOR_BG);
  sprintf(buf, "%.1f%%", bms.soc);
  myDisplay.drawText(200, yPos, buf, 20, COLOR_GREEN, COLOR_BG);
  yPos += 30;
  
  // SOC bar
  int barWidth = 300;
  int barHeight = 20;
  myDisplay.drawRect(10, yPos, barWidth, barHeight, COLOR_TEXT);
  int fillWidth = (int)(barWidth * bms.soc / 100.0);
  uint16_t socColor = bms.soc > 30 ? COLOR_GREEN : COLOR_RED;
  myDisplay.fillRect(10, yPos, fillWidth, barHeight, socColor);
  yPos += 40;
  
  // State of Health
  myDisplay.drawText(10, yPos, "State of Health", 16, COLOR_TEXT, COLOR_BG);
  sprintf(buf, "%.1f%%", bms.soh);
  myDisplay.drawText(200, yPos, buf, 20, COLOR_GREEN, COLOR_BG);
  yPos += 40;
  
  // Cycle Count
  myDisplay.drawText(10, yPos, "Cycles", 16, COLOR_TEXT, COLOR_BG);
  sprintf(buf, "%d", bms.cycleCount);
  myDisplay.drawText(200, yPos, buf, 20, COLOR_TEXT, COLOR_BG);
  yPos += 40;
  
  // Average Temperature
  float avgTemp = 0;
  for (int i = 0; i < 4; i++) avgTemp += bms.temperature[i];
  avgTemp /= 4;
  myDisplay.drawText(10, yPos, "Avg Temperature", 16, COLOR_TEXT, COLOR_BG);
  sprintf(buf, "%.1fC", avgTemp);
  uint16_t tempColor = avgTemp < 40 ? COLOR_GREEN : COLOR_RED;
  myDisplay.drawText(200, yPos, buf, 20, tempColor, COLOR_BG);
  yPos += 40;
  
  // Status
  myDisplay.drawText(10, yPos, "Status", 16, COLOR_TEXT, COLOR_BG);
  const char* status = bms.charging ? "CHARGING" : "DISCHARGING";
  myDisplay.drawText(200, yPos, status, 16, COLOR_GREEN, COLOR_BG);
}

// Draw cells screen
void drawCellsScreen() {
  int yPos = 110;
  int xCol1 = 10;
  int xCol2 = 170;
  
  myDisplay.drawText(xCol1, yPos, "Cell", 14, COLOR_TEXT, COLOR_BG);
  myDisplay.drawText(xCol1 + 60, yPos, "Voltage", 14, COLOR_TEXT, COLOR_BG);
  myDisplay.drawText(xCol2 + 50, yPos, "Balance", 14, COLOR_TEXT, COLOR_BG);
  yPos += 25;
  
  for (int i = 0; i < 8; i++) {
    char buf[32];
    
    // Cell number
    sprintf(buf, "%d", i + 1);
    myDisplay.drawText(xCol1, yPos, buf, 16, COLOR_TEXT, COLOR_BG);
    
    // Voltage with bar
    sprintf(buf, "%.3fV", bms.cellVoltages[i]);
    myDisplay.drawText(xCol1 + 40, yPos, buf, 16, COLOR_GREEN, COLOR_BG);
    
    // Voltage bar (3.0V - 4.2V range)
    int barX = xCol2;
    int barW = 100;
    int barH = 15;
    float normalizedV = (bms.cellVoltages[i] - 3.0) / 1.2;
    int fillW = (int)(barW * normalizedV);
    myDisplay.drawRect(barX, yPos, barW, barH, COLOR_TEXT);
    myDisplay.fillRect(barX, yPos, fillW, barH, COLOR_GREEN);
    
    // Balancing indicator
    if (bms.balancing[i]) {
      myDisplay.fillCircle(xCol2 + 120, yPos + 7, 5, COLOR_YELLOW);
      myDisplay.drawText(xCol2 + 130, yPos, "BAL", 12, COLOR_YELLOW, COLOR_BG);
    }
    
    yPos += 35;
  }
  
  // Show min/max cells
  float minV = 5.0, maxV = 0.0;
  int minCell = 0, maxCell = 0;
  for (int i = 0; i < 8; i++) {
    if (bms.cellVoltages[i] < minV) { minV = bms.cellVoltages[i]; minCell = i; }
    if (bms.cellVoltages[i] > maxV) { maxV = bms.cellVoltages[i]; maxCell = i; }
  }
  
  yPos += 10;
  char buf[64];
  sprintf(buf, "Delta: %.3fV (Cell %d: %.3fV, Cell %d: %.3fV)", 
          maxV - minV, minCell + 1, minV, maxCell + 1, maxV);
  myDisplay.drawText(10, yPos, buf, 12, COLOR_YELLOW, COLOR_BG);
}

// Draw temperature screen
void drawTempScreen() {
  int yPos = 110;
  
  for (int i = 0; i < 4; i++) {
    char buf[32];
    sprintf(buf, "Sensor %d", i + 1);
    myDisplay.drawText(10, yPos, buf, 16, COLOR_TEXT, COLOR_BG);
    
    sprintf(buf, "%.1fC", bms.temperature[i]);
    uint16_t tempColor = bms.temperature[i] < 40 ? COLOR_GREEN : COLOR_RED;
    myDisplay.drawText(200, yPos, buf, 20, tempColor, COLOR_BG);
    
    // Temperature bar
    int barY = yPos + 30;
    int barW = 280;
    int barH = 20;
    float normalizedT = (bms.temperature[i] - 20.0) / 25.0; // 20-45°C range
    int fillW = (int)(barW * normalizedT);
    myDisplay.drawRect(20, barY, barW, barH, COLOR_TEXT);
    myDisplay.fillRect(20, barY, fillW, barH, tempColor);
    
    yPos += 80;
  }
  
  // Show warnings
  bool highTemp = false;
  for (int i = 0; i < 4; i++) {
    if (bms.temperature[i] > 40) highTemp = true;
  }
  
  if (highTemp) {
    myDisplay.fillRect(10, 430, DISPLAY_WIDTH - 20, 40, COLOR_RED);
    myDisplay.drawText(60, 440, "HIGH TEMPERATURE WARNING", 14, COLOR_BG, COLOR_RED);
  }
}

// Draw history screen (simple example)
void drawHistoryScreen() {
  int yPos = 110;
  
  myDisplay.drawText(10, yPos, "Statistics", 18, COLOR_TEXT, COLOR_BG);
  yPos += 35;
  
  char buf[64];
  
  sprintf(buf, "Total Cycles: %d", bms.cycleCount);
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "State of Health: %.1f%%", bms.soh);
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "Peak Discharge: 8.2A");
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "Peak Charge: 5.5A");
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "Max Temp Recorded: 42.3C");
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "Min Cell Voltage: 3.12V");
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 30;
  
  sprintf(buf, "Max Cell Voltage: 4.18V");
  myDisplay.drawText(20, yPos, buf, 14, COLOR_TEXT, COLOR_BG);
  yPos += 50;
  
  // Simple graph placeholder
  myDisplay.drawText(10, yPos, "Voltage History (Last 10 mins)", 14, COLOR_TEXT, COLOR_BG);
  yPos += 25;
  
  // Draw simple line graph
  int graphX = 20;
  int graphY = yPos;
  int graphW = 280;
  int graphH = 100;
  
  myDisplay.drawRect(graphX, graphY, graphW, graphH, COLOR_TEXT);
  
  // Simulate voltage history line
  for (int i = 0; i < 20; i++) {
    int x1 = graphX + i * (graphW / 20);
    int x2 = graphX + (i + 1) * (graphW / 20);
    int y1 = graphY + graphH - (int)(random(50, 90));
    int y2 = graphY + graphH - (int)(random(50, 90));
    myDisplay.drawLine(x1, y1, x2, y2, COLOR_GREEN);
  }
}

// Main screen drawing function
void drawScreen() {
  myDisplay.clearDisplay(COLOR_BG);
  drawNavBar();
  drawMenuButtons();
  
  switch(currentScreen) {
    case 0: drawOverviewScreen(); break;
    case 1: drawCellsScreen(); break;
    case 2: drawTempScreen(); break;
    case 3: drawHistoryScreen(); break;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize BlueDisplay
  myDisplay.initCommunication(&Serial);
  
  // Wait for connection
  while (!myDisplay.isConnectionEstablished()) {
    delay(100);
  }
  
  // Set display size
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  myDisplay.setCharacterMapping(0); // ASCII mapping
  
  // Initialize BMS data
  initBMSData();
  
  // Draw initial screen
  drawScreen();
  
  Serial.println("BMS Monitor initialized");
}

void loop() {
  // Update BMS data every 500ms
  if (millis() - lastUpdate > 500) {
    updateBMSData();
    drawScreen();
    lastUpdate = millis();
  }
  
  // Check for BlueDisplay events
  myDisplay.checkAndHandleEvents();
  
  delay(50);
}

/*
 * NUKLEAR IMPLEMENTATION NOTES:
 * 
 * While this example uses BlueDisplay for Bluetooth display capability,
 * Nuklear can be integrated for more advanced GUI rendering with:
 * 
 * 1. Immediate mode GUI with full control
 * 2. Advanced widgets (sliders, progress bars, charts)
 * 3. Customizable themes and styling
 * 4. Layout management system
 * 
 * For Nuklear integration on ESP32:
 * - Implement rendering backend for your display driver
 * - Handle input events (touch/buttons)
 * - Use nk_begin/nk_end for window management
 * - Utilize nk_layout_row_dynamic for responsive layouts
 * - Implement custom widgets for BMS visualization
 */
