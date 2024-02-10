// Adafruit OLED display library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display dimensions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Display object
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Declaration for SSD1306 display connected using software SPI (default case):
/*
#define OLED_MOSI   D7 
#define OLED_CLK    D5
#define OLED_DC     D2
#define OLED_CS     D8
#define OLED_RESET  D3
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/
///*
// Comment out above, uncomment this block to use hardware SPI
#define OLED_DC     D2
#define OLED_CS     D8
#define OLED_RESET  D3
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);
//*/

// Define the widget element struct
// Each widget element has a name, a label, a position, a font size, an update rate, and a display mode
// The display modes are: 0 for hex, 1 for dec, 2 for bar, 3 for rolling graph, 4 for analog gauge
// The parameters for each mode are: min, max, color, width, height, radius, etc.
struct WidgetElement {
  String name; // The name of the widget element
  String label; // The label to show on the display
  int x; // The x position of the label
  int y; // The y position of the label
  int font; // The font size of the label
  int rate; // The update rate of the widget element in milliseconds
  int mode; // The display mode of the widget element
  int params[10]; // The parameters for the display mode
};

// Define the layout array
// Each layout is an array of widget elements
// You can add or remove layouts as you wish
WidgetElement layouts[][5] = {
  // Layout 1: temperature, humidity, light, sound, pressure
  {
    // Widget element 1: temperature in hex
    {
      "temp", // The name of the widget element
      "Temp:", // The label to show on the display
      0, // The x position of the label
      0, // The y position of the label
      1, // The font size of the label
      500, // The update rate of the widget element in milliseconds
      0, // The display mode of the widget element (0 for hex)
     {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The color
        8*8, // Total width  (used by ClearArea() to clear underneath , total size estimated)
        1*8, // Total height (used by ClearArea() to clear underneath , total size estimated)
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      }
 
    },
    // Widget element 2: humidity in dec
    {
      "hum", // The name of the widget element
      "Hum:", // The label to show on the display
      64, // The x position of the label
      0, // The y position of the label
      1, // The font size of the label
      1000, // The update rate of the widget element in milliseconds
      1, // The display mode of the widget element (1 for dec)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The color
        8*8, // Total width  (used by ClearArea() to clear underneath , total size estimated)
        1*8, // Total height (used by ClearArea() to clear underneath , total size estimated)
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      } 
    },
    // Widget element 3: light in bar
    {
      "light", // The name of the widget element
      "Light:", // The label to show on the display
      0, // The x position of the label
      16, // The y position of the label
      1, // The font size of the label
      50, // The update rate of the widget element in milliseconds
      0x21, // The display mode of the widget element (0x21 for 2-bar 1-subtype fade)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The bar color
        64, // The bar width
        16, // The bar height
        20, // the bar white rate
        50, // the bar fade rate
        0, 0, 0 // The unused parameters for the display mode
      }
    },
    // Widget element 4: sound in rolling graph
    {
      "sound", // The name of the widget element
      "Sound:", // The label to show on the display
      64, // The x position of the graph
      16, // The y position of the graph
      1, // The font size of the label
      500, // The update rate of the widget element in milliseconds
      3, // The display mode of the widget element (3 for rolling graph)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The graph color
        64, // The graph width
        32, // The graph height
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      }
    },
    // Widget element 5: pressure in analog gauge
    {
      "press", // The name of the widget element
      "Press:", // The label to show on the display
      0, // The x position of the label
      32, // The y position of the label
      1, // The font size of the label
      50, // The update rate of the widget element in milliseconds
      4, // The display mode of the widget element (4 for analog gauge)
      {
        0, // The minimum value of the widget element
        100, // The maximum value of the widget element
        WHITE, // The gauge color
        32, // The gauge width
        32, // The gauge height
        16, // The gauge radius
        0, 0, 0, 0 // The unused parameters for the display mode
      }
    }
  },
  // Layout 2: voltage, current, power, energy, frequency
  {
    // Widget element 1: voltage in dec
    {
      "volt", // The name of the widget element
      "Volt:", // The label to show on the display
      0, // The x position of the label
      0, // The y position of the label
      1, // The font size of the label
      1000, // The update rate of the widget element in milliseconds
      1, // The display mode of the widget element (1 for dec)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The color
        8*8, // Total width  (used by ClearArea() to clear underneath , total size estimated)
        1*8, // Total height (used by ClearArea() to clear underneath , total size estimated)
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      } 
    },
    // Widget element 2: current in dec
    {
      "curr", // The name of the widget element
      "Curr:", // The label to show on the display
      64, // The x position of the label
      0, // The y position of the label
      1, // The font size of the label
      1000, // The update rate of the widget element in milliseconds
      1, // The display mode of the widget element (1 for dec)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The color
        8*8, // Total width  (used by ClearArea() to clear underneath , total size estimated)
        1*8, // Total height (used by ClearArea() to clear underneath , total size estimated)
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      } 
    },
    // Widget element 3: power in bar
    {
      "pow", // The name of the widget element
      "Pow:", // The label to show on the display
      0, // The x position of the label
      16, // The y position of the label
      1, // The font size of the label
      500, // The update rate of the widget element in milliseconds
      2, // The display mode of the widget element (2 for bar)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The bar color
        64, // The bar width
        16, // The bar height
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      }
    },
    // Widget element 4: energy in rolling graph
    {
      "en", // The name of the widget element
      "En:", // The label to show on the display
      64, // The x position of the label
      16, // The y position of the label
      1, // The font size of the label
      100, // The update rate of the widget element in milliseconds
      3, // The display mode of the widget element (3 for rolling graph)
      {
        0, // The minimum value of the widget element
        1023, // The maximum value of the widget element
        WHITE, // The graph color
        64, // The graph width
        32, // The graph height
        0, 0, 0, 0, 0 // The unused parameters for the display mode
      }
    },
// Widget element 5: frequency in analog gauge
    {
      "freq", // The name of the widget element
      "Freq:", // The label to show on the display
      0, // The x position of the label
      32, // The y position of the label
      1, // The font size of the label
      500, // The update rate of the widget element in milliseconds
      4, // The display mode of the widget element (4 for analog gauge)
      {
        0, // The minimum value of the widget element
        100, // The maximum value of the widget element
        WHITE, // The gauge color
        32, // The gauge width
        32, // The gauge height
        16, // The gauge radius
        0, 0, 0, 0 // The unused parameters for the display mode
      }
    }
  }
};

// Define the number of layouts
#define NUM_LAYOUTS (sizeof(layouts) / sizeof(layouts[0]))

// Define the number of widget elements per layout
#define NUM_WIDGETS (sizeof(layouts[0]) / sizeof(layouts[0][0]))

// Define the buffer size for the rolling graph
#define BUFFER_SIZE 64

// Define the buffer for the rolling graph
int buffer[BUFFER_SIZE];

// Define the buffer index for the rolling graph
int bufferIndex = 0;

// Define the last update time for each widget element
unsigned long lastUpdate[NUM_WIDGETS];

// Define the current value for each widget element
int currentValue[NUM_WIDGETS];

// Define the current layout index
int currentLayout = 0;

// Define the last layout change time
unsigned long lastLayoutChange = 0;

// Define the layout change interval
unsigned long layoutChangeInterval = 16000; // 6 seconds

// Initialize the display
void initDisplay() {
  // Initialize with I2C address 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("SSD1306 allocation failed");
    for (;;); // Don't proceed, loop forever
  }

  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the display buffer
  display.clearDisplay();

  // Set the display rotation
//  display.setRotation(0);

  // Set the display contrast
//  display.setContrast(255);
//  display.dim(0); any value above 0 switches off the display in my case

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("test");


  // Display the buffer on the screen
  display.display();
  delay (2000);
}

// Update the display
void updateDisplay() {
  // Check if the layout needs to be changed
  if (millis() - lastLayoutChange >= layoutChangeInterval) {
    // Update the last layout change time
    lastLayoutChange = millis();

    // Increment the current layout index
    currentLayout++;

    // Wrap the current layout index
    if (currentLayout >= NUM_LAYOUTS) {
      currentLayout = 0;
    }

    // Clear the display buffer
    display.clearDisplay();
  }

  // Get the current layout
  WidgetElement* layout = layouts[currentLayout];

  // Loop through the widget elements
  for (int i = 0; i < NUM_WIDGETS; i++) {
    // Get the current widget element
    WidgetElement widget = layout[i];

    // Check if the widget element needs to be updated
    if (millis() - lastUpdate[i] >= widget.rate) {
      // Update the last update time
      lastUpdate[i] = millis();

      // Update the current value
      currentValue[i] = readWidget(widget.name);

      // Clear the area of the widget element
//      clearArea(widget);
    
      // Display the label
//      display.setTextSize(widget.font);
//      display.setTextColor(WHITE);
//      display.setCursor(widget.x, widget.y);
//      display.print(widget.label);

      // Display the value according to the mode
      switch (widget.mode) {
        case 0: // Hex
          clearArea(widget);
          display.setTextSize(widget.font);
          display.setTextColor(WHITE);
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);              
          display.print("0x");
          display.print(currentValue[i], HEX);
          break;
        case 1: // Dec
          clearArea(widget);
          display.setTextSize(widget.font);
          display.setTextColor(WHITE);
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);
          display.print(currentValue[i], DEC);
          break;
          
        case 0x2: // Bar
          clearArea(widget);
//          clearArea_50(widget);

          displayBar(widget, currentValue[i]);
          display.setTextSize(widget.font);
//          display.setTextColor(WHITE);
          display.setTextColor(2); // FIXME: 2 for xor , but this should not be hardcoded as in color display this will fuck things up. 
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);          
          break;

        case 0x21: // Bar with fade
//          clearArea(widget);
          clearAreaRate(widget,widget.params[6]); // set the clear area rate with the rate defined in the widget parameters

//          displayBar(widget, currentValue[i]);
          displayBarRate(widget, currentValue[i]);

          display.setTextSize(widget.font);
//          display.setTextColor(WHITE);
          display.setTextColor(2); // FIXME: 2 for xor , but this should not be hardcoded as in color display this will fuck things up. 
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);          
          break;
          
        case 3: // Rolling graph
          clearArea(widget);
          display.setTextSize(widget.font);
//          display.setTextColor(WHITE);
          display.setTextColor(2); // FIXME: 2 for xor , but this should not be hardcoded as in color display this will fuck things up. 
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);          
          
          displayGraph(widget, currentValue[i]);
          break;
        case 4: // Analog gauge
          clearArea(widget);
          display.setTextSize(widget.font);
//          display.setTextColor(WHITE);
          display.setTextColor(2); // FIXME: 2 for xor , but this should not be hardcoded as in color display this will fuck things up. 
          display.setCursor(widget.x, widget.y);
          display.print(widget.label);          

          displayGauge(widget, currentValue[i]);
          break;
        default: // Invalid mode
          display.print("?");
          break;
      }
      display.display();
    }
  }

  // Display the buffer on the screen
//  display.display();
}

// Read a widget element from a sensor or a pin
int readWidget(String name) {
  // TODO: Replace this with your own code to read the widget element from a sensor or a pin
  // For example, you can use analogRead(), digitalRead(), or a library function
  // For now, we just return a random value
  return random(0, 1024);
}

// Clear the area of a widget element
void clearArea(WidgetElement widget) {
  // Get the parameters
  int min = widget.params[0]; // The minimum value
  int max = widget.params[1]; // The maximum value
  int color = widget.params[2]; // The color
  int width = widget.params[3]; // The width
  int height = widget.params[4]; // The height
  int radius = widget.params[5]; // The radius

  // Calculate the area to clear
  int x1 = widget.x;
  int y1 = widget.y;
  int x2 = widget.x + width;
  int y2 = widget.y + height+1;

  // Clear the area
  display.fillRect(x1, y1, x2 - x1, y2 - y1, BLACK);
}

// Clear approx 50%the area of a widget element
void clearAreaRate(WidgetElement widget, int rate) {
  // Get the parameters
  int min = widget.params[0]; // The minimum value
  int max = widget.params[1]; // The maximum value
  int color = widget.params[2]; // The color
  int width = widget.params[3]; // The width
  int height = widget.params[4]; // The height
  int radius = widget.params[5]; // The radius

  // Calculate the area to clear
  int x1 = widget.x;
  int y1 = widget.y;
  int x2 = widget.x + width;
  int y2 = widget.y + height+1;

  // Clear the area 
//  display.fillRect(x1, y1, x2 - x1, y2 - y1, BLACK);
  fillRectWithRate(x1, y1, x2 - x1, y2 - y1, BLACK,rate);
   
}

void fillRectWithRate(int x, int y, int w, int h, int color, int rate) {
  // calculate the number of pixels to fill
  // Fill the rectangle from left to right, top to bottom
  for (int py=y; py < y+h  ; py++) {
    for (int px=x ; px < x+w ; px++) {
      if ( random(100) > rate) {
        display.drawPixel(px,py,color);
      }
    }
  }
}

// Display a bar
void displayBar(WidgetElement widget, int value) {
  // Get the parameters
  int min = widget.params[0]; // The minimum value
  int max = widget.params[1]; // The maximum value
  int color = widget.params[2]; // The bar color
  int width = widget.params[3]; // The bar width
  int height = widget.params[4]; // The bar height

  // Map the value to the bar range
  int bar = map(value, min, max, 0, width);

  // Draw the bar
  display.fillRect(widget.x, widget.y, bar, height, color);
}

// Display a bar
void displayBarRate(WidgetElement widget, int value) {
  // Get the parameters
  int min     = widget.params[0]; // The minimum value
  int max     = widget.params[1]; // The maximum value
  int color   = widget.params[2]; // The bar color
  int width   = widget.params[3]; // The bar width
  int height  = widget.params[4]; // The bar height
  int rate    = widget.params[5]; // the bar intensity rate

  // Map the value to the bar range
  int bar = map(value, min, max, 0, width);

  // Draw the bar
  fillRectWithRate(widget.x, widget.y, bar, height, color,rate);
}


// Display a rolling graph
void displayGraph(WidgetElement widget, int value) {
  // Get the parameters
  int min = widget.params[0]; // The minimum value
  int max = widget.params[1]; // The maximum value
  int color = widget.params[2]; // The graph color
  int width = widget.params[3]; // The graph width
  int height = widget.params[4]; // The graph height

  // Map the value to the graph range
  int graph = map(value, min, max, 0, height);

  // Shift the buffer to the left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }

  // Add the new value to the buffer
  buffer[BUFFER_SIZE - 1] = graph;

  // Increment the buffer index
  bufferIndex++;

  // Wrap the buffer index
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
  }

  // Draw the graph
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int x = widget.x + i;
    int y = widget.y + height - buffer[i];
    display.drawPixel(x, y, color);
  }
}

// Display an analog gauge
void displayGauge(WidgetElement widget, int value) {
  // Get the parameters
  int min = widget.params[0]; // The minimum value
  int max = widget.params[1]; // The maximum value
  int color = widget.params[2]; // The gauge color
  int width = widget.params[3]; // The gauge width
  int height = widget.params[4]; // The gauge height
  int radius = widget.params[5]; // The gauge radius

  // Map the value to the gauge range
  int gauge = map(value, min, max, -90, 90);

  // Calculate the gauge angle in radians
  float angle = gauge * PI / 180.0;

  // Calculate the gauge end point
  int x = widget.x + width / 2 + radius * cos(angle);
  int y = widget.y + height / 2 + radius * sin(angle);

  // Draw the gauge
  display.drawLine(widget.x + width / 2, widget.y + height / 2, x, y, color);
}

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the display
  initDisplay();

  // Initialize the random seed
  randomSeed(analogRead(0));
}

// Loop function
void loop() {
  // Update the display
  updateDisplay();
}
