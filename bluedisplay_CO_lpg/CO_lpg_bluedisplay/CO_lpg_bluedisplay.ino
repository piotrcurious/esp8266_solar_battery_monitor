
#include <stdlib.h>  // For rand()
#include <stdbool.h> // for bitwise operations
//#define PACKETS_FROM_NETWORK

#include <esp_bt.h>

#ifdef PACKETS_FROM_NETWORK
#include "wifi_settings.h"
#endif //#ifdef PACKETS_FROM_NETWORK

#include <BlueDisplay.hpp>

#define COLOR_FOREGROUND COLOR16_WHITE
#define GRAPH0_COLOR COLOR16_YELLOW
#define GRAPH1_COLOR COLOR16_GREEN

#define COLOR_BACKGROUND COLOR16_BLACK

#define GRAYING_FACTOR 0.9 // factor of making color more gray

//#define Serial0 Serial // for old IDF

// a string buffer for any purpose...
char sStringBuffer[128];

// BlueDisplay object
//BlueDisplay bluedisplay;

#include "telemetry_frame.hpp" // make sure to sync that with sender
                                //hpp to enable #pragma tags


//struct telemetry_frame tframe ; // define a global variable to store telemetry frame
telemetry_frame tframe ; // define a global variable to store telemetry frame

#include "graph_settings.h"

#ifdef PACKETS_FROM_NETWORK
// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);
#endif //#ifdef PACKETS_FROM_NETWORK

//helper function to make a color more pale

color16_t toGrayishColor(color16_t color, float factor) {
    // Ensure the factor is between 0 and 1
    if (factor < 0.0) factor = 0.0;
    if (factor > 1.0) factor = 1.0;

    // Extract the red, green, and blue components from the 16-bit color
    uint8_t red = (color >> 11) & 0x1F;   // 5 bits for red
    uint8_t green = (color >> 5) & 0x3F;  // 6 bits for green
    uint8_t blue = color & 0x1F;          // 5 bits for blue

    // Calculate the average intensity (grayscale value)
    uint8_t gray = (red * 255 / 31 + green * 255 / 63 + blue * 255 / 31) / 3;

    // Convert the grayscale value to the 5-bit or 6-bit range as needed
    uint8_t grayRed = gray * 31 / 255;
    uint8_t grayGreen = gray * 63 / 255;
    uint8_t grayBlue = gray * 31 / 255;

    // Blend the original color with the grayscale value using the factor
    red = red * (1.0 - factor) + grayRed * factor;
    green = green * (1.0 - factor) + grayGreen * factor;
    blue = blue * (1.0 - factor) + grayBlue * factor;

    // Combine the components back into a 16-bit color
    color16_t grayishColor = (red << 11) | (green << 5) | blue;
    return grayishColor;
}

// Function to update the display with dynamic positioning
void DisplayDebug() {
    int displayWidth = BlueDisplay1.getDisplayWidth();
    int displayHeight = BlueDisplay1.getDisplayHeight();
    int x0 = 0;

    uint16_t y0 = 16 ; //  label 
    uint16_t y1 = 16*2; // resolution
    uint16_t y2 = 16*3; // CO
    uint16_t y3 = 16*4; // IP
    uint16_t y4 = 16*5; // total packets

    // Clear the rectangle under the debug window
    BlueDisplay1.fillRect(x0, 0, 12*16, y3, COLOR_BACKGROUND);
    // frame 
    BlueDisplay1.drawRect(x0, 0, 12*16, y3, COLOR_FOREGROUND,1);

    if (graphComplete){
    BlueDisplay1.drawText(x0, y0, "    Voltage", 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    } else {
    BlueDisplay1.drawText(x0, y0, "   (Voltage)", 16, COLOR_FOREGROUND, COLOR_BACKGROUND); // indicate that graph drawing is not finished
    }
    sprintf(sStringBuffer,"Res: x:%u, y:%u",displayWidth,displayHeight);
    BlueDisplay1.drawText(x0, y1, sStringBuffer,16,COLOR_FOREGROUND, COLOR_BACKGROUND);
//    BlueDisplay1.drawText(x0, y1, "SSID: " + String(ssid),16,COLOR_FOREGROUND, COLOR_BACKGROUND);

//    sprintf(sStringBuffer,"Status: %s", status);
//    BlueDisplay1.drawText(x0, y2, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);

//    if(new_packet) {Serial0.println(tframe.CO_sensor);}
    
    sprintf(sStringBuffer,"Voltage: %f", tframe.CO_sensor);
    BlueDisplay1.drawText(x0, y2, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);

//     sprintf_P(sStringBuffer, PSTR("%02d"), ip);
        //sStringBuffer = WiFi.localIP().toString().c_str();

#ifdef PACKETS_FROM_NETWORK
       sprintf(sStringBuffer,"IP:%s", WiFi.softAPIP().toString());
       BlueDisplay1.drawText(x0, y3, sStringBuffer,16,COLOR_FOREGROUND, COLOR_BACKGROUND);

    sprintf(sStringBuffer,"packets: %d", total_packets);
    BlueDisplay1.drawText(x0, y4, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);
#endif //#ifdef PACKETS_FROM_NETWORK


    //bluedisplay.flushDisplay();
}

void setup() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Initialize BlueDisplay
//    bluedisplay.begin("ESP32 BlueDisplay");
#if defined(ESP32) // fixme : should be free to set up on any port, not hardfixed
    Serial0.begin(115200);

esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);
esp_bredr_tx_power_set(ESP_PWR_LVL_P9,ESP_PWR_LVL_P9);

//    ESP_PWR_LVL_N12 = 0,                /*!< Corresponding to -12dbm */
//    ESP_PWR_LVL_N9  = 1,                /*!< Corresponding to  -9dbm */
//    ESP_PWR_LVL_N6  = 2,                /*!< Corresponding to  -6dbm */
//    ESP_PWR_LVL_N3  = 3,                /*!< Corresponding to  -3dbm */
//    ESP_PWR_LVL_N0  = 4,                /*!< Corresponding to   0dbm */
//    ESP_PWR_LVL_P3  = 5,                /*!< Corresponding to  +3dbm */
//    ESP_PWR_LVL_P6  = 6,                /*!< Corresponding to  +6dbm */
//    ESP_PWR_LVL_P9  = 7,                /*!< Corresponding to  +9dbm */

 esp_power_level_t min_power_level;
 esp_power_level_t max_power_level;
 esp_bredr_tx_power_get(&min_power_level, &max_power_level);
 
    Serial0.println("bt_pwr");
    Serial0.println(min_power_level);
    Serial0.println(max_power_level);
 
 //   Serial.println(StartMessage);
      initSerial("CO_sensor");
//    initSerial(ssid);

esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);
esp_bredr_tx_power_set(ESP_PWR_LVL_P9,ESP_PWR_LVL_P9);

// esp_bredr_tx_power_get(&min_power_level, &max_power_level);
// 
//    Serial0.println("bt_pwr");
//    Serial0.println(min_power_level);
//    Serial0.println(max_power_level);

 
    
//    Serial0.println("Start ESP32 BT-client with name \"voltage\"");//  who cares
#else
    initSerial();
#endif

    BlueDisplay1.initCommunication(&initDisplay, &drawGui);
    checkAndHandleEvents(); // this copies the display size and time from remote device

#ifdef PACKETS_FROM_NETWORK
    delay(200);
    // Start the SoftAP
    Serial0.println("Starting SoftAP...");
    WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
//    WiFi.softAP(ssid, password);
    delay (200);
    //esp_wifi_set_ps(WIFI_PS_NONE); // disable wifi power saving so packets do not get deferred
                                    //this breaks bluetooth coexistence

    static wifi_country_t wifi_country = {.cc="SU", .schan=1, .nchan=13, .max_tx_power=84, .policy=WIFI_COUNTRY_POLICY_MANUAL};
    esp_wifi_set_country(&wifi_country) ; /* set country for channel range [1, 13] */
    esp_wifi_set_max_tx_power(84);
//    WiFi.setTxPower(WIFI_POWER_20_5dBm); // 

    delay(200);
    //    Serial0.println(WiFi.getTxPower());
//    wifi_country_t myCountry;
//      if(esp_wifi_get_country(&myCountry) == ESP_OK){
//        Serial0.print("Country Code: ");
//        Serial0.println(myCountry.cc);
//        Serial0.println(myCountry.max_tx_power);
//      }

    
  // Initialize the asyncUDP object
  if (udp.listenMulticast(multicastIP, multicastPort)) {
//    Serial0.println("UDP listening"); // who cares
  delay(500);
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }
#endif //#ifdef PACKETS_FROM_NETWORK    

    minutes_millis_last = millis() ; 
    
#ifdef GRAPH_TEST

   for (uint16_t j = 0 ; j <(NUMBER_OF_BUFFERS) ; j++){
    for (uint16_t i = 0; i < (MINUTES_GRAPH_BUFFER_MAX-2); i++) {
      // initalize array with bogus values
      switch (j){
      case 0:
      minutes_buffer[j][i]=random(1000)*0.001;
      if (minutes_buffer[j][i]>0.8){minutes_buffer[j][i]=NAN;}
      if (minutes_buffer[j][i]<0.5){minutes_buffer[j][i]=NAN;}
      break;
      case 1:
      minutes_buffer[j][i]=random(2000)*0.001;
      if (minutes_buffer[j][i]>1.5){minutes_buffer[j][i]=NAN;}
      if (minutes_buffer[j][i]<1.0){minutes_buffer[j][i]=NAN;}
      break;      
      }
    }
    minutes_buffer_min[j]= 0.0;
    minutes_buffer_max[j]= 2.0; 
   }
#endif // GRAPH_TEST
}

#ifdef PACKETS_FROM_NETWORK    
// Function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet) {
  // Copy the UDP packet data to the pulse data variable
  memcpy((byte*)&tframe, packet.data(), sizeof(tframe));
  new_packet = true; 
  total_packets++; 
}
#endif //#ifdef PACKETS_FROM_NETWORK    

void update_minute_buffer () {

  for (uint16_t j = 0 ; j < (NUMBER_OF_BUFFERS); j++) {
    minutes_buffer_min[j] = minutes_buffer_max[j]; // set graphMin to last graphMax value
    minutes_buffer_max[j] = 0;
    for (uint16_t i = 0; i < (MINUTES_GRAPH_BUFFER_MAX-1); i++) {
      minutes_buffer[j][i] = minutes_buffer[j][i + 1];
//      Serial0.println(i+1);
      if (minutes_buffer[j][i] > minutes_buffer_max[j]) {
        minutes_buffer_max[j] = minutes_buffer[j][i];
        }
      if (minutes_buffer[j][i] < minutes_buffer_min[j]) {
        minutes_buffer_min[j] = minutes_buffer[j][i];
      }
      // graphMax_oled = max(graphMax_oled, rollingBuffer_oled[i]); 
      // or use that instead 
    }
    // Add the pulse length to the rolling buffer

//Serial0.println(j);

#ifdef PACKETS_FROM_NETWORK  
      if(new_packet) {
        switch (j) {
          case 0:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.CO_sensor;
          break;
          case 1:
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.LPG_sensor;        
          break;
        }
//        new_packet = false; 
      } else {
//        minutes_buffer[MINUTES_GRAPH_BUFFER_MAX-1] = NAN ;      
        switch (j) {
          case 0:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = NAN;
          break;
          case 1:
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = NAN;        
          break;
        }
      }
#else  //#ifdef PACKETS_FROM_NETWORK  

//        minutes_buffer[MINUTES_GRAPH_BUFFER_MAX-1] = tframe.CO_sensor; // local
        switch (j) {
          case 0:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.CO_sensor;
          break;
          case 1:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.LPG_sensor;        
//          Serial0.println(MINUTES_GRAPH_BUFFER_MAX-1);
          break;
        }
#endif //#ifdef PACKETS_FROM_NETWORK      
  }

#ifdef PACKETS_FROM_NETWORK      
      if(new_packet) {new_packet = false;}  
#endif //#ifdef PACKETS_FROM_NETWORK      

}

// Function to plot the graph 
void plotGraph(float *data, uint16_t dataSize,uint16_t graphPosX,uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max) {
  uint16_t xStart = graphPosX;
  uint16_t yStart = graphPosY;

  // Calculate the number of data points that can fit within the graph width
  int pointsToPlot = graphWidth;
  int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

  float xScale = (float)graphWidth / (float)(pointsToPlot - 1);
  float yScale = (float)graphHeight / (graph_max - graph_min);

  int lastX = -1;
  int lastY = -1;
  bool lastValid = false;

  color16_t whiteColor = COLOR16_BLACK;
  color16_t grayColor =  COLOR16_BLUE;

  for (int i = startIndex; i < dataSize; i++) {
    if (!isnan(data[i])) {  // Check if the current data point is valid
      int x = xStart + (int)((i - startIndex) * xScale);
      int y = yStart + graphHeight - (int)((data[i] - graph_min) * yScale);

      if (lastValid) {
        BlueDisplay1.drawLine(lastX, lastY, x, y, whiteColor);  // Draw a line between the last valid point and the current point using white color
      }

      lastX = x;
      lastY = y;
      lastValid = true;
    } else {
      if (lastValid) {
        // Draw a gray line to mark missing data if the last point was valid
        int j = i + 1;
        while (j < dataSize && isnan(data[j])) {
          j++;
        }

        if (j < dataSize) {
          int nextValidX = xStart + (int)((j - startIndex) * xScale);
          int nextValidY = yStart + graphHeight - (int)((data[j] - graph_min) * yScale);
          BlueDisplay1.drawLine(lastX, lastY, nextValidX, nextValidY, grayColor);  // Use gray color for missing data indicator
        }
      }
      lastValid = false;
    }
  }
    sprintf(sStringBuffer,"%f", graph_min);    
    BlueDisplay1.drawText(xStart+graphWidth-16*8, yStart+graphHeight-8, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(xStart, yStart+graphHeight-1, graphWidth, yStart+graphHeight-1, COLOR16_RED);  //boundary

    sprintf(sStringBuffer,"%f", graph_max);    
    BlueDisplay1.drawText(xStart+graphWidth-16*8, yStart, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(xStart, yStart, graphWidth, yStart, COLOR16_RED);  //boundary

    sprintf(sStringBuffer,"%f", data[dataSize-1]);    // get last data point
    int nextValidY = yStart + graphHeight - (int)((data[dataSize-1] - graph_min) * yScale);
    BlueDisplay1.drawLine(xStart, nextValidY, graphWidth, nextValidY, COLOR16_LIGHT_GREY);  //gray line with current data
    BlueDisplay1.drawText(xStart+graphWidth-16*8, nextValidY, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);

}

// Function to add a line to the buffer
void bufferLine(int x1, int y1, int x2, int y2, color16_t color, uint8_t buffer_index) {
    if (lineBufferIndex < MAX_LINES) {
        lineBuffer[buffer_index][lineBufferIndex].x1 = x1;
        lineBuffer[buffer_index][lineBufferIndex].y1 = y1;
        lineBuffer[buffer_index][lineBufferIndex].x2 = x2;
        lineBuffer[buffer_index][lineBufferIndex].y2 = y2;
        lineBuffer[buffer_index][lineBufferIndex].color = color;
        lineBufferIndex++;
    } else {
        // Handle buffer overflow (e.g., log an error or extend buffer size)
    }
}

// Function to draw a line from the buffer and mark it as drawn - not clearing underneath
void drawBufferedLineNoClr(uint16_t index, uint8_t buffer_index) {
    if (index < lineBufferIndex && lineBuffer[buffer_index][index].color != DRAWN_MAGIC_NUMBER) {
        BlueDisplay1.drawLine(lineBuffer[buffer_index][index].x1, lineBuffer[buffer_index][index].y1, 
                              lineBuffer[buffer_index][index].x2, lineBuffer[buffer_index][index].y2, 
                              lineBuffer[buffer_index][index].color);
        lineBuffer[buffer_index][index].color = DRAWN_MAGIC_NUMBER;  // Mark the line as drawn
    }
}

// Function to draw a line from the buffer and mark it as drawn - clearing rect under the line (height is assumed to be graph height)
void drawBufferedLineClr(uint16_t index,uint8_t buffer_index) {
    if (index < lineBufferIndex && lineBuffer[buffer_index][index].color != DRAWN_MAGIC_NUMBER) {
        uint16_t x1 = lineBuffer[buffer_index][index].x1;
        uint16_t y1 = lineBuffer[buffer_index][index].y1;
        uint16_t x2 = lineBuffer[buffer_index][index].x2;
        uint16_t y2 = lineBuffer[buffer_index][index].y2;
        
        // Determine the minimum and maximum x values for the rectangle
//        uint16_t xMin = x1 < x2 ? x1 : x2;
//        uint16_t xMax = x1 > x2 ? x1 : x2;
        
        // Define the rectangle width and use the global graph height for height
//        uint16_t rectWidth = xMax - xMin + 1;
        uint16_t rectHeight = globalGraphYPos+globalGraphHeight;  // Use the global graph height

        // Clear the rectangle before drawing the line
//        BlueDisplay1.fillRect(xMin, globalGraphYPos, xMax, rectHeight, COLOR_BACKGROUND);
        BlueDisplay1.fillRect(x1, globalGraphYPos, x2, rectHeight, COLOR_BACKGROUND);

        // Now draw the line
        BlueDisplay1.drawLine(x1, y1, x2, y2, lineBuffer[buffer_index][index].color);
        
        // Mark the line as drawn
        lineBuffer[buffer_index][index].color = DRAWN_MAGIC_NUMBER;
    }
}

// Modified plotGraph function with buffer reset and storing global graph height
// note that there is only one buffer so if you wish to draw two plots at the same time you need to 
// invent new way to create multiple buffers. 
// for now this is simply not implemented and if you want to plot two graphs either use unbuffered plotgraph to plot another
// implement semaphores to make sure new buffer is created once old graph is fully drawn
// or implement more buffers and passing buffer indexes to all the functions. 

/*
void plotGraph_buffered(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor,uint8_t buffer_index) {
    // Reset the line buffer index
    lineBufferIndex = 0;    
    // Reset the current line index for drawing the buffer
    currentLineIndex = 0; 
    graphComplete = false;
   // Reset the graph buffer completeness marker
    lfsr = 1;  // Reset the LFSR to the initial seed value
    linesDrawn = 0;  // Reset the lines drawn counter
      
    // Store the graph height in the global variable
    globalGraphHeight = graphHeight;
    // Store the graph Y position in the global variable 
    globalGraphYPos = graphPosY; 

    uint16_t xStart = graphPosX;
    uint16_t yStart = graphPosY;

    int pointsToPlot = graphWidth-(LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS);
//    int pointsToPlot = graphWidth;

    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (((float)graphWidth-(LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS)))/ (float)(pointsToPlot - 1);
//    float xScale = (((float)graphWidth))/ (float)(pointsToPlot - 1);

    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR) ; // buffer the grayish color for faster plotting

    //color16_t whiteColor = COLOR16_BLACK;
    //color16_t grayColor = COLOR16_BLUE;

    for (uint16_t i = startIndex; i < dataSize; i++) {
        if (!isnan(data[i])) {  // Check if the current data point is valid
            uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                bufferLine(lastX, lastY, x, y, graphColor,buffer_index);  // Buffer the line instead of drawing it immediately
            }

            lastX = x;
            lastY = y;
            lastValid = true;
        } else {
            if (lastValid) {
                uint16_t j = i + 1;
                while (j < dataSize && isnan(data[j])) {
                    j++;
                }

                if (j < dataSize) {
                    uint16_t nextValidX = xStart + (uint16_t)((j - startIndex) * xScale);
                    uint16_t nextValidY = yStart + graphHeight - (uint16_t)((data[j] - graph_min) * yScale);
                    bufferLine(lastX, lastY, nextValidX, nextValidY, grayishColor,buffer_index);  // Buffer the line for missing data
                } else {
              lastValid = false;}
            }
            //lastValid = false; // it causes line buffers mismatch in size so then they cannot be drawn in parallel. 
        }
    }
 }

*/


/*
void plotGraph_buffered(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
                        uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor,uint8_t buffer_index) {
    // Reset the line buffer index
    lineBufferIndex = 0;
    // Reset the current line index for drawing the buffer
    currentLineIndex = 0;
    graphComplete = false;
    // Reset the graph buffer completeness marker
    lfsr = 1;  // Reset the LFSR to the initial seed value
    linesDrawn = 0;  // Reset the lines drawn counter

    // Store the graph height in the global variable
    globalGraphHeight = graphHeight;
    // Store the graph Y position in the global variable
    globalGraphYPos = graphPosY;

    uint16_t xStart = graphPosX;
    uint16_t yStart = graphPosY;

    int pointsToPlot = graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS);
    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (((float)graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS))) / (float)(pointsToPlot - 1);
    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR); // Buffer the grayish color for faster plotting

    for (uint16_t i = startIndex; i < dataSize; i++) {
        if (!isnan(data[i])) {  // Check if the current data point is valid
            uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                bufferLine(lastX, lastY, x, y, graphColor,buffer_index);  // Buffer the line instead of drawing it immediately
            }

            lastX = x;
            lastY = y;
            lastValid = true;
        } else {
            // Handle multiple missing values by finding the next valid point
            uint16_t j = i + 1;
            while (j < dataSize && isnan(data[j])) {
                j++;
            }

            if (lastValid && j < dataSize) {
                uint16_t nextValidX = xStart + (uint16_t)((j - startIndex) * xScale);
                uint16_t nextValidY = yStart + graphHeight - (uint16_t)((data[j] - graph_min) * yScale);

                // Calculate the number of missing points and the increments for interpolation
                uint16_t numMissing = j - i;
                float deltaX = (float)(nextValidX - lastX) / (numMissing + 1);
                float deltaY = (float)(nextValidY - lastY) / (numMissing + 1);

                // Interpolate across all missing points
                for (uint16_t k = 1; k <= numMissing; k++) {
                    //uint16_t interpolatedX = lastX + (uint16_t)(deltaX * k);
                    //uint16_t interpolatedY = lastY + (uint16_t)(deltaY * k);
                    float factor = (float)k/(numMissing+1);
                    uint16_t interpolatedX = lastX + factor*(nextValidX - lastX);
                    uint16_t interpolatedY = lastY + factor*(nextValidY - lastY);
                    
                    bufferLine(lastX, lastY, interpolatedX, interpolatedY, grayishColor,buffer_index);  // Buffer the interpolated line
                    lastX = interpolatedX;
                    lastY = interpolatedY;
                }

                // Draw the final line to the next valid point
                bufferLine(lastX, lastY, nextValidX, nextValidY, grayishColor,buffer_index);
                lastX = nextValidX;
                lastY = nextValidY;
//                lastValid = true;
            } else {
                // If no valid point is found after missing values, mark as invalid
                lastValid = false;
            }

            // Skip to the last handled index
            i = j - 1;
        }
    }
}

*/

void plotGraph_buffered(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor,uint8_t buffer_index) {
    // Reset the line buffer index
    lineBufferIndex = 0;
    // Reset the current line index for drawing the buffer
    currentLineIndex = 0;
    graphComplete = false;
    // Reset the graph buffer completeness marker
    lfsr = 1;  // Reset the LFSR to the initial seed value
    linesDrawn = 0;  // Reset the lines drawn counter

    // Store the graph height in the global variable
    globalGraphHeight = graphHeight;
    // Store the graph Y position in the global variable
    globalGraphYPos = graphPosY;

    uint16_t xStart = graphPosX;
    uint16_t yStart = graphPosY;

    int pointsToPlot = graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS);
    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (((float)graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS))) / (float)(pointsToPlot - 1);
    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = 0;
    int lastY = 0;
    bool lastValid = true;

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR);  // Buffer the grayish color for faster plotting

    for (uint16_t i = startIndex; i < dataSize; i++) {
        // Calculate current X position for this data point
        uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);

        if (!isnan(data[i])) {  // If the data point is valid
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                // Draw a line from the last valid point to the current valid point
                bufferLine(lastX, lastY, x, y, graphColor,buffer_index);
            }

            // Update the last valid point coordinates
            lastX = x;
            lastY = y;
            lastValid = true;
        } else {  // If the data point is missing
            if (lastValid) {
                // Find the next valid point for interpolation
                uint16_t j = i + 1;
                while (j < dataSize && isnan(data[j])) {
                    j++;
                }

                if (j < dataSize) {  // If a next valid point is found
                    // Calculate the coordinates of the next valid point
                    uint16_t nextValidX = xStart + (uint16_t)((j - startIndex) * xScale);
                    uint16_t nextValidY = yStart + graphHeight - (uint16_t)((data[j] - graph_min) * yScale);

                    // Linear interpolation for the current missing point
                    float factor = (float)(i - startIndex + 1) / (float)(j - startIndex + 1);
                    uint16_t interpY = lastY + factor * (nextValidY - lastY);

                    // Draw the interpolated line segment for the missing point
                    bufferLine(lastX, lastY, x, interpY, grayishColor,buffer_index);

                    // Update the last point to the interpolated point
                    lastX = x;
                    lastY = interpY;
                } else {
                    lastValid = false;  // No further valid points found, stop interpolating
                }
            } else { // if(lastValid) if a data point is missing, and there was no last valid point
                    bufferLine(lastX,lastY,x,0,grayishColor,buffer_index); // draw a 0 because we need something drawn in for this entry
                                                                    // such situation occurs at the first point of the graph
                    lastX = x;
                    lastY = 0; 
                    lastValid = true; // set to 0 
                  }
            
        } // else 
    }
}

void drawLabels (float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor){

    float yScale = (float)graphHeight / (graph_max - graph_min);

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR) ; // buffer the grayish color for faster plotting

    // Draw the axis boundaries and graph labels
    sprintf(sStringBuffer, "%.2f", graph_min);    
    BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS), graphPosY + graphHeight - 8, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
//    bufferLine(xStart, yStart + graphHeight - 1, xStart + graphWidth, yStart + graphHeight - 1, COLOR16_RED);
    BlueDisplay1.drawLine(graphPosX, graphPosY + graphHeight - 1, graphPosX + graphWidth, graphPosY + graphHeight - 1, COLOR16_RED);

    sprintf(sStringBuffer, "%.2f", graph_max);    
    BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS), graphPosY+LEGEND_LABEL_FONT_SIZE, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
//    bufferLine(xStart, yStart, xStart + graphWidth, yStart, COLOR16_RED);
    BlueDisplay1.drawLine(graphPosX, graphPosY, graphPosX + graphWidth, graphPosY, COLOR16_RED);

    // last value 

    uint16_t lastDataY = graphPosY + graphHeight - (uint16_t)((data[dataSize - 2] - graph_min) * yScale);
    BlueDisplay1.drawLine(graphPosX, lastDataY, graphPosX + graphWidth, lastDataY, COLOR_BACKGROUND);  // clear last data line

    sprintf(sStringBuffer, "%.2f", data[dataSize - 1]);    // Get the last data point
    lastDataY = graphPosY + graphHeight - (uint16_t)((data[dataSize - 1] - graph_min) * yScale);
    BlueDisplay1.drawText(graphPosX + graphWidth - (LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS), lastDataY, sStringBuffer, 16, graphColor, COLOR_BACKGROUND);
//    bufferLine(xStart, lastDataY, xStart + graphWidth, lastDataY, COLOR16_LIGHT_GREY);
    BlueDisplay1.drawLine(graphPosX, lastDataY, graphPosX + graphWidth, lastDataY, grayishColor);
}


               

// Function to draw all lines from the buffer
void drawAllBufferedLines() {
    for (int i = 0; i < lineBufferIndex; i++) {
            drawBufferedLineClr(i,0); // background graph, clearing underneath
            drawBufferedLineNoClr(i,1); // next graph, overlaying over

    }
    graphComplete = true;
}

// Function to draw a random subset of lines from the buffer
void drawRandomLines(uint16_t numLinesToDraw) {
    if (numLinesToDraw > lineBufferIndex) {
        numLinesToDraw = lineBufferIndex;  // Limit to available lines
    }

    for (uint16_t i = 0; i < numLinesToDraw; i++) {
      uint16_t randomIndex ; 
        for (uint16_t in = 0 ; in < lineBufferIndex; in++){ // retry as many times as lines in buffer 
          randomIndex = rand() % lineBufferIndex;  // Get a random line index
          if (lineBuffer[0][randomIndex].color != DRAWN_MAGIC_NUMBER) {
            break;    // break if found
          }
        }
            drawBufferedLineClr(randomIndex,0); // background graph, clearing underneath
            drawBufferedLineNoClr(randomIndex,1); // next graph, overlaying over
    }
}

// Function to draw a random subset of lines from the buffer not iterating to find number which was not drawn before
void drawRandomLinesFast(uint16_t numLinesToDraw) {
    if (numLinesToDraw > lineBufferIndex) {
        numLinesToDraw = lineBufferIndex;  // Limit to available lines
    }

    for (uint16_t i = 0; i < numLinesToDraw; i++) {
      uint16_t randomIndex ; 
          randomIndex = rand() % lineBufferIndex;  // Get a random line index
//          if (lineBuffer[randomIndex].color != DRAWN_MAGIC_NUMBER) {
//            break;    // break if found
//          }       
            drawBufferedLineClr(randomIndex,0); // background graph, clearing underneath
            drawBufferedLineNoClr(randomIndex,1); // next graph, overlaying over
    }
}

// Function to draw a specified number of lines sequentially from the buffer
void drawSequentialLines(int numLinesToDraw) {
    // Draw lines sequentially from the current position
    for (int i = 0; i < numLinesToDraw; i++) {
        if (currentLineIndex < lineBufferIndex) {
            drawBufferedLineClr(currentLineIndex,0); // background graph, clearing underneath
            drawBufferedLineNoClr(currentLineIndex,1); // next graph, overlaying over
            
            currentLineIndex++;
        } else {
            // If we reach the end of the buffer, stop drawing
            graphComplete = true;
            break;
        }
    }
}

// Function to implement 16bit Galois LFSR
uint16_t galoisLFSR(bool *wrapped) {
    static uint16_t initialSeed = 1;  // Track the initial seed to detect wraparound
    uint16_t lsb = lfsr & 1;  // Get the least significant bit
    lfsr >>= 1;               // Shift the LFSR right by 1
    if (lsb) {
        // Apply the polynomial (0xB400 corresponds to the polynomial x^12 + x^11 + x^10 + x^4 + 1)
//        lfsr ^= 0xB400 ;  // Use the full 16bit polynomial
//        lfsr ^= 0xD008 ;  // Use the 12bit polynomial

        lfsr ^= LFSR_POLYNOMIAL; // use polynomial defined in graph settings , matching the maximum possible width
    }
    // Check if the LFSR has wrapped around to the initial seed value
    if (lfsr == initialSeed) {
        *wrapped = true;
    }
    return lfsr ;
}

// Function to draw a specified number of lines based on the LFSR sequence
void drawLinesUsingLFSR(int numLinesToDraw) {
    uint16_t linesDrawn = 0;
    bool wrapped = false;

    while (linesDrawn < numLinesToDraw) {
        uint16_t lineIndex = galoisLFSR(&wrapped); 
        // Ensure the generated index is within the range of valid buffered lines
        if (lineIndex < lineBufferIndex) {
            drawBufferedLineClr(lineIndex-1,0);  // the range starts from and is never 0 1 so shift by 1.
                          // drawBufferedLineClr(index , buffer_index); 
            drawBufferedLineNoClr(lineIndex-1,1); // the 2nd graph overlays the first graph
            linesDrawn++;
        }
        // Break if the LFSR has wrapped around to avoid infinite looping
        if (wrapped) {
        graphComplete = true;
            break;
        }
        // break if more lines drawn than MAX_LINES possible
        if (linesDrawn>MAX_LINES) {
          graphComplete = true; 
          break; 
        }
    }    
}

void loop() {
//  struct telemetry_frame tframe ;
 //   BlueDisplay1.clearDisplay();
     if (debug_millis_last<millis()) {      
      DisplayDebug();
      //Serial0.println(minutes_dataArraySize);
      debug_millis_last=millis()+DEBUG_INTERVAL;
     } 

   if (sensor_update_millis_last<millis()) {
      tframe.CO_sensor = 0.001* analogReadMilliVolts(34);
      tframe.LPG_sensor = 0.001* analogReadMilliVolts(35);
      sensor_update_millis_last=millis()+SENSOR_UPDATE_INTERVAL;  
      new_packet = true; // fixme - quick hax
   }

    if (graph_millis_last<millis()) {      
      graph_millis_last=millis()+GRAPH_INTERVAL;
      if (graphComplete){
      plotGraph_buffered(minutes_buffer[0], minutes_dataArraySize-1, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[0], minutes_buffer_max[0],GRAPH0_COLOR,0);
      plotGraph_buffered(minutes_buffer[1], minutes_dataArraySize-1, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[1], minutes_buffer_max[1],GRAPH1_COLOR,1);
      }
     } 

     if (labels_millis_last<millis()) {      
      labels_millis_last=millis()+LABELS_INTERVAL;
      if (graphComplete){
      drawLabels(minutes_buffer[0], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[0], minutes_buffer_max[0],GRAPH0_COLOR);     
      }
     } 


    if (minutes_millis_last<millis()) {
      minutes_millis_last=millis()+MINUTES_INTERVAL;
      update_minute_buffer();      
//    uint16_t displayWidth = BlueDisplay1.getDisplayWidth();
//    uint16_t displayHeight = BlueDisplay1.getDisplayHeight();
//    plotGraph(minutes_buffer, minutes_dataArraySize, GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, minutes_buffer_min, minutes_buffer_max);
//    plotGraph_buffered(minutes_buffer[0], minutes_dataArraySize-1, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[0], minutes_buffer_max[0],GRAPH0_COLOR,0);
//    plotGraph_buffered(minutes_buffer[1], minutes_dataArraySize-1, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[1], minutes_buffer_max[1],GRAPH1_COLOR,1);

    }


//    drawSequentialLines(int numLinesToDraw)
  //  if (!graphComplete) { drawSequentialLines(16);} // draws from left to right , sequential 
    //if (!graphComplete) {drawRandomLinesFast(6); } // draws random lines , not caring if drawn random segment had been drawn before (fast to complete)
//    if (!graphComplete) {drawRandomLines(8); }     // draw random lines, ensuring random number drawn hit the area not drawn before (high cpu but low bandwidth)

    if (!graphComplete) {drawLinesUsingLFSR(CHUNKS_PER_DRAW); }  // draw basing on LFSR generated looping sequence. 
                                            // if the LFSR is not optimized for display width it will take a bit longer 
                                            // by default 16bit LFSR iterating 65536 times is used, 
                                            // if the iteration hits outside width of the graph, next number is drawn, increasing cpu time slightly
                                            // generating next lfsr is ultra fast so this should not matter in real life application
                                             

    checkAndHandleEvents();

#ifdef GRAPH_TEST    
 //   delay(100); // Update delay (for testing purposes, remove it )
//    delay(5000); // Update every 5 seconds
//   delay(10000); // Update every 10 seconds
#endif //#ifdef GRAPH_TEST

}

void initDisplay(void) {
    // Initialize display size and flags
    displayWidth = BlueDisplay1.getMaxDisplayWidth();
    displayHeight = BlueDisplay1.getMaxDisplayHeight();    
    //BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, DISPLAY_WIDTH,
//    DISPLAY_HEIGHT);
#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, displayWidth,displayHeight);
#else //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE , displayWidth,displayHeight);
#endif //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 

    // Initialize button position, size, colors etc.
//    TouchButtonBlinkStartStop.init((DISPLAY_WIDTH - BUTTON_WIDTH_2) / 2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_2,
//    BUTTON_HEIGHT_4, COLOR16_BLUE, "Start", 44, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, doBlink,
//            &doBlinkStartStop);
//    TouchButtonBlinkStartStop.setCaptionForValueTrue("Stop");

//    BlueDisplay1.debug(StartMessage);
}

/*
 * Function is called for resize + connect too
 */
void drawGui(void) {
    displayWidth = BlueDisplay1.getMaxDisplayWidth();
    displayHeight = BlueDisplay1.getMaxDisplayHeight();    
    //BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, DISPLAY_WIDTH,
//    DISPLAY_HEIGHT);
#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, displayWidth,displayHeight);
#else //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE , displayWidth,displayHeight);
#endif //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND);
    plotGraph_buffered(minutes_buffer[0], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X, displayHeight-(GRAPH_Y), minutes_buffer_min[0], minutes_buffer_max[0],GRAPH0_COLOR,0);
    plotGraph_buffered(minutes_buffer[1], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[1], minutes_buffer_max[1],GRAPH1_COLOR,1);


//    TouchButtonBlinkStartStop.drawButton();
}
