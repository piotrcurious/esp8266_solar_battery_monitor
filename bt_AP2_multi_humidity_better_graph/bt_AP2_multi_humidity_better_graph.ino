#include <stdlib.h>  // For rand()
#include <stdbool.h> // for bitwise operations
#include "wifi_settings.h"
#include <BlueDisplay.hpp>
#include "graph_settings.h"
#include <esp_bt.h>

//#define Serial0 Serial // for old IDF

// a string buffer for any purpose...
char sStringBuffer[32];

//helper for order of graphs array 
// Function prototypes
void initializeArray();
void addNumber(int number);
void removeNumber(int number);
void printArray();
bool numberExists(int number);

// Function to initialize the array with -1
void initializeArray() {
  for (int i = 0; i < NUMBER_OF_GRAPHS; i++) {
    graph_order[i] = -1;
  }
}

// Function to add a number to the array at the last available position if it doesn't already exist
void addNumber(int number) {
  if (numberExists(number)) {
    Serial0.println("Number already exists in the array. No action taken.");
    return;
  }

  if (graph_orderSize < NUMBER_OF_GRAPHS) {
    graph_order[graph_orderSize] = number;
    graph_orderSize++;
  } else {
    Serial0.println("Array is full. Cannot add more numbers.");
  }
}

// Function to remove a number from the array and shift elements if necessary
void removeNumber(int number) {
  int index = -1;

  // Find the index of the number to be removed
  for (int i = 0; i < graph_orderSize; i++) {
    if (graph_order[i] == number) {
      index = i;
      break;
    }
  }

  // If the number was found, remove it and shift the elements
  if (index != -1) {
    for (int i = index; i < graph_orderSize - 1; i++) {
      graph_order[i] = graph_order[i + 1];
    }
    graph_order[graph_orderSize - 1] = -1; // Mark the last position as empty
    graph_orderSize--;
  } else {
    Serial0.println("Number not found in the array.");
  }
}

// Function to check if a number exists in the array
bool numberExists(int number) {
  for (int i = 0; i < graph_orderSize; i++) {
    if (graph_order[i] == number) {
      return true;
    }
  }
  return false;
}

// Function to print the current state of the array
void printArray(uint16_t x0 = 12*32 , uint16_t y0 = 16 ) {
//  Serial0.print("Array: ");
  for (int i = 0; i < NUMBER_OF_GRAPHS; i++) {
//    Serial0.print(graph_order[i]);
//    Serial0.print(" ");
    if (graph_order[i] > -1) {
      sprintf(sStringBuffer,"%d", graph_order[i]);
      BlueDisplay1.drawText(x0+i*LEGEND_LABEL_FONT_WIDTH, y0, sStringBuffer,LEGEND_LABEL_FONT_SIZE, COLOR_BACKGROUND, GRAPH_COLOR[graph_order[i]]);
          } else {
      BlueDisplay1.drawText(x0+i*LEGEND_LABEL_FONT_WIDTH, y0,"-",LEGEND_LABEL_FONT_SIZE, COLOR_BACKGROUND, COLOR16_GREY);      
          }
    }
  Serial0.println();
}

// BlueDisplay object
//BlueDisplay bluedisplay; // this is hardcoded for some weird reason

#define PACKETS_FROM_NETWORK

#include "telemetry_frame.hpp" // make sure to sync that with sender
                                //hpp to enable #pragma tags

telemetry_frame tframe ; // define a global variable to store telemetry frame

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

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
    uint16_t y2 = 16*3; // voltage
    uint16_t y3 = 16*4; // IP
    uint16_t y4 = 16*5; // total packets

    // Clear the rectangle under the debug window
    BlueDisplay1.fillRect(x0, 0, LEGEND_LABEL_FONT_WIDTH*16, y3, COLOR_BACKGROUND);
    // frame 
    BlueDisplay1.drawRect(x0, 0, LEGEND_LABEL_FONT_WIDTH*16, y3, COLOR_FOREGROUND,1);

    if (graphComplete){
        if (current_graph >=0 ) {
          BlueDisplay1.drawText(x0, y0, graph_labels[current_graph], LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);
          } else { 
            printArray(LEGEND_LABEL_FONT_WIDTH*8,LEGEND_LABEL_FONT_SIZE); 
            }
    } else {
        if (current_graph >=0 ) {
          BlueDisplay1.drawText(x0, y0, graph_labels[current_graph], LEGEND_LABEL_FONT_SIZE, COLOR_BACKGROUND, COLOR_FOREGROUND);
          } // indicate that graph drawing is not finished
    }
    
    sprintf(sStringBuffer,"Res: x:%u, y:%u",displayWidth,displayHeight);
    BlueDisplay1.drawText(x0, y1, sStringBuffer,LEGEND_LABEL_FONT_SIZE,COLOR_FOREGROUND, COLOR_BACKGROUND);
//    BlueDisplay1.drawText(x0, y1, "SSID: " + String(ssid),16,COLOR_FOREGROUND, COLOR_BACKGROUND);

//    sprintf(sStringBuffer,"Status: %s", status);
//    BlueDisplay1.drawText(x0, y2, sStringBuffer,16, COLOR_FOREGROUND, COLOR_BACKGROUND);

//    if(new_packet) {Serial0.println(tframe.voltage_ADC0);} // for debug
    
    sprintf(sStringBuffer,"Voltage: %f", tframe.voltage_ADC0);
    BlueDisplay1.drawText(x0, y2, sStringBuffer,LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);
//Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");

//  BlueDisplay1.drawText(x0, y2, String(esp_get_free_heap_size()),LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);

       sprintf(sStringBuffer,"IP:%s", WiFi.softAPIP().toString());
       BlueDisplay1.drawText(x0, y3, sStringBuffer,LEGEND_LABEL_FONT_SIZE,COLOR_FOREGROUND, COLOR_BACKGROUND);

    sprintf(sStringBuffer,"packets: %d", total_packets);
    BlueDisplay1.drawText(x0, y4, sStringBuffer,LEGEND_LABEL_FONT_SIZE, COLOR_FOREGROUND, COLOR_BACKGROUND);

}

//void handleSwipe(struct Swipe*);
void handleSwipe(struct Swipe *const swipe_param) ;

void disableUnnecessaryBtFeatures() {
  // Disable scanning
  esp_bt_gap_cancel_discovery();
  
  // Disable device discovery
  //esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  
  // Disable sniff mode
  esp_bt_sleep_disable();
  
  // Disable Secure Simple Pairing
 // esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &esp_bt_io_cap_none, sizeof(uint8_t));
}

void configureCoexistence() {
  // Set WiFi as the preferred mode in coexistence
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  
  // Optionally, adjust other coexistence parameters
  // esp_coex_set_bt_tx_power(ESP_COEX_BT_TX_PWR_LOW);
  // esp_coex_set_wifi_tx_power(ESP_COEX_WIFI_TX_PWR_HIGH);
}


bool btStartMinimal() {
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  
  // Minimize memory usage
  bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
  bt_cfg.bt_max_acl_conn = 1;
  bt_cfg.bt_max_sync_conn = 0;
  
  if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
    return false;
  }
  
  if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
    return false;
  }
  
  if (esp_bluedroid_init() != ESP_OK) {
    return false;
  }
  
  if (esp_bluedroid_enable() != ESP_OK) {
    return false;
  }

  return true;
}

void setup() {
    
    // Initialize BlueDisplay
//    bluedisplay.begin("ESP32 BlueDisplay");
#if defined(ESP32)
    Serial0.begin(115200); // disabled to save memory. 
 //   Serial.println(StartMessage);
    btStartMinimal();
    initSerial("voltage2");
//    Serial0.println("Start ESP32 BT-client with name \"voltage\""); // who cares
#else
    initSerial();
#endif
//    WiFi.setTxPower(WIFI_POWER_20dBm); // default
    WiFi.setTxPower(WIFI_POWER_20_5dBm); // 

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);
    esp_bredr_tx_power_set(ESP_PWR_LVL_P9,ESP_PWR_LVL_P9);
    
    BlueDisplay1.initCommunication(&initDisplay, &drawGui);
    checkAndHandleEvents(); // this copies the display size and time from remote device
    registerSwipeEndCallback(&handleSwipe);

    delay(200);
    // Start the SoftAP
 //   Serial0.println("Starting SoftAP..."); // who cares
    WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);

    delay (200);
    //esp_wifi_set_ps(WIFI_PS_NONE); // disable wifi power saving so packets do not get deferred
                                    //this breaks bluetooth coexistence

    static const wifi_country_t wifi_country = {.cc="SU", .schan=1, .nchan=13, .max_tx_power=84, .policy=WIFI_COUNTRY_POLICY_MANUAL};
    esp_wifi_set_country(&wifi_country) ; /* set country for channel range [1, 13] */
    esp_wifi_set_max_tx_power(84);
//    WiFi.setTxPower(WIFI_POWER_20_5dBm); // 

    //debug wifi
    esp_wifi_scan_stop();
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);

 // Set Bluetooth interrupt priority lower than WiFi
  esp_intr_alloc(ETS_BT_MAC_INTR_SOURCE, ESP_INTR_FLAG_IRAM, NULL, NULL, NULL);
  esp_intr_alloc(ETS_WIFI_MAC_INTR_SOURCE, ESP_INTR_FLAG_IRAM, NULL, NULL, NULL);
  esp_intr_set_in_iram(ETS_WIFI_MAC_INTR_SOURCE, true);
//  esp_intr_set_in_iram(ETS_WIFI_MAC_INTR_SOURCE, true);
  
 // Disable unnecessary Bluetooth features
  disableUnnecessaryBtFeatures();

  // Configure coexistence
  configureCoexistence();
   
    
  // Initialize the asyncUDP object
  if (udp.listenMulticast(multicastIP, multicastPort)) {
 //   Serial0.println("UDP listening"); // who cares
    delay(500);
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }
    minutes_millis_last = millis() ; 
    
#ifdef GRAPH_TEST
   for (uint16_t j = 0 ; j <(NUMBER_OF_BUFFERS) ; j++){
    for (uint16_t i = 0; i < (MINUTES_GRAPH_BUFFER_MAX); i++) {
      // initalize array with bogus values
      switch (j){
      case 0:
      minutes_buffer[j][i]=random(6000);
      if (minutes_buffer[j][i]>3000){minutes_buffer[j][i]=NAN;}
      if (minutes_buffer[j][i]<100){minutes_buffer[j][i]=NAN;}
      break;
      case 1:
      minutes_buffer[j][i]=random(2000)*0.001;
      if (minutes_buffer[j][i]>1.5){minutes_buffer[j][i]=NAN;}
      if (minutes_buffer[j][i]<1.0){minutes_buffer[j][i]=NAN;}
      break;      
      case 2: 
      minutes_buffer[j][i]=random(1000)*-0.1;
      if (minutes_buffer[j][i]<-80){minutes_buffer[j][i]=NAN;}
      if (minutes_buffer[j][i]>-30.0){minutes_buffer[j][i]=NAN;}
      break;      
      } // switch(j){

    }//for (uint16_t i = 0; i < (MINUTES_GRAPH_BUFFER_MAX); i++) {
    switch (j){
      case 0:
      minutes_buffer_min[j]= 0.0;
      minutes_buffer_max[j]= 3.3; 
      break;
      case 1:
      minutes_buffer_min[j]= 0.0;
      minutes_buffer_max[j]= 2; 
      break;
      case 2:
      minutes_buffer_min[j]= -90;
      minutes_buffer_max[j]= 0 ; 
      break;
    } //switch (j){
   }//for (uint16_t j = 0 ; j <(NUMBER_OF_BUFFERS) ; j++){
#endif // GRAPH_TEST

// wifi_country_t wifi_country = WIFI_COUNTRY_US ;
//    static wifi_country_t wifi_country = {.cc="US", .schan=1, .nchan=13 , .max_tx_power=78, .policy=WIFI_COUNTRY_POLICY_AUTO};
//    ESP_ERROR_CHECK ( esp_wifi_set_country ( &wifi_country ) );
//
//    Serial0.println(WiFi.getTxPower());
//    wifi_country_t myCountry;
//      if(esp_wifi_get_country(&myCountry) == ESP_OK){
//        Serial0.print("Country Code: ");
//        Serial0.println(myCountry.cc);
//        Serial0.println(myCountry.max_tx_power);
//      }      

  initializeArray();  // Initialize the array with -1


}


// Function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet) { // there is 128k of IRAM and it does not overlap with heap and DRAM. 
    // putting stuff in IRAM actually offloads DRAM and increases heap. 
    // stuff in IRAM needs to be padded to 32bit structures. 
//void handlePacket(AsyncUDPPacket packet) { // slower but less memory
  // Copy the UDP packet data to the pulse data variable
  memcpy((byte*)&tframe, packet.data(), sizeof(tframe));
  new_packet = true; 
  total_packets++; 
}

/* old logic
void update_minute_buffer () {
  minutes_buffer_min = minutes_buffer_max; // set graphMin to last graphMax value
  minutes_buffer_max = 0;
  for (int i = 0; i < (MINUTES_GRAPH_BUFFER_MAX - 1); i++) {

    minutes_buffer[i] = minutes_buffer[i + 1];
    if (minutes_buffer[i] > minutes_buffer_max) {
      minutes_buffer_max = minutes_buffer[i];
      }
    if (minutes_buffer[i] < minutes_buffer_min) {
      minutes_buffer_min = minutes_buffer[i];
    }
    // graphMax_oled = max(graphMax_oled, rollingBuffer_oled[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer
    if(new_packet) {
      minutes_buffer[MINUTES_GRAPH_BUFFER_MAX-1] = tframe.voltage_ADC0;
      new_packet = false; 
    } else {
      minutes_buffer[MINUTES_GRAPH_BUFFER_MAX-1] = NAN ;      
    }
}
*/


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
          if (tframe.radio_active_time != NAN) { // if sensor reports no value, leave previous value . this allows updating only part of data without deleting other data
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.radio_active_time;
          }         
          break; 
          case 1:
          if (tframe.voltage_ADC0 != NAN) { // if sensor reports no value, leave previous value . this allows updating only part of data without deleting other data
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.voltage_ADC0;
          }
          break;
          case 2:
          if (tframe.wifi_rssi != NAN) { // if sensor reports no value, leave previous value . this allows updating only part of data without deleting other data
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.wifi_rssi;        
          }
          break;
          case 3:
                if (tframe.SH4x_rel_humidity  != NAN) { // if sensor reports no value, leave previous value . this allows updating only part of data without deleting other data
                minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.SH4x_rel_humidity;      
          }
          break; 
          case 4:
                if (tframe.SH4x_temperature != NAN) { // if sensor reports no value, leave previous value . this allows updating only part of data without deleting other data
                minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.SH4x_temperature;
          }
          break;
          default:
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
          case 2:
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = NAN;        
          break;
          case 3:
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = NAN;        
          break;
          case 4:
        minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = NAN;        
          break;
          default:
          break;
        }
      }
#else  //#ifdef PACKETS_FROM_NETWORK  

//        minutes_buffer[MINUTES_GRAPH_BUFFER_MAX-1] = tframe.CO_sensor; // local
        switch (j) {
          case 0:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.radio_active_time;
          break;
          case 1:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.voltage_ADC0;
          break;
          case 2:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.wifi_rssi;        
//          Serial0.println(MINUTES_GRAPH_BUFFER_MAX-1);
          break;
          case 3:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.SH4x_rel_humidity;
          break;
          case 4:
          minutes_buffer[j][MINUTES_GRAPH_BUFFER_MAX-1] = tframe.SH4x_temperature;
          break;
          default:
          break;
        }
#endif //#ifdef PACKETS_FROM_NETWORK      

// local data should be updated here
  }

#ifdef PACKETS_FROM_NETWORK      
      if(new_packet) {new_packet = false;}  
#endif //#ifdef PACKETS_FROM_NETWORK      

}

/* deprecated
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

*/

/*
// Function to add a line to the buffer
void bufferLine(int x1, int y1, int x2, int y2, color16_t color) {
    if (lineBufferIndex < MAX_LINES) {
        lineBuffer[lineBufferIndex].x1 = x1;
        lineBuffer[lineBufferIndex].y1 = y1;
        lineBuffer[lineBufferIndex].x2 = x2;
        lineBuffer[lineBufferIndex].y2 = y2;
        lineBuffer[lineBufferIndex].color = color;
        lineBufferIndex++;
    } else {
        // Handle buffer overflow (e.g., log an error or extend buffer size)
    }
}

// Function to draw a line from the buffer and mark it as drawn - not clearing underneath
void drawBufferedLineNoClr(uint16_t index) {
    if (index < lineBufferIndex && lineBuffer[index].color != DRAWN_MAGIC_NUMBER) {
        BlueDisplay1.drawLine(lineBuffer[index].x1, lineBuffer[index].y1, 
                              lineBuffer[index].x2, lineBuffer[index].y2, 
                              lineBuffer[index].color);
        lineBuffer[index].color = DRAWN_MAGIC_NUMBER;  // Mark the line as drawn
    }
}

// Function to draw a line from the buffer and mark it as drawn - clearing rect under the line (height is assumed to be graph height)
void drawBufferedLineClr(uint16_t index) {
    if (index < lineBufferIndex && lineBuffer[index].color != DRAWN_MAGIC_NUMBER) {
        uint16_t x1 = lineBuffer[index].x1;
        uint16_t y1 = lineBuffer[index].y1;
        uint16_t x2 = lineBuffer[index].x2;
        uint16_t y2 = lineBuffer[index].y2;
        
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
        BlueDisplay1.drawLine(x1, y1, x2, y2, lineBuffer[index].color);
        
        // Mark the line as drawn
        lineBuffer[index].color = DRAWN_MAGIC_NUMBER;
    }
}
*/

/*
// Modified plotGraph function with buffer reset and storing global graph height
// note that there is only one buffer so if you wish to draw two plots at the same time you need to 
// invent new way to create multiple buffers. 
// for now this is simply not implemented and if you want to plot two graphs either use unbuffered plotgraph to plot another
// implement semaphores to make sure new buffer is created once old graph is fully drawn
// or implement more buffers and passing buffer indexes to all the functions. 

void plotGraph_buffered(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor) {
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
    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (((float)graphWidth-(LEGEND_LABEL_FONT_SIZE*LEGEND_LABEL_CHARS)))/ (float)(pointsToPlot - 1);
//    float xScale = (((float)graphWidth))/ (float)(pointsToPlot - 1);

    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

//    color16_t whiteColor = COLOR16_BLACK;
//    color16_t grayColor = COLOR16_BLUE;
    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR) ; // buffer the grayish color for faster plotting


    for (uint16_t i = startIndex; i < dataSize; i++) {
        if (!isnan(data[i])) {  // Check if the current data point is valid
            uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                bufferLine(lastX, lastY, x, y, graphColor);  // Buffer the line instead of drawing it immediately
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
                    bufferLine(lastX, lastY, nextValidX, nextValidY, grayishColor);  // Buffer the line for missing data, using slightly more pale color
                }
            }
            lastValid = false;
        }
    }

}

*/



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

/*
// Function to draw all lines from the buffer
void drawAllBufferedLines() {
    for (int i = 0; i < lineBufferIndex; i++) {
        drawBufferedLineClr(i);
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
          if (lineBuffer[randomIndex].color != DRAWN_MAGIC_NUMBER) {
            break;    // break if found
          }
        }
        drawBufferedLineClr(randomIndex);
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
        drawBufferedLineClr(randomIndex);
    }
}

// Function to draw a specified number of lines sequentially from the buffer
void drawSequentialLines(int numLinesToDraw) {
    // Draw lines sequentially from the current position
    for (int i = 0; i < numLinesToDraw; i++) {
        if (currentLineIndex < lineBufferIndex) {
            drawBufferedLineClr(currentLineIndex);
            currentLineIndex++;
        } else {
            // If we reach the end of the buffer, stop drawing
            graphComplete = true;
            break;
        }
    }
}
*/

// Function to implement a 16-bit Galois LFSR
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

/*
// Function to draw a specified number of lines based on the LFSR sequence
void drawLinesUsingLFSR(int numLinesToDraw) {
    uint16_t linesDrawn = 0;
    bool wrapped = false;

    while (linesDrawn < numLinesToDraw) {
        uint16_t lineIndex = galoisLFSR(&wrapped); 
        // Ensure the generated index is within the range of valid buffered lines
        if (lineIndex < lineBufferIndex) {
//            drawBufferedLineClr(lineIndex-1);  // the range starts from and is never 0 1 so shift by 1.
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
*/




// new graph plotting system
// it is called like plotgraph_buffered
// but draws directly to screen - only the section we specify. 
// if graph index is 0 , section is cleared
// if graph index is >0 , data is plotted over 
// there is no check if section was drawn before - use LFSR to determine that or draw sequentially. 
// this allows drawing individual graphs (with obligatory graph 0 to clear underneath)
// or draw multi graphs . 
// main benefit over buffered approach is using less memory - ESP32 has only 100k of memory available for static arrays and it's shared with heap. about 28k in practice when wifi and bt is on. 

//void plotGraph(float *data, uint16_t data_startpoint, uint16_t graphPosX, uint16_t graphPosY,
//               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor, uint8_t buffer_index, bool clear_under = false) {
void plotGraphSection(float *data, uint16_t data_startpoint, uint16_t graph_data_size, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor, bool clear_under = false) {
// version with buffer index passed to operator. 
// it allows plotting graphs of different size on same point
    
    // Determine the number of graphs and the number of data points in each graph
    //uint16_t number_of_graphs = sizeof(data_array) / sizeof(data_array[0]);
    //uint16_t number_of_graphs = NUMBER_OF_BUFFERS ; 
//    uint16_t graph_data_size = sizeof(data[0]) / sizeof(data[0][0]);
//    uint16_t graph_data_size = sizeof(data) / sizeof(data[0]);

    //Serial0.println(graph_data_size);
    //Serial0.println(sizeof(*data));


//    // Ensure the graph_number is within bounds
//    if (graph_number >= number_of_graphs) {
//        return;  // Invalid graph number, nothing to plot
//    }

    // Ensure the data points to plot are within bounds
    //if (data_startpoint >= graph_data_size - 1) {
    //    return;  // Out of bounds, nothing to plot
    //}

    // Get the current and next data points from the correct graph
//    float currentValue = data[buffer_index][data_startpoint];
//    float nextValue = data[buffer_index][data_startpoint + 1];
    float currentValue = data[data_startpoint];
    float nextValue = data[data_startpoint + 1];

    // Calculate scaling factor for converting data to graph coordinates
    float scale = (graph_max - graph_min) / (float)graphHeight;

    // Calculate x and y positions of the current point
    uint16_t x0 = graphPosX + (data_startpoint * graphWidth) / graph_data_size;
    //Serial0.println(x0);

    // Clear the rectangle under the segment if graph_number is 0
//    if (graph_number == 0) {
    if (clear_under) {
//        tft.fillRect(x0, graphPosY, graphWidth / graph_data_size, graphHeight, ILI9341_BLACK);  // Clear only the segment area
        BlueDisplay1.fillRect(x0, graphPosY, x0+graphWidth/(graph_data_size)+1, graphPosY+graphHeight, COLOR_BACKGROUND); // bluedisplay selects bounds instead of relative position
    }
    // Skip plotting if the current value is NaN
    if (isnan(currentValue)) {
        return;  // Nothing to plot if the current point is NaN
    }
    uint16_t y0 = graphPosY + graphHeight - (uint16_t)((currentValue - graph_min) / scale);

    // If the next value is NaN, draw a dot at the current point
    if (isnan(nextValue)) {
    //    tft.drawPixel(x0, y0, ILI9341_WHITE);  // Draw a dot
    BlueDisplay1.drawPixel(x0, y0, graphColor);  // Draw a dot

    } else {
        // Calculate x and y positions of the next point
        uint16_t x1 = graphPosX + ((data_startpoint + 1) * graphWidth) / graph_data_size;
        uint16_t y1 = graphPosY + graphHeight - (uint16_t)((nextValue - graph_min) / scale);

        // Draw a line between the current point and the next point
//        tft.drawLine(x0, y0, x1, y1, ILI9341_WHITE);  // Draw a line
          BlueDisplay1.drawLine(x0, y0, x1, y1, graphColor);
    }
}

void plotGraph(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor, bool clear_under = false) {
// Function to draw all lines from the buffer
    for (int i = 0; i < (dataSize-1); i++) {
        plotGraphSection(data, i, dataSize, graphPosX, graphPosY,
                graphWidth, graphHeight, graph_min, graph_max, graphColor,clear_under);
    }
    graphComplete = true;
}

//TODO : passing pointers and array parameters instead of short circuit. 

void plotGraphMulti(uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight ){ //,
               // float graph_min, float graph_max, color16_t graphColor, bool clear_under = false) {
    for (int j = 0; j < (dataSize-1); j++) {
        for (uint8_t i = 0 ; i < NUMBER_OF_GRAPHS; i++ ) {
          bool clear_under; 
          if (i == 0) {clear_under=true;} else {clear_under=false;}
          if (graph_order[i] >=0 ){
            plotGraphSection(minutes_buffer[graph_order[i]], j, dataSize, graphPosX, graphPosY,
                graphWidth, graphHeight, minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]], GRAPH_COLOR[graph_order[i]],clear_under);
          //plotGraph(minutes_buffer[graph_order[i]],minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y),
          //      minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],GRAPH_COLOR[graph_order[i]], clear_under );       
          }  
        }
    }
    graphComplete = true;

}
void loop() {
     if (debug_millis_last<millis()) {      
      DisplayDebug();
      debug_millis_last=millis()+DEBUG_INTERVAL;
     } 

     if (labels_millis_last<millis()) {      
 //     drawLabels(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[current_graph], minutes_buffer_max[current_graph),GRAPH_COLOR[current_graph]);    
      labels_millis_last=millis()+LABELS_INTERVAL;
     if (graphComplete){
        if (current_graph>=0){ //positive numbers mean real graphs
            drawLabels(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[current_graph], minutes_buffer_max[current_graph],GRAPH_COLOR[current_graph]);     
        } else { //negative numbers mean layouts. draw labels for the first graph in stack
           if ((graph_order[0])>=0) {
              drawLabels(minutes_buffer[graph_order[0]], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[graph_order[0]], minutes_buffer_max[graph_order[0]],GRAPH_COLOR[graph_order[0]]);     
           }
        }
      }
     } 

    if (minutes_millis_last<millis()) {
      minutes_millis_last=millis()+MINUTES_INTERVAL;
      update_minute_buffer();      
//    plotGraph_buffered(minutes_buffer, minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min, minutes_buffer_max,GRAPH1_COLOR);
   if (graphComplete){
         if (current_graph>=0) { // positive graph values mean real graphs
            plotGraph(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y),
                  minutes_buffer_min[current_graph], minutes_buffer_max[current_graph],GRAPH_COLOR[current_graph],true);     
//      plotGraph_buffered(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y), minutes_buffer_min[current_graph], minutes_buffer_max[current_graph],GRAPH_COLOR[current_graph]);
         }else {
      plotGraphMulti(minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y)); 
      //          minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],GRAPH_COLOR[graph_order[i]], clear_under );
    }
         
      }    
    }//if (minutes_millis_last<millis()) {

    
 
//    drawSequentialLines(int numLinesToDraw)
//    if (!graphComplete) { drawSequentialLines(2);} // draws from left to right , sequential 
    //if (!graphComplete) {drawRandomLinesFast(6); } // draws random lines , not caring if drawn random segment had been drawn before (fast to complete)
//    if (!graphComplete) {drawRandomLines(8); }     // draw random lines, ensuring random number drawn hit the area not drawn before (high cpu but low bandwidth)

   // if (!graphComplete) {drawLinesUsingLFSR(CHUNKS_PER_DRAW); }  // draw basing on LFSR generated looping sequence. 
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
 //   BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, displayWidth,displayHeight);
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
//    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, displayWidth,displayHeight);
#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, displayWidth,displayHeight);
#else //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE , displayWidth,displayHeight);
#endif //#ifdef DO_NOT_NEED_BASIC_TOUCH_EVENTS 

    BlueDisplay1.clearDisplay(COLOR_BACKGROUND);
    if (current_graph>=0) { // positive graph values mean real graphs
      plotGraph(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y),
              minutes_buffer_min[current_graph], minutes_buffer_max[current_graph],GRAPH_COLOR[current_graph],true);
      //  plotGraph(minutes_buffer, minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X, displayHeight-(GRAPH_Y), minutes_buffer_min, minutes_buffer_max,GRAPH1_COLOR,clear_under);
    }else { // negative means drawing layouts . 
      //todo : more layouts and ability to choose layout to configure by swipe up. 
      plotGraphMulti(minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y)); 
    }

    
//    TouchButtonBlinkStartStop.drawButton();
}

void handleSwipe(struct Swipe *const swipe_param) {
  if (swipe_param->SwipeMainDirectionIsX && swipe_param->TouchDeltaAbsMax > 100) {
    BlueDisplay1.clearDisplay(COLOR_BACKGROUND);
    if (swipe_param->TouchDeltaX < 0 && current_graph < (NUMBER_OF_BUFFERS-1)) {current_graph++;} // swipe right
    if (swipe_param->TouchDeltaX > 0 && current_graph > -1) {current_graph--;} // swipe left
    Serial0.println(current_graph);

    if (current_graph>=0) { // positive graph values mean real graphs
      plotGraph(minutes_buffer[current_graph], minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y),
              minutes_buffer_min[current_graph], minutes_buffer_max[current_graph],GRAPH_COLOR[current_graph],true);
    } else {
/*
      for (uint8_t i = 0 ; i < NUMBER_OF_GRAPHS; i++ ) {
        bool clear_under; 
        if (i == 0) {clear_under=true;} else {clear_under=false;}
        if (graph_order[i] >=0 ){
          plotGraph(minutes_buffer[graph_order[i]],minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X,displayHeight-(GRAPH_Y),
                minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],GRAPH_COLOR[graph_order[i]], clear_under );       
        }  
      }
 */
      plotGraphMulti(minutes_dataArraySize, GRAPH_X, GRAPH_Y, displayWidth-GRAPH_X-LEGEND_LABEL_FONT_WIDTH*4,displayHeight-(GRAPH_Y)); 
      //          minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],GRAPH_COLOR[graph_order[i]], clear_under );
    }

    DisplayDebug();
  }else {
// vertical swipe adds or removes graphs to the graph order array
    if (swipe_param->TouchDeltaY < 0 ) {
        if (current_graph>=0) {addNumber(current_graph);};
          }// swipe up
    if (swipe_param->TouchDeltaY > 0 ) {
          if (current_graph>=0) {removeNumber(current_graph);}
          }// swipe down 
  }
    printArray(LEGEND_LABEL_FONT_WIDTH*16,LEGEND_LABEL_FONT_SIZE);

  
}
