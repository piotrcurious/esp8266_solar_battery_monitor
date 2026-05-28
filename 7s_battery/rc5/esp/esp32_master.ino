#include <WiFi.h>
#include <WebServer.h>
#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const char *ssid = "BMS_Master_AP";
const char *password = "12345678";
const int IR_RX_PIN = 15;
const int IR_TX_PIN = 4;
const int TOTAL_NODES = 32;
const unsigned long LIS_WINDOW = 500; // Shorter window for 8-packet transmission
const unsigned long OFFLINE_TIMEOUT = 20000; // 20s

WebServer server(80);

struct BatteryCell {
    float voltage = 0.0;
    bool online = false;
    unsigned long lastUpdateTime = 0;
    
    // Decoding state
    uint8_t receivedChunks[8];
    bool chunkPresent[8];
};

BatteryCell pack[TOTAL_NODES];

// --- Web Dashboard ---
String getDashboardHTML() {
    String html = "<!DOCTYPE html><html lang='en'><head>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>BMS Telemetry Dashboard</title>";
    html += "<script src='https://cdn.tailwindcss.com'></script>";
    html += "<meta http-equiv='refresh' content='2'>";
    html += "</head><body class='bg-slate-900 text-slate-100 min-h-screen font-sans'>";
    
    html += "<div class='container mx-auto px-4 py-8'>";
    html += "  <header class='flex flex-col md:flex-row justify-between items-center border-b border-slate-700 pb-6 mb-8'>";
    html += "    <div>";
    html += "      <h1 class='text-3xl font-extrabold text-transparent bg-clip-text bg-gradient-to-r from-teal-400 to-indigo-400'>BMS Master Control</h1>";
    html += "      <p class='text-slate-400 text-sm mt-1'>Network SSID: <span class='text-teal-400 font-semibold'>" + String(ssid) + "</span> | AP IP: <span class='text-teal-400 font-semibold'>" + WiFi.softAPIP().toString() + "</span></p>";
    html += "    </div>";
    
    float totalVoltage = 0.0;
    int activeCells = 0;
    for (int i = 0; i < TOTAL_NODES; i++) {
        if (pack[i].online) {
            totalVoltage += pack[i].voltage;
            activeCells++;
        }
    }
    
    html += "    <div class='flex gap-4 mt-4 md:mt-0'>";
    html += "      <div class='bg-slate-800 border border-slate-700 rounded-xl p-4 text-center min-w-[130px]'>";
    html += "        <span class='block text-[10px] text-slate-400 uppercase font-bold tracking-wider'>Active Nodes</span>";
    html += "        <span class='text-2xl font-bold text-teal-400'>" + String(activeCells) + " / " + String(TOTAL_NODES) + "</span>";
    html += "      </div>";
    html += "      <div class='bg-slate-800 border border-slate-700 rounded-xl p-4 text-center min-w-[130px]'>";
    html += "        <span class='block text-[10px] text-slate-400 uppercase font-bold tracking-wider'>Series Voltage</span>";
    html += "        <span class='text-2xl font-bold text-indigo-400'>" + String(totalVoltage, 2) + " V</span>";
    html += "      </div>";
    html += "    </div>";
    html += "  </header>";

    html += "  <div class='grid grid-cols-2 sm:grid-cols-4 md:grid-cols-6 lg:grid-cols-8 gap-4'>";
    for (int i = 0; i < TOTAL_NODES; i++) {
        bool isOnline = pack[i].online;
        float v = pack[i].voltage;
        
        String borderCol = "border-slate-800 bg-slate-800/40";
        String valCol = "text-slate-500";
        String statusDot = "bg-red-500 animate-pulse";
        
        if (isOnline) {
            statusDot = "bg-emerald-400";
            if (v >= 3.65) {
                borderCol = "border-emerald-500/30 bg-slate-800";
                valCol = "text-emerald-400";
            } else if (v >= 3.0) {
                borderCol = "border-amber-500/30 bg-slate-800";
                valCol = "text-amber-400";
            } else {
                borderCol = "border-red-500/30 bg-slate-800 animate-pulse";
                valCol = "text-red-400";
            }
        }
        
        html += "    <div class='border rounded-xl p-4 transition duration-200 " + borderCol + "'>";
        html += "      <div class='flex justify-between items-center mb-1'>";
        html += "        <span class='text-[10px] font-semibold uppercase tracking-wider text-slate-400'>Node " + String(i) + "</span>";
        html += "        <span class='h-2.5 w-2.5 rounded-full " + statusDot + "'></span>";
        html += "      </div>";
        
        if (isOnline) {
            html += "      <div class='text-lg font-bold tracking-tight " + valCol + "'>" + String(v, 3) + " V</div>";
            unsigned long age = (millis() - pack[i].lastUpdateTime) / 1000;
            html += "      <div class='text-[9px] text-slate-500 mt-0.5'>Age: " + String(age) + "s ago</div>";
        } else {
            html += "      <div class='text-md font-bold tracking-tight text-slate-600'>OFFLINE</div>";
            html += "      <div class='text-[9px] text-slate-600 mt-0.5'>Unreachable</div>";
        }
        html += "    </div>";
    }
    html += "  </div>";
    html += "</div></body></html>";
    return html;
}

void handleRoot() {
    server.send(200, "text/html", getDashboardHTML());
}

void processIncomingFragment(int nodeID, uint8_t rc5Cmd) {
    uint8_t index = (rc5Cmd >> 2) & 0x07; // 3-bit index (0-7)
    uint8_t payload = rc5Cmd & 0x03;      // 2-bit payload

    if (index < 8) {
        pack[nodeID].receivedChunks[index] = payload;
        pack[nodeID].chunkPresent[index] = true;
    }
}

void pollNode(int nodeID) {
    for (int j = 0; j < 8; j++) pack[nodeID].chunkPresent[j] = false;

    // Wake and Poll
    IrSender.sendRC5(nodeID, 0x01, 1);
    delay(50);
    IrSender.sendRC5(nodeID, 0x01, 1);

    unsigned long startListen = millis();
    int packetsReceived = 0;

    while (millis() - startListen < LIS_WINDOW) {
        // Early exit if we haven't seen anything for 200ms
        if (packetsReceived == 0 && (millis() - startListen > 250)) break;

        if (IrReceiver.decode()) {
            if (IrReceiver.decodedIRData.protocol == RC5 && 
                IrReceiver.decodedIRData.address == nodeID && 
                (IrReceiver.decodedIRData.command & 0x20)) {
                
                processIncomingFragment(nodeID, IrReceiver.decodedIRData.command);
                packetsReceived++;
            }
            IrReceiver.resume();
        }
        server.handleClient();
        yield();
    }

    bool complete = true;
    for (int j = 0; j < 8; j++) if (!pack[nodeID].chunkPresent[j]) complete = false;

    if (complete) {
        uint16_t mv = 0;
        for (int j = 0; j < 8; j++) {
            mv |= (uint16_t)pack[nodeID].receivedChunks[j] << (j * 2);
        }

        if (mv > 0 && mv < 10000) {
            pack[nodeID].voltage = mv / 1000.0f;
            pack[nodeID].online = true;
            pack[nodeID].lastUpdateTime = millis();
        }
    } else {
        if (pack[nodeID].online && (millis() - pack[nodeID].lastUpdateTime > OFFLINE_TIMEOUT)) {
            pack[nodeID].online = false;
        }
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN, DISABLE_LED_FEEDBACK);
    server.on("/", handleRoot);
    server.begin();
    Serial.println("BMS Master Online");
}

void loop() {
    server.handleClient();
    for (int i = 0; i < TOTAL_NODES; i++) {
        pollNode(i);
        yield();
    }
}
