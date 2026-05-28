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
const unsigned long LIS_WINDOW = 650;
const unsigned long OFFLINE_TIMEOUT = 30000;

WebServer server(80);

struct BatteryCell {
    float voltage = 0.0;
    bool online = false;
    unsigned long lastUpdateTime = 0;
    
    uint8_t receivedChunks[8];
    bool chunkPresent[8];
    uint8_t receivedChecksum = 0xFF;
};

BatteryCell pack[TOTAL_NODES];
int cycleCounter = 0;

void handleData() {
    String json = "{ \"nodes\": [";
    for (int i = 0; i < TOTAL_NODES; i++) {
        json += "{";
        json += "\"id\":" + String(i) + ",";
        json += "\"v\":" + String(pack[i].voltage, 3) + ",";
        json += "\"online\":" + String(pack[i].online ? "true" : "false") + ",";
        json += "\"age\":" + String((millis() - pack[i].lastUpdateTime) / 1000);
        json += "}";
        if (i < TOTAL_NODES - 1) json += ",";
    }
    json += "] }";
    server.send(200, "application/json", json);
}

String getDashboardHTML() {
    String html = "<!DOCTYPE html><html lang='en'><head>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>BMS Dashboard</title>";
    html += "<script src='https://cdn.tailwindcss.com'></script>";
    html += "<meta http-equiv='refresh' content='2'>";
    html += "</head><body class='bg-slate-900 text-slate-100 min-h-screen font-sans'>";
    html += "<div class='container mx-auto px-4 py-8'>";
    html += "  <header class='flex justify-between items-center border-b border-slate-700 pb-6 mb-8'>";
    html += "    <h1 class='text-2xl font-bold'>BMS Master</h1>";
    html += "    <div class='text-sm text-slate-400'>Cycle: " + String(cycleCounter) + "</div>";
    html += "  </header>";
    html += "  <div class='grid grid-cols-2 sm:grid-cols-4 md:grid-cols-8 gap-4'>";
    for (int i = 0; i < TOTAL_NODES; i++) {
        String status = pack[i].online ? "text-emerald-400" : "text-slate-600";
        html += "    <div class='bg-slate-800 p-3 rounded-lg text-center border border-slate-700'>";
        html += "      <div class='text-[10px] uppercase text-slate-500'>Node " + String(i) + "</div>";
        html += "      <div class='text-lg font-bold " + status + "'>" + (pack[i].online ? String(pack[i].voltage, 2) + "V" : "OFF") + "</div>";
        html += "    </div>";
    }
    html += "  </div></div></body></html>";
    return html;
}

void handleRoot() {
    server.send(200, "text/html", getDashboardHTML());
}

void processIncomingFragment(int nodeID, uint8_t rc5Cmd) {
    if (rc5Cmd & 0x20) { // Data packet (0x20 flag)
        uint8_t index = (rc5Cmd >> 2) & 0x07;
        uint8_t payload = rc5Cmd & 0x03;
        if (index < 8) {
            pack[nodeID].receivedChunks[index] = payload;
            pack[nodeID].chunkPresent[index] = true;
        }
    } else if (rc5Cmd & 0x10) { // Checksum packet (0x10 flag)
        pack[nodeID].receivedChecksum = rc5Cmd & 0x0F;
    }
}

void pollNode(int nodeID) {
    for (int j = 0; j < 8; j++) pack[nodeID].chunkPresent[j] = false;
    pack[nodeID].receivedChecksum = 0xFF;

    IrSender.sendRC5(nodeID, 0x01, 1);
    delay(50);
    IrSender.sendRC5(nodeID, 0x01, 1);

    unsigned long startListen = millis();
    while (millis() - startListen < LIS_WINDOW) {
        if (IrReceiver.decode()) {
            // Master MUST check address matches polled node
            if (IrReceiver.decodedIRData.protocol == RC5 &&
                IrReceiver.decodedIRData.address == nodeID) {
                processIncomingFragment(nodeID, IrReceiver.decodedIRData.command);
            }
            IrReceiver.resume();
        }
        server.handleClient();
        yield();
    }

    bool allPresent = true;
    for (int j = 0; j < 8; j++) if (!pack[nodeID].chunkPresent[j]) allPresent = false;

    if (allPresent && pack[nodeID].receivedChecksum != 0xFF) {
        uint16_t mv = 0;
        uint8_t calcSum = 0;
        for (int j = 0; j < 8; j++) {
            mv |= (uint16_t)pack[nodeID].receivedChunks[j] << (j * 2);
            calcSum += pack[nodeID].receivedChunks[j];
        }

        if ((calcSum & 0x0F) == pack[nodeID].receivedChecksum) {
            pack[nodeID].voltage = mv / 1000.0f;
            pack[nodeID].online = true;
            pack[nodeID].lastUpdateTime = millis();
        }
    } else {
        if (pack[nodeID].online && (millis() - pack[nodeID].lastUpdateTime > OFFLINE_TIMEOUT)) {
            pack[nodeID].online = false;
        }
    }

    // Inter-node delay to let IR reflections clear
    delay(20);
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN, DISABLE_LED_FEEDBACK);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
}

void loop() {
    server.handleClient();
    for (int i = 0; i < TOTAL_NODES; i++) {
        if (pack[i].online || (cycleCounter % 10 == 0)) {
            pollNode(i);
        }
        yield();
    }
    cycleCounter++;
}
