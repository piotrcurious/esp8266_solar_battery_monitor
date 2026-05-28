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
const unsigned long LIS_WINDOW = 550;
const unsigned long OFFLINE_TIMEOUT = 30000;

WebServer server(80);

enum NodeStatus { OFFLINE, ONLINE, INCOMPLETE, CHECKSUM_ERROR };

struct BatteryCell {
    float voltage = 0.0;
    NodeStatus status = OFFLINE;
    unsigned long lastUpdateTime = 0;
    
    uint8_t receivedChunks[8];
    bool chunkPresent[8];
    uint8_t receivedChecksum = 0xFF;
};

BatteryCell pack[TOTAL_NODES];
int cycleCounter = 0;

String statusToString(NodeStatus s) {
    switch(s) {
        case ONLINE: return "ONLINE";
        case INCOMPLETE: return "INCOMPLETE";
        case CHECKSUM_ERROR: return "CRC_ERR";
        default: return "OFFLINE";
    }
}

void handleData() {
    String json = "{ \"nodes\": [";
    for (int i = 0; i < TOTAL_NODES; i++) {
        json += "{";
        json += "\"id\":" + String(i) + ",";
        json += "\"v\":" + String(pack[i].voltage, 3) + ",";
        json += "\"status\":\"" + statusToString(pack[i].status) + "\",";
        json += "\"age\":" + String((millis() - pack[i].lastUpdateTime) / 1000);
        json += "}";
        if (i < TOTAL_NODES - 1) json += ",";
    }
    json += "] }";
    server.send(200, "application/json", json);
}

void handleReset() {
    for (int i = 0; i < TOTAL_NODES; i++) {
        pack[i].status = OFFLINE;
        pack[i].voltage = 0.0;
    }
    cycleCounter = 0;
    server.send(200, "text/plain", "System Reset OK");
}

String getDashboardHTML() {
    float totalV = 0, minV = 10.0, maxV = 0;
    int active = 0;
    for(int i=0; i<TOTAL_NODES; i++) {
        if(pack[i].status == ONLINE) {
            totalV += pack[i].voltage;
            if(pack[i].voltage < minV) minV = pack[i].voltage;
            if(pack[i].voltage > maxV) maxV = pack[i].voltage;
            active++;
        }
    }
    if(active == 0) minV = 0;

    String html = "<!DOCTYPE html><html lang='en'><head>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>BMS Pro Dashboard</title>";
    html += "<script src='https://cdn.tailwindcss.com'></script>";
    html += "<meta http-equiv='refresh' content='2'>";
    html += "</head><body class='bg-slate-950 text-slate-100 min-h-screen font-sans'>";
    html += "<div class='container mx-auto px-4 py-8'>";

    // Header Stats
    html += "<header class='grid grid-cols-1 md:grid-cols-4 gap-4 mb-8'>";
    html += "  <div class='bg-slate-900 border border-slate-800 p-4 rounded-xl'><div class='text-[10px] text-slate-500 uppercase font-bold tracking-widest'>Series Voltage</div><div class='text-2xl font-black text-teal-400'>" + String(totalV, 2) + "V</div></div>";
    html += "  <div class='bg-slate-900 border border-slate-800 p-4 rounded-xl'><div class='text-[10px] text-slate-500 uppercase font-bold tracking-widest'>Min / Max Cell</div><div class='text-2xl font-black text-indigo-400'>" + String(minV, 2) + " / " + String(maxV, 2) + "</div></div>";
    html += "  <div class='bg-slate-900 border border-slate-800 p-4 rounded-xl'><div class='text-[10px] text-slate-500 uppercase font-bold tracking-widest'>Delta</div><div class='text-2xl font-black text-amber-400'>" + String(maxV - minV, 3) + "V</div></div>";
    html += "  <div class='bg-slate-900 border border-slate-800 p-4 rounded-xl'><div class='text-[10px] text-slate-500 uppercase font-bold tracking-widest'>Cycle / Active</div><div class='text-2xl font-black text-emerald-400'>" + String(cycleCounter) + " / " + String(active) + "</div></div>";
    html += "</header>";

    html += "<div class='grid grid-cols-2 sm:grid-cols-4 md:grid-cols-8 gap-4 mb-8'>";
    for (int i = 0; i < TOTAL_NODES; i++) {
        String color = "border-slate-800 text-slate-600";
        if(pack[i].status == ONLINE) color = "border-emerald-500/30 text-emerald-400 bg-emerald-500/5";
        else if(pack[i].status == INCOMPLETE) color = "border-amber-500/30 text-amber-400 animate-pulse";
        else if(pack[i].status == CHECKSUM_ERROR) color = "border-orange-500/30 text-orange-500";

        html += "    <div class='border p-3 rounded-lg text-center transition duration-300 " + color + "'>";
        html += "      <div class='text-[9px] uppercase font-bold opacity-60'>Node " + String(i) + "</div>";
        html += "      <div class='text-md font-black'>" + (pack[i].status == ONLINE ? String(pack[i].voltage, 2) + "V" : statusToString(pack[i].status)) + "</div>";
        if(pack[i].status != OFFLINE) {
            unsigned long age = (millis() - pack[i].lastUpdateTime) / 1000;
            html += "      <div class='text-[8px] opacity-40'>" + String(age) + "s ago</div>";
        }
        html += "    </div>";
    }
    html += "  </div>";

    html += "  <footer class='flex justify-between items-center border-t border-slate-800 pt-6'>";
    html += "    <button onclick=\"fetch('/reset').then(()=>location.reload())\" class='bg-red-500/10 hover:bg-red-500/20 text-red-500 text-xs font-bold py-2 px-4 rounded border border-red-500/20 transition'>Reset Pack State</button>";
    html += "    <div class='text-[10px] text-slate-600 font-mono'>v4.6.0 | IR-BMS Master</div>";
    html += "  </footer>";

    html += "</div></body></html>";
    return html;
}

void handleRoot() {
    server.send(200, "text/html", getDashboardHTML());
}

void processIncomingFragment(int nodeID, uint8_t rc5Cmd) {
    if (rc5Cmd & 0x20) {
        uint8_t index = (rc5Cmd >> 2) & 0x07;
        uint8_t payload = rc5Cmd & 0x03;
        if (index < 8) {
            pack[nodeID].receivedChunks[index] = payload;
            pack[nodeID].chunkPresent[index] = true;
        }
    } else if (rc5Cmd & 0x10) {
        pack[nodeID].receivedChecksum = rc5Cmd & 0x0F;
    }
}

// Internal polling logic, returns true if success
bool pollNodeInternal(int nodeID) {
    for (int j = 0; j < 8; j++) pack[nodeID].chunkPresent[j] = false;
    pack[nodeID].receivedChecksum = 0xFF;

    // Single poll command (no repeats) to minimize collision risk
    IrSender.sendRC5(nodeID, 0x01, 0);

    unsigned long startListen = millis();
    bool anyData = false;
    while (millis() - startListen < LIS_WINDOW) {
        if (IrReceiver.decode()) {
            if (IrReceiver.decodedIRData.protocol == RC5 && IrReceiver.decodedIRData.address == nodeID) {
                processIncomingFragment(nodeID, IrReceiver.decodedIRData.command);
                anyData = true;
            }
            IrReceiver.resume();
        }
        server.handleClient();
        yield();
    }

    if (!anyData) return false;

    bool allPresent = true;
    for (int j = 0; j < 8; j++) if (!pack[nodeID].chunkPresent[j]) allPresent = false;

    if (!allPresent || pack[nodeID].receivedChecksum == 0xFF) {
        pack[nodeID].status = INCOMPLETE;
        return false;
    }

    uint16_t mv = 0;
    uint8_t calcSum = 0;
    for (int j = 0; j < 8; j++) {
        mv |= (uint16_t)pack[nodeID].receivedChunks[j] << (j * 2);
        calcSum += pack[nodeID].receivedChunks[j];
    }

    if ((calcSum & 0x0F) == pack[nodeID].receivedChecksum) {
        pack[nodeID].voltage = mv / 1000.0f;
        pack[nodeID].status = ONLINE;
        pack[nodeID].lastUpdateTime = millis();
        return true;
    } else {
        pack[nodeID].status = CHECKSUM_ERROR;
        return false;
    }
}

void pollNode(int nodeID) {
    // Attempt 1
    if (!pollNodeInternal(nodeID)) {
        // If it failed but wasn't a total timeout, retry once immediately
        if (pack[nodeID].status == INCOMPLETE || pack[nodeID].status == CHECKSUM_ERROR) {
            delay(50); // Small gap before retry
            pollNodeInternal(nodeID);
        } else {
            // Total timeout case
            if (pack[nodeID].status != OFFLINE && (millis() - pack[nodeID].lastUpdateTime > OFFLINE_TIMEOUT)) {
                pack[nodeID].status = OFFLINE;
            }
        }
    }
    delay(20);
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN, DISABLE_LED_FEEDBACK);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/reset", handleReset);
    server.begin();
}

void loop() {
    server.handleClient();
    for (int i = 0; i < TOTAL_NODES; i++) {
        if (pack[i].status != OFFLINE || (cycleCounter % 10 == 0)) {
            pollNode(i);
        }
        yield();
    }
    cycleCounter++;
}
