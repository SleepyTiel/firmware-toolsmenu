#include "tcp_gps.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/wifi/wifi_common.h"
#include <WiFi.h>
#include <globals.h>

namespace TcpGps {
namespace {
constexpr size_t RING_SIZE = 8192;
constexpr uint16_t DEFAULT_PORT = 10110;
constexpr unsigned long RECONNECT_MS = 3000;
constexpr unsigned long CONNECT_TIMEOUT_MS = 5000;

WiFiClient client;
TaskHandle_t taskHandle = nullptr;
SemaphoreHandle_t mutex = nullptr;
uint8_t ring[RING_SIZE];
size_t ringHead = 0;
size_t ringTail = 0;
size_t ringCount = 0;

volatile bool wantConnection = false;
String hostStr = "";
uint16_t portNum = DEFAULT_PORT;
volatile unsigned long lines = 0;
volatile unsigned long bytes = 0;
volatile unsigned long lastRx = 0;
String lastNmeaLine = "";
String lineAccum = "";

void ensureMutex() {
    if (!mutex) mutex = xSemaphoreCreateMutex();
}

bool pushByte(uint8_t b) {
    if (!mutex) return false;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    if (ringCount >= RING_SIZE) {
        ringTail = (ringTail + 1) % RING_SIZE;
        ringCount--;
    }
    ring[ringHead] = b;
    ringHead = (ringHead + 1) % RING_SIZE;
    ringCount++;
    xSemaphoreGive(mutex);
    return true;
}

void clearLocked() {
    ringHead = 0;
    ringTail = 0;
    ringCount = 0;
}

void netTask(void *) {
    unsigned long lastAttempt = 0;
    for (;;) {
        if (!wantConnection) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!WiFi.isConnected()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!client.connected()) {
            unsigned long now = millis();
            if (now - lastAttempt < RECONNECT_MS) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            lastAttempt = now;
            Serial.printf("[TCP GPS] reconnecting %s:%u\n", hostStr.c_str(), portNum);
            client.stop();
            if (!client.connect(hostStr.c_str(), portNum, CONNECT_TIMEOUT_MS)) {
                Serial.printf("[TCP GPS] reconnect failed %s:%u\n", hostStr.c_str(), portNum);
                continue;
            }
            Serial.printf("[TCP GPS] reconnected %s:%u\n", hostStr.c_str(), portNum);
            gpsConnected = true;
        }
        if (client.available() > 0) {
            uint8_t chunk[256];
            int n = client.read(chunk, sizeof(chunk));
            for (int i = 0; i < n; i++) {
                pushByte(chunk[i]);
                bytes++;
                char c = static_cast<char>(chunk[i]);
                if (c == '\n') {
                    lineAccum.trim();
                    if (lineAccum.startsWith("$")) {
                        lastNmeaLine = lineAccum;
                        lines++;
                        lastRx = millis();
                        Serial.printf("[TCP GPS] %s\n", lastNmeaLine.c_str());
                    }
                    lineAccum = "";
                } else if (c != '\r') {
                    lineAccum += c;
                    if (lineAccum.length() > 160) lineAccum = "";
                }
            }
        } else {
            if (!client.connected()) {
                Serial.println("[TCP GPS] server closed connection");
                client.stop();
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void startTask() {
    if (taskHandle) return;
    ensureMutex();
    xTaskCreate(netTask, "tcpGps", 4096, nullptr, 1, &taskHandle);
}

void stopTask() {
    if (!taskHandle) return;
    TaskHandle_t h = taskHandle;
    taskHandle = nullptr;
    vTaskDelete(h);
    vTaskDelay(pdMS_TO_TICKS(50));
}
} // namespace

bool isConnected() { return wantConnection && client.connected(); }

bool hasData() {
    if (!mutex) return false;
    bool has = false;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        has = ringCount > 0;
        xSemaphoreGive(mutex);
    }
    return has;
}

size_t available() {
    if (!mutex) return 0;
    size_t n = 0;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        n = ringCount;
        xSemaphoreGive(mutex);
    }
    return n;
}

int read() {
    if (!mutex) return -1;
    int out = -1;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (ringCount > 0) {
            out = ring[ringTail];
            ringTail = (ringTail + 1) % RING_SIZE;
            ringCount--;
        }
        xSemaphoreGive(mutex);
    }
    return out;
}

String host() { return hostStr; }
uint16_t port() { return portNum; }
unsigned long lastRxMs() { return lastRx; }
unsigned long lineCount() { return lines; }
unsigned long byteCount() { return bytes; }
String lastLine() { return lastNmeaLine; }

bool connect(const String &host, uint16_t port) {
    if (host.isEmpty() || port == 0) return false;
    if (!WiFi.isConnected()) return false;
    disconnect();
    hostStr = host;
    hostStr.trim();
    portNum = port;
    lines = 0;
    bytes = 0;
    lastRx = 0;
    lastNmeaLine = "";
    lineAccum = "";
    ensureMutex();
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        clearLocked();
        xSemaphoreGive(mutex);
    }
    Serial.printf("[TCP GPS] connecting %s:%u\n", hostStr.c_str(), portNum);
    if (!client.connect(hostStr.c_str(), portNum, CONNECT_TIMEOUT_MS)) {
        Serial.printf("[TCP GPS] connect failed %s:%u\n", hostStr.c_str(), portNum);
        return false;
    }
    wantConnection = true;
    gpsConnected = true;
    startTask();
    Serial.printf("[TCP GPS] connected %s:%u\n", hostStr.c_str(), portNum);
    return true;
}

void disconnect() {
    wantConnection = false;
    stopTask();
    client.stop();
    if (xSemaphoreTake(mutex ? mutex : (ensureMutex(), mutex), pdMS_TO_TICKS(100)) == pdTRUE) {
        clearLocked();
        xSemaphoreGive(mutex);
    }
    gpsConnected = false;
    Serial.println("[TCP GPS] disconnected");
}

String statusString() {
    String s = "TCP GPS\n";
    s += "Target: " + (hostStr.isEmpty() ? "-" : hostStr + ":" + String(portNum)) + "\n";
    s += "WiFi: " + String(WiFi.isConnected() ? WiFi.localIP().toString() : "down") + "\n";
    s += "TCP: " + String(isConnected() ? "connected" : "down") + "\n";
    s += "NMEA lines: " + String((unsigned long)lines) + "\n";
    s += "Bytes: " + String((unsigned long)bytes) + "\n";
    s += "Buffered: " + String((unsigned long)available()) + "\n";
    if (lastNmeaLine.isEmpty()) s += "Last: -\n";
    else s += "Last: " + lastNmeaLine.substring(0, 48) + "\n";
    s += "GPS icon: " + String(gpsConnected ? "on" : "off") + "\n";
    return s;
}
} // namespace TcpGps

void tcpGpsToolsMenu() {
    while (true) {
        bool on = TcpGps::isConnected();
        options = {
            {on ? "Disconnect" : "Connect", []() {
                 if (TcpGps::isConnected()) {
                     TcpGps::disconnect();
                     displayInfo("TCP GPS off", true);
                     return;
                 }
                 if (!WiFi.isConnected()) {
                     if (!wifiConnectMenu(WIFI_STA) || !WiFi.isConnected()) {
                         displayError("Wi-Fi connection failed", true);
                         return;
                     }
                 }
                 String h = keyboard("192.168.", 15, "GPS TCP host:");
                 if (h.length() == 0 || h == "\x1B") return;
                 h.trim();
                 String pStr = num_keyboard("10110", 5, "GPS TCP port:");
                 if (pStr.length() == 0 || pStr == "\x1B") return;
                 long p = pStr.toInt();
                 if (p < 1 || p > 65535) {
                     displayError("Invalid port", true);
                     return;
                 }
                 displayTextLine("Connecting...");
                 if (TcpGps::connect(h, static_cast<uint16_t>(p))) {
                     displaySuccess(("TCP GPS " + h + ":" + String(p)).c_str(), true);
                 } else {
                     displayError("TCP GPS failed", true);
                 }
             }},
            {"Status", []() { displayInfo(TcpGps::statusString(), true); }},
        };
        int sel = loopOptions(options, MENU_TYPE_SUBMENU, "TCP GPS");
        options.clear();
        if (sel < 0 || check(EscPress)) break;
    }
}
