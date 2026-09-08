#pragma once
#include <Arduino.h>

namespace TcpGps {
bool isConnected();
bool hasData();
size_t available();
int read();
String host();
uint16_t port();
unsigned long lastRxMs();
unsigned long lineCount();
unsigned long byteCount();
String lastLine();
bool connect(const String &host, uint16_t port);
void disconnect();
String statusString();
}

void tcpGpsToolsMenu();
