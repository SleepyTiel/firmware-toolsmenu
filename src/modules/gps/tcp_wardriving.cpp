#include "tcp_wardriving.h"
#include "tcp_gps.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/wifi/wifi_common.h"
#include "current_year.h"
#include "modules/ble/ble_common.h"
#include <cctype>

#define MAX_WAIT 5000

static bool parseMacToU64(const String &mac, uint64_t &out) {
    uint64_t value = 0;
    int nibbles = 0;
    for (size_t i = 0; i < mac.length(); i++) {
        char c = mac[i];
        if (c == ':' || c == '-') continue;
        if (!isxdigit(static_cast<unsigned char>(c))) return false;
        value <<= 4;
        if (c >= '0' && c <= '9') value |= static_cast<uint64_t>(c - '0');
        else if (c >= 'a' && c <= 'f') value |= static_cast<uint64_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') value |= static_cast<uint64_t>(c - 'A' + 10);
        else return false;
        nibbles++;
        if (nibbles > 12) return false;
    }
    if (nibbles != 12) return false;
    out = value;
    return true;
}

TcpWardriving::TcpWardriving(bool scanWiFi, bool scanBLE) {
    this->scanWiFi = scanWiFi;
    this->scanBLE = scanBLE;
    setup();
}

TcpWardriving::~TcpWardriving() {
    returnToMenu = true;
}

void TcpWardriving::setup() {
    wifiNetworkCount = 0;
    bluetoothDeviceCount = 0;
    display_banner();
    padprintln("Initializing...");

    loadAlertMACs();
    begin_wifi();
    if (!begin_gps()) return;

    sessionStartMs = millis();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return loop();
}

void TcpWardriving::begin_wifi() {
    if (!WiFi.isConnected()) wifiConnectMenu(WIFI_STA);
}

bool TcpWardriving::begin_gps() {
    if (!TcpGps::isConnected()) {
        padprintln("No TCP GPS link");
        padprintln("Tools -> TCP GPS first");
        displayError("No TCP GPS link", true);
        return false;
    }

    int count = 0;
    padprintln("Waiting for TCP NMEA");
    while (TcpGps::available() <= 0) {
        if (check(EscPress)) return false;
        if (!TcpGps::isConnected()) {
            displayError("TCP GPS lost", true);
            return false;
        }
        displayTextLine("Waiting TCP GPS: " + String(count) + "s");
        Serial.printf("[TCP Wardriving] waiting %ds buffered=%u\n", count, (unsigned)TcpGps::available());
        count++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    gpsConnected = true;
    Serial.println("[TCP Wardriving] NMEA stream detected");
    return true;
}

void TcpWardriving::end() {
    if (scanBLE) {
        BLEDevice::deinit(true);
        pBLEScan = nullptr;
        bleInitialized = false;
    }
    returnToMenu = true;
}

void TcpWardriving::loop() {
    int count = 0;
    returnToMenu = false;
    while (1) {
        display_banner();

        if (TcpGps::available() > 0) {
            count = 0;
            while (TcpGps::available() > 0) {
                int c = TcpGps::read();
                if (c < 0) break;
                gps.encode((char)c);
            }
            if (gps.location.isUpdated()) {
                padprintln("GPS location updated");
                set_position();
                scanWiFiBLE();
            } else {
                padprintln("GPS location not updated");
                dump_gps_data();

                if (filename == "" && gps.date.year() >= CURRENT_YEAR && gps.date.year() < CURRENT_YEAR + 5)
                    create_filename();
            }
        } else {
            if (!TcpGps::isConnected()) {
                displayError("TCP GPS lost!");
                return end();
            }
            if (count > 5) {
                displayError("GPS not Found!");
                return end();
            }
            padprintln("No GPS data available");
            count++;
        }

        unsigned long tmp = millis();
        while (millis() - tmp < MAX_WAIT && !gps.location.isUpdated()) {
            if (check(EscPress) || returnToMenu) return end();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void TcpWardriving::set_position() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    if (initial_position_set) distance += gps.distanceBetween(cur_lat, cur_lng, lat, lng);
    else initial_position_set = true;

    cur_lat = lat;
    cur_lng = lng;
}

void TcpWardriving::display_banner() {
    drawMainBorderWithTitle("TCP Wardriving");

    padprintln("");
    if (filename != "") padprintln("File: " + filename.substring(0, filename.length() - 4));
    String txt = "Found";
    if (scanWiFi) txt += " WiFi: " + String(wifiNetworkCount);
    if (scanBLE) txt += " BLE: " + String(bluetoothDeviceCount);
    padprint(txt);
    if (foundMACAddressCount) padprint(" Alert: " + String(foundMACAddressCount));

    padprintln("");
    uint32_t elapsedMs = millis() - sessionStartMs;
    uint32_t elapsedSeconds = elapsedMs / 1000;
    uint32_t hours = elapsedSeconds / 3600;
    uint32_t minutes = (elapsedSeconds / 60) % 60;
    uint32_t seconds = elapsedSeconds % 60;
    padprintf("Distance: %.2fkm  ET: %02lu:%02lu:%02lu\n", distance / 1000, hours, minutes, seconds);
}

void TcpWardriving::dump_gps_data() {
    if (!date_time_updated && (!gps.date.isUpdated() || !gps.time.isUpdated())) {
        padprintln("Waiting for valid GPS data");
        return;
    }
    date_time_updated = true;
    padprintf(2, "Date: %02d-%02d-%02d\n", gps.date.year(), gps.date.month(), gps.date.day());
    padprintf(2, "Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    padprintf(2, "Sat:  %d\n", gps.satellites.value());
    padprintf(2, "HDOP: %.2f\n", gps.hdop.hdop());
}

String TcpWardriving::auth_mode_to_string(wifi_auth_mode_t authMode) {
    switch (authMode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
        default: return "UNKNOWN";
    }
}

void TcpWardriving::scanWiFiBLE() {
    FS *fs;
    if (!getFsStorage(fs)) {
        padprintln("Storage setup error");
        displayError("Storage setup error", true);
        returnToMenu = true;
        return;
    }

    if (filename == "") create_filename();

    if (!(*fs).exists("/BruceWardriving")) (*fs).mkdir("/BruceWardriving");

    bool is_new_file = false;
    if (!(*fs).exists("/BruceWardriving/" + filename)) is_new_file = true;
    File file = (*fs).open("/BruceWardriving/" + filename, is_new_file ? FILE_WRITE : FILE_APPEND);

    if (!file) {
        padprintln("Failed to open file for writing");
        displayError("Failed to open file for writing", true);
        returnToMenu = true;
        return;
    }

    if (is_new_file) {
        file.println(
            "WigleWifi-1.6,appRelease=v" + String(BRUCE_VERSION) + ",model=M5Stack GPS Unit,release=v" +
            String(BRUCE_VERSION) +
            ",device=ESP32 M5Stack,display=SPI TFT,board=ESP32 M5Stack,brand=Bruce,star=Sol,body=4,subBody=1"
        );
        file.println(
            "MAC,SSID,AuthMode,FirstSeen,Channel,Frequency,RSSI,CurrentLatitude,CurrentLongitude,"
            "AltitudeMeters,AccuracyMeters,RCOIs,MfgrId,Type"
        );
    }

    padprintf("Coord: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
    padprintln("Start Scanning...");

    if (scanBLE && pBLEScan != nullptr && pBLEScan->isScanning()) {
        pBLEScan->stop();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int networksFound = scanWiFi ? scanWiFiNetworks() : 0;
    int bleFound = 0;
    if (networksFound > 0) {
        for (int i = 0; i < networksFound; i++) {
            String macAddress = WiFi.BSSIDstr(i);
            uint64_t macKey = 0;
            bool macKeyOk = parseMacToU64(macAddress, macKey);

            enforceRegisteredMACLimit();
            if (!macKeyOk || registeredMACs.find(macKey) == registeredMACs.end()) {

                if (macKeyOk) registeredMACs.insert(macKey);
                int32_t channel = WiFi.channel(i);

                char buffer[512];
                snprintf(
                    buffer,
                    sizeof(buffer),
                    "%s,\"%s\",[%s],%04d-%02d-%02d %02d:%02d:%02d,%ld,%ld,%ld,%f,%f,%f,%f,,,WIFI\n",
                    macAddress.c_str(),
                    WiFi.SSID(i).c_str(),
                    auth_mode_to_string(WiFi.encryptionType(i)).c_str(),
                    gps.date.year(),
                    gps.date.month(),
                    gps.date.day(),
                    gps.time.hour(),
                    gps.time.minute(),
                    gps.time.second(),
                    channel,
                    channel != 14 ? 2407 + (channel * 5) : 2484,
                    WiFi.RSSI(i),
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.meters(),
                    gps.hdop.hdop() * 1.0
                );
                file.print(buffer);

                checkForAlert(macAddress, "WiFi", WiFi.SSID(i));

                wifiNetworkCount++;
            }

            if ((i & 0x1F) == 0) vTaskDelay(1);
        }
    }
    if (scanWiFi) {
        WiFi.scanDelete();
        vTaskDelay(120 / portTICK_PERIOD_MS);
    }

    if (scanBLE) {
        if (!bleInitialized || pBLEScan == nullptr) {
            if (!BLEDevice::init("")) {
                Serial.println(" Failed to init BLE");
                file.close();
                vTaskDelay(500 / portTICK_PERIOD_MS);
                return;
            }
            pBLEScan = BLEDevice::getScan();
            pBLEScan->setActiveScan(true);
            pBLEScan->setInterval(SCAN_INT);
            pBLEScan->setWindow(SCAN_WINDOW);
            bleInitialized = true;
        }
        if (pBLEScan->isScanning()) {
            pBLEScan->stop();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        BLEScanResults foundDevices;

        foundDevices = pBLEScan->getResults(scanTime * 1000, false);

        int count = foundDevices.getCount();
        bleFound = count;
        if (count == 0) {
            pBLEScan->clearResults();
            vTaskDelay(150 / portTICK_PERIOD_MS);
            goto scan_summary;
        }

        int deviceIndex = 0;
        for (int i = 0; i < count; i++) {
            const NimBLEAdvertisedDevice *device = foundDevices.getDevice(i);
            if (!device) continue;

            String address;
            String name;
            int rssi = 0;
            uint16_t manufacturerId = 0;

            try {
                address = device->getAddress().toString().c_str();
                name = device->getName().c_str();
                rssi = device->getRSSI();

                if (device->haveManufacturerData()) {
                    std::string mfgData = device->getManufacturerData();
                    if (!mfgData.empty() && mfgData.length() >= 2) {
                        manufacturerId = (uint16_t(mfgData[1]) << 8) | uint16_t(mfgData[0]);
                    }
                }
            } catch (...) { continue; }

            enforceRegisteredMACLimit();
            uint64_t macKey = 0;
            bool macKeyOk = parseMacToU64(address, macKey);
            if (!macKeyOk || registeredMACs.find(macKey) == registeredMACs.end()) {
                if (macKeyOk) registeredMACs.insert(macKey);

                char buffer[512];
                char manufacturerIdStr[8] = "";
                if (manufacturerId != 0) {
                    snprintf(manufacturerIdStr, sizeof(manufacturerIdStr), "%04X", manufacturerId);
                }
                snprintf(
                    buffer,
                    sizeof(buffer),
                    "%s,\"%s\",Misc [BLE],%04d-%02d-%02d %02d:%02d:%02d,0,,%d,%f,%f,%f,%f,,%s,BLE\n",
                    address.c_str(),
                    name.c_str(),
                    gps.date.year(),
                    gps.date.month(),
                    gps.date.day(),
                    gps.time.hour(),
                    gps.time.minute(),
                    gps.time.second(),
                    rssi,
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.meters(),
                    gps.hdop.hdop() * 1.0,
                    manufacturerIdStr
                );
                file.print(buffer);

                checkForAlert(address, "BLE", name);

                bluetoothDeviceCount++;
            }

            if ((deviceIndex++ & 0x1F) == 0) vTaskDelay(1);
        }

        pBLEScan->clearResults();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

scan_summary:
    if (scanWiFi || scanBLE) {
        String summary = "Scan done.";
        if (scanWiFi) summary += " WiFi: " + String(networksFound);
        if (scanBLE) summary += " BLE: " + String(bleFound);
        padprintln(summary);
    }

    file.close();
}

void TcpWardriving::enforceRegisteredMACLimit() {
    if (registeredMACs.size() < MAX_REGISTERED_MACS) return;

    registeredMACs.clear();
    macCacheClears++;
    padprintln("MAC cache cleared to prevent heap overflow");
}

int TcpWardriving::scanWiFiNetworks() {
    wifiConnected = true;
    int network_amount = WiFi.scanNetworks();
    return network_amount;
}

void TcpWardriving::loadAlertMACs() {
    FS *fs;
    if (!getFsStorage(fs)) return;

    if (!(*fs).exists("/BruceWardriving")) (*fs).mkdir("/BruceWardriving");

    if ((*fs).exists("/BruceWardriving/alert.txt")) {
        File alertFile = (*fs).open("/BruceWardriving/alert.txt", FILE_READ);
        if (alertFile) {
            while (alertFile.available()) {
                String line = alertFile.readStringUntil('\n');
                line.trim();
                if (line.length() > 0 && !line.startsWith("#")) {
                    line.toLowerCase();
                    alertMACs.insert(line);
                }
            }
            alertFile.close();
            if (alertMACs.size() > 0) { padprintln("Loaded " + String(alertMACs.size()) + " alert MACs"); }
        }
    } else {
        File alertFile = (*fs).open("/BruceWardriving/alert.txt", FILE_WRITE);
        if (alertFile) {
            alertFile.println("# Alert MAC addresses - one per line");
            alertFile.println("# Lines starting with # are comments");
            alertFile.println("# Example:");
            alertFile.println("# aa:bb:cc:dd:ee:ff");
            alertFile.close();
        }
    }
}

void TcpWardriving::create_filename() {
    char timestamp[20];
    sprintf(
        timestamp,
        "%02d%02d%02d_%02d%02d%02d",
        gps.date.year() % 100,
        gps.date.month() % 100,
        gps.date.day() % 100,
        gps.time.hour() % 100,
        gps.time.minute() % 100,
        gps.time.second() % 100
    );
    filename = String(timestamp) + "_wardriving.csv";
}

void TcpWardriving::checkForAlert(const String &macAddress, const String &deviceType, const String &deviceName) {
    String macLower = macAddress;
    macLower.toLowerCase();

    if (alertMACs.find(macLower) != alertMACs.end()) {
        String alertMsg = "ALERT: " + deviceType + " found!";
        if (deviceName.length() > 0) { alertMsg += " Name: " + deviceName; }
        alertMsg += " MAC: " + macAddress;

        foundMACAddressCount++;

        displayError(alertMsg.c_str());

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
