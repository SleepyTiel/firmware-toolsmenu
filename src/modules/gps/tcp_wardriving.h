#pragma once

#include "modules/ble/ble_common.h"
#include <TinyGPS++.h>
#include <cstdint>
#include <esp_wifi_types.h>
#include <globals.h>
#include <set>

class TcpWardriving {
public:
    TcpWardriving(bool scanWiFi = false, bool scanBLE = false);
    ~TcpWardriving();

    void setup();
    void loop();

private:
    bool date_time_updated = false;
    bool initial_position_set = false;
    double cur_lat;
    double cur_lng;
    double distance = 0;
    uint32_t sessionStartMs = 0;
    String filename = "";
    TinyGPSPlus gps;
    std::set<uint64_t> registeredMACs;
    std::set<String> alertMACs;
    bool scanWiFi = false;
    bool scanBLE = false;
    bool bleInitialized = false;
    int wifiNetworkCount = 0;
    int bluetoothDeviceCount = 0;
    int foundMACAddressCount = 0;
    uint32_t macCacheClears = 0;
    static constexpr size_t MAX_REGISTERED_MACS = 250;

    void begin_wifi(void);
    bool begin_gps(void);
    void end(void);

    void display_banner(void);
    void dump_gps_data(void);

    void set_position(void);
    void scanWiFiBLE(void);
    int scanWiFiNetworks(void);
    void enforceRegisteredMACLimit(void);
    void loadAlertMACs(void);
    void checkForAlert(const String &macAddress, const String &deviceType, const String &deviceName = "");
    String auth_mode_to_string(wifi_auth_mode_t authMode);
    void create_filename(void);
};
