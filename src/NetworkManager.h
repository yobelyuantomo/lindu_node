#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include "ConfigManager.h"

class NetworkManager {
public:
    void begin(ConfigManager* configMgr);
    void loop();
    bool isConnected();
    void forcePublishStatus();
    void publishStatus(String status, bool sensor_ok, float tilt_angle, String pose, bool motion_detected = false);
    void publishEvent(float pga, float sta_lta, int freq_hz, float ax, float ay, float az, unsigned long uptime_ms, double epoch, float lat, float lon, float temp, float pres, int gas_raw);
    void publishLog(const char* message);
    
private:
    ConfigManager* _configMgr;
    
    void reconnectMQTT();
    
    unsigned long _last_reconnect_attempt = 0; 
    
    float haversine(float lat1, float lon1, float lat2, float lon2);
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
};

#endif


extern unsigned long global_alarm_until;
extern bool is_valve_locked;
#if !ARDUINO_USB_CDC_ON_BOOT
extern bool is_door_locked; // Khusus unit ESP32 classic (esp32_wroom)
extern unsigned long last_motion_ms; // Khusus unit ESP32 classic - timestamp deteksi PIR terakhir
#endif
