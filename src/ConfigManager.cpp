#include "ConfigManager.h"
#include <WiFiManager.h>
#include <Preferences.h>

Preferences prefs;

void ConfigManager::begin() {
    prefs.begin("lindu", false);
    loadConfig();
}

void ConfigManager::loadConfig() {
    // ponytail: fallback ke MAC address jika belum ada ID
    String defaultId = "node_" + String((uint32_t)(ESP.getEfuseMac() >> 16), HEX);
    String id = prefs.getString("node_id", defaultId);
    strlcpy(config.node_id, id.c_str(), sizeof(config.node_id));
    
    config.lat = prefs.getFloat("lat", 0.0);
    config.lon = prefs.getFloat("lon", 0.0);
    
    String srv = prefs.getString("mqtt_srv", "192.168.68.105");
    strlcpy(config.mqtt_server, srv.c_str(), sizeof(config.mqtt_server));
    config.is_provisioned = (config.lat != 0.0);
    String repo = prefs.getString("ota_repo", "yobelyuantomo/lindu_node");
    strlcpy(config.ota_repo, repo.c_str(), sizeof(config.ota_repo));

}

void ConfigManager::saveConfig() {
    prefs.putString("node_id", config.node_id);
    prefs.putFloat("lat", config.lat);
    prefs.putFloat("lon", config.lon);
    prefs.putString("mqtt_srv", config.mqtt_server);
    prefs.putString("ota_repo", config.ota_repo);
}

void ConfigManager::resetConfig() {
    prefs.clear();
    WiFi.disconnect(true, true);
    ESP.restart();
}

bool ConfigManager::startCaptivePortal() {
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    
    // --- TAMBAHAN HTML KUSTOM UNTUK PANDUAN HARDWARE ---
    static String html = ""; // Gunakan static agar pointer c_str() tetap valid di memory
    html = "<div style='background-color:#E8F8F5; padding:10px; border-left:4px solid #1ABC9C; margin-bottom:15px; text-align:left;'>";
    html += "<h3 style='margin-top:0;'>📌 Panduan Hardware Lindu-EEW</h3>";
    html += "<p style='font-size:12px;'>Pastikan komponen terpasang sesuai pin berikut sebelum alat beroperasi:</p>";
    html += "<table border='1' style='width:100%; text-align:left; border-collapse:collapse; font-size:12px; background:white;'>";
    html += "<tr style='background:#1ABC9C; color:white;'><th>Modul/Sensor</th><th>Pinout Alat Ini</th></tr>";

#if ARDUINO_USB_CDC_ON_BOOT
    // ESP32-S3 Variant
    html += "<tr><td>Sensor Getar (LSM6DS3)</td><td>SDA: <b>10</b>, SCL: <b>11</b></td></tr>";
    html += "<tr><td>Sensor Suhu/Cuaca (BME280)</td><td>SDA: <b>10</b>, SCL: <b>11</b></td></tr>";
    html += "<tr><td>Lampu Status (WS2812)</td><td>Pin <b>48</b></td></tr>";
    html += "<tr><td>Buzzer Sirine (Aktif Low)</td><td>Pin <b>6</b></td></tr>";
    html += "<tr><td>Motor Servo Katup Gas</td><td>Pin <b>5</b></td></tr>";
#else
    // ESP32 Classic/WROOM Variant
    html += "<tr><td>Sensor Getar (LSM6DS3)</td><td>SDA: <b>18</b>, SCL: <b>19</b></td></tr>";
    html += "<tr><td>Sensor Suhu/Cuaca (BME280)</td><td>SDA: <b>21</b>, SCL: <b>22</b></td></tr>";
    html += "<tr><td>Sensor Gas Bocor (MQ-2)</td><td>Analog <b>34</b></td></tr>";
    html += "<tr><td>Lampu Status (WS2812)</td><td>Pin <b>4</b></td></tr>";
    html += "<tr><td>Buzzer Sirine (Aktif Low)</td><td>Pin <b>14</b></td></tr>";
    html += "<tr><td>Solenoid Katup Pipa Air</td><td>Relay 1 (Pin <b>26</b>)</td></tr>";
    html += "<tr><td>Solenoid Kunci Pintu</td><td>Relay 2 (Pin <b>25</b>)</td></tr>";
#endif

    html += "</table>";
    html += "<p style='font-size:11px; margin-bottom:0;'><i>Device ID: <b>" + String(config.node_id) + "</b></i></p>";
    html += "</div>";
    html += "<hr>";
    
    WiFiManagerParameter custom_html(html.c_str());
    wm.addParameter(&custom_html);
    // ----------------------------------------------------

    // Custom HTML params (Labels yang diperjelas)
    char latStr[16]; dtostrf(config.lat, 4, 6, latStr);
    char lonStr[16]; dtostrf(config.lon, 4, 6, lonStr);
    
    WiFiManagerParameter custom_mqtt("mqtt", "Alamat IP Server MQTT", config.mqtt_server, 64);
    WiFiManagerParameter custom_lat("lat", "Koordinat GPS Latitude", latStr, 16);
    WiFiManagerParameter custom_lon("lon", "Koordinat GPS Longitude", lonStr, 16);
    WiFiManagerParameter custom_repo("repo", "Repositori OTA (User/Repo)", config.ota_repo, 64);
    
    wm.addParameter(&custom_mqtt);
    wm.addParameter(&custom_lat);
    wm.addParameter(&custom_lon);
    wm.addParameter(&custom_repo);

    // ponytail: blocking 3 menit, jika gagal biarkan Watchdog mereset
    wm.setConfigPortalTimeout(180); 

    if (!wm.autoConnect(config.node_id)) {
        return false;
    }

    // Save params jika diupdate via portal
    config.lat = atof(custom_lat.getValue());
    config.lon = atof(custom_lon.getValue());
    strlcpy(config.mqtt_server, custom_mqtt.getValue(), sizeof(config.mqtt_server));
    strlcpy(config.ota_repo, custom_repo.getValue(), sizeof(config.ota_repo));
    saveConfig();

    return true;
}
