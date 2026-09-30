#include <Preferences.h>
#include "OTAUpdater.h"
#include "NetworkManager.h"
#include "MLInference.h"

bool remote_debug_enabled = false;

#include <Preferences.h>
extern Preferences actPrefs;

extern unsigned long global_alarm_until;
#include "SensorManager.h"
extern SensorManager sensorMgr;
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

extern unsigned long identify_until;

WiFiClient espClient;
PubSubClient mqtt(espClient);
NetworkManager* instance = nullptr; 

void NetworkManager::begin(ConfigManager* configMgr) {
    _configMgr = configMgr;
    instance = this;
    
    // Ganti IP broker ke IP laptop user
    mqtt.setServer(_configMgr->config.mqtt_server, 1883);
    mqtt.setCallback(NetworkManager::mqttCallback);
}

void NetworkManager::loop() {
    // AUTO-RECONNECT WiFi jika terputus saat runtime
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long last_wifi_retry = 0;
        if (millis() - last_wifi_retry > 10000) { // Retry setiap 10 detik
            last_wifi_retry = millis();
            Serial.println("[WiFi] Koneksi terputus! Mencoba reconnect...");
            WiFi.disconnect();
            WiFi.begin(); // Gunakan kredensial tersimpan
        }
        return; // Jangan coba MQTT jika WiFi belum konek
    }
    
    if (!mqtt.connected()) {
        reconnectMQTT();
    }
    mqtt.loop();
}

void NetworkManager::reconnectMQTT() {
    if (millis() - _last_reconnect_attempt > 5000) {
        _last_reconnect_attempt = millis();
        
        char willTopic[64];
        snprintf(willTopic, sizeof(willTopic), "lindu/sensor/%s/status", _configMgr->config.node_id);
        
        char willPayload[128];
        snprintf(willPayload, sizeof(willPayload), "{\"status\":\"offline\",\"node_id\":\"%s\"}", _configMgr->config.node_id);
        
        if (mqtt.connect(_configMgr->config.node_id, willTopic, 1, true, willPayload)) {
            Serial.println("TERHUBUNG KE MQTT!");
            
            char statusPayload[512];
            snprintf(statusPayload, sizeof(statusPayload), 
                "{\"status\":\"online\",\"node_id\":\"%s\",\"lat\":%.4f,\"lon\":%.4f,\"pose\":\"%s\",\"tilt_angle\":%.1f,\"sensor_ok\":%s}",
                _configMgr->config.node_id, _configMgr->config.lat, _configMgr->config.lon, sensorMgr.getPose().c_str(), sensorMgr.getTiltAngle(), sensorMgr.sensor_ok ? "true" : "false");
            mqtt.publish(willTopic, statusPayload, true);
            
            // Subscribe ke topik sensor spesifik dan global
            String my_id = String(_configMgr->config.node_id);
            mqtt.subscribe("lindu/sensor/cmd/all", 1);
            mqtt.subscribe(("lindu/sensor/cmd/" + my_id).c_str(), 1);
            // Tetap subscribe ke topik lama demi kompatibilitas mundur jika ada
            mqtt.subscribe("lindu/actuator/cmd/all", 1);
        }
    }
}

void formatEpochMs(double epoch, char* out, size_t n) {
    long long ms = (long long)(epoch * 1000.0 + 0.5);
    snprintf(out, n, "%lld.%03d", ms / 1000, (int)(ms % 1000));
}

void NetworkManager::publishEvent(float pga, float sta_lta, int freq_hz, float ax, float ay, float az, unsigned long uptime_ms, double epoch, float lat, float lon, float temp, float pres, int gas_raw) {
    if (!mqtt.connected()) return;

    // ArduinoJson membulatkan double ke ~9 digit signifikan, sehingga epoch
    // (10 digit) kehilangan seluruh bagian sub-detiknya dan ~10 sampel pada
    // burst 10 Hz berbagi `ts` yang sama. Itu membuat dt = 0 di ekstraksi
    // fitur (energi dan durasi jadi nol). Tulis sebagai angka mentah dengan
    // milidetik; nama field tetap `ts` sehingga konsumen lama tidak berubah.
    char ts_raw[24];
    formatEpochMs(epoch, ts_raw, sizeof(ts_raw));

    StaticJsonDocument<512> doc;
    doc["node_id"] = _configMgr->config.node_id;
    doc["ts"]      = serialized(ts_raw);
    doc["lat"]     = lat;
    doc["lon"]     = lon;
    doc["pga"]     = pga;
    doc["sta_lta"] = sta_lta;
    doc["freq_hz"] = freq_hz;
    doc["ax"]      = ax;
    doc["ay"]      = ay;
    doc["az"]      = az;
    doc["valve_status"] = is_valve_locked ? "DISABLED" : "ENABLED";
#if !ARDUINO_USB_CDC_ON_BOOT
    doc["door_status"] = is_door_locked ? "LOCKED" : "UNLOCKED";
#else
    doc["door_status"] = "UNAVAILABLE";
#endif
    doc["gas_alert"] = sensorMgr.gas_leak_detected;
    doc["gas_raw"] = gas_raw;
    doc["temperature"] = temp;
    doc["pressure"] = pres;
    
    // Data Cuaca (Hanya diisi jika valid)
    if (temp > 0.0) doc["temperature"] = temp;
    if (pres > 0.0) doc["pressure"] = pres;

    char buffer[512];
    serializeJson(doc, buffer);
    
    char topic[64];
    snprintf(topic, sizeof(topic), "lindu/sensor/%s/telemetry", _configMgr->config.node_id);
    mqtt.publish(topic, buffer);
}

float NetworkManager::haversine(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * PI / 180.0;
    float dLon = (lon2 - lon1) * PI / 180.0;
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    float a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    float c = 2 * asin(sqrt(a));
    return 6371.0 * c;
}

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++) msg += (char)payload[i];

    if (String(topic) == "lindu/actuator/cmd/all") {
        StaticJsonDocument<1024> doc;
        DeserializationError error = deserializeJson(doc, msg);
        if (error) return;
        
        if (doc["cmd"] == "trigger_siren") {
            float e_lat = doc["epicenter_lat"];
            float e_lon = doc["epicenter_lon"];
            
            float dist = instance->haversine(
                instance->_configMgr->config.lat, 
                instance->_configMgr->config.lon, 
                e_lat, e_lon
            );
            
            bool is_bypass = doc["bypass"] | false;
            
            if (dist < 50.0 || is_bypass) {
#if !ARDUINO_USB_CDC_ON_BOOT
                Serial.println("[!] SIRINE MENYALA! Valve Dikunci Tutup. Pintu tetap mengikuti occupancy PIR (unlock hanya jika ada orang terdeteksi 10 menit terakhir).");
#else
                Serial.println("[!] SIRINE MENYALA! Valve Dikunci Tutup!");
#endif
                global_alarm_until = millis() + 15000;
                is_valve_locked = true;
                actPrefs.putBool("valve_locked", true);
                // Door TIDAK dipaksa buka di sini - status pintu selalu murni mengikuti
                // occupancy PIR (lihat main.cpp), termasuk saat alarm gempa aktif.
            } else {
                Serial.println("[i] Epicenter terlalu jauh. Abaikan.");
            }
        } else if (doc["cmd"] == "cancel_alarm") {
            // Dipakai dashboard saat tombol "Abaikan Peringatan" ditekan. Hard reset
            // semua aktuator ke kondisi normal: valve terbuka, pintu terkunci - terlepas
            // dari occupancy PIR sesaat sebelum ini. Harus mematikan global_alarm_until
            // SEKARANG JUGA agar loop occupancy di main.cpp (yang hanya jalan selama
            // is_global_alarm true) langsung berhenti dan tidak menimpa balik is_door_locked.
            Serial.println("[i] Perintah Sistem: ALARM DIBATALKAN. Mengembalikan aktuator ke kondisi normal.");
            global_alarm_until = 0;
            is_valve_locked = false;
            actPrefs.putBool("valve_locked", false);
#if !ARDUINO_USB_CDC_ON_BOOT
            is_door_locked = true;
#endif
        } else if (doc["cmd"] == "enable_valve") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Valve di-ENABLE (Aliran Dibuka).");
                is_valve_locked = false;
                actPrefs.putBool("valve_locked", false);
            } else {
                Serial.println("[i] Perintah Enable diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "disable_valve") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Valve di-DISABLE (Aliran Ditutup).");
                is_valve_locked = true;
                actPrefs.putBool("valve_locked", true);
            } else {
                Serial.println("[i] Perintah Disable diabaikan (Bukan untuk Node ini).");
            }
#if !ARDUINO_USB_CDC_ON_BOOT
        // Command door lock: khusus unit ESP32 classic (esp32_wroom)
        } else if (doc["cmd"] == "lock_door") {
            // Berlaku penuh di luar alarm gempa. Selama alarm gempa aktif, loop occupancy
            // di main.cpp mengambil alih is_door_locked berdasarkan deteksi PIR 10 menit
            // terakhir, jadi command manual ini bisa ditimpa balik saat itu.
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Pintu di-LOCK.");
                is_door_locked = true;
            } else {
                Serial.println("[i] Perintah Lock Door diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "unlock_door") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Pintu di-UNLOCK.");
                is_door_locked = false;
            } else {
                Serial.println("[i] Perintah Unlock Door diabaikan (Bukan untuk Node ini).");
            }
        } else if (doc["cmd"] == "reset_pir") {
            // Tombol debug di Grafana: reset nilai sensor PIR, node dianggap TIDAK ADA
            // gerakan/manusia sama sekali dalam 10 menit terakhir. Berguna untuk testing
            // behavior door-lock saat alarm gempa tanpa perlu menunggu window 10 menit
            // habis secara alami, atau saat sensor PIR fisik belum terpasang.
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: Sensor PIR di-RESET (dianggap sepi 10 menit terakhir).");
                last_motion_ms = 0;
            } else {
                Serial.println("[i] Perintah Reset PIR diabaikan (Bukan untuk Node ini).");
            }
#endif
        } else if (doc["cmd"] == "identify") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                identify_until = millis() + 10000; // 10 detik
                Serial.println("[i] Perintah Sistem: IDENTIFY. Lampu berkedip putih.");
            }
        } else if (doc["cmd"] == "set_location") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                if (doc.containsKey("lat") && doc.containsKey("lon")) {
                    instance->_configMgr->config.lat = doc["lat"];
                    instance->_configMgr->config.lon = doc["lon"];
                    instance->_configMgr->saveConfig();
                    Serial.println("[i] Perintah Sistem: SET LOCATION. Koordinat diperbarui!");
                    instance->forcePublishStatus(); // Segera kirim update ke dashboard
                }
            }
        } else if (doc["cmd"] == "factory_reset") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[!] Perintah Sistem: FACTORY RESET. Menghapus semua memori dan Restart...");
                instance->_configMgr->resetConfig(); // Hapus WiFi & Koordinat
                Preferences prefs;
                prefs.begin("ota", false);
                prefs.clear(); // Hapus blacklist OTA
                prefs.end();
                delay(1000);
                ESP.restart();
            }
        } else if (doc["cmd"] == "set_broker") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                if (doc.containsKey("server")) {
                    strlcpy(instance->_configMgr->config.mqtt_server, (const char*)doc["server"], 64);
                    instance->_configMgr->saveConfig();
                    Serial.println("[i] Perintah Sistem: SET BROKER. Restarting ESP32...");
                    delay(1000);
                    ESP.restart();
                }
            }
        } else if (doc["cmd"] == "ml_selftest") {
            // Bukti kesetaraan ekstraksi fitur firmware vs server. Node
            // menerbitkan vektor fitur atas jendela 2 detik terakhirnya beserta
            // rentang waktu jendela; server mencocokkannya dengan hasil
            // ekstraksinya sendiri atas telemetri yang sama.
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                double t_first = 0, t_last = 0;
                int n_samples = 0;
                if (!mlInference.isReady() || !mlInference.selfTest(&t_first, &t_last, &n_samples)) {
                    Serial.println("[ML] ml_selftest dilewati: model tidak siap atau buffer kosong");
                } else {
                    float feat[32];
                    int nf = mlInference.lastFeatures(feat, 32);
                    char tf[24], tl[24];
                    formatEpochMs(t_first, tf, sizeof(tf));
                    formatEpochMs(t_last, tl, sizeof(tl));
                    char buf[768];
                    int len = snprintf(buf, sizeof(buf),
                        "{\"node_id\":\"%s\",\"model\":\"%s\",\"t_first\":%s,\"t_last\":%s,\"n_samples\":%d,\"features\":[",
                        my_id.c_str(), mlInference.modelVersion(), tf, tl, n_samples);
                    for (int i = 0; i < nf && len < (int)sizeof(buf) - 24; i++) {
                        len += snprintf(buf + len, sizeof(buf) - len, "%s%.9g", i ? "," : "", feat[i]);
                    }
                    snprintf(buf + len, sizeof(buf) - len, "]}");
                    char topic[64];
                    snprintf(topic, sizeof(topic), "lindu/sensor/%s/ml_selftest", my_id.c_str());
                    mqtt.publish(topic, buf);
                    Serial.printf("[ML] ml_selftest diterbitkan: %d fitur, %d sampel\n", nf, n_samples);
                }
            }
        } else if (doc["cmd"] == "force_update") {
            // Hanya memicu cek OTA. Tidak me-restart: node yang sudah di versi
            // terbaru tidak perlu mati ~30 detik, dan restart berulang dari
            // tombol dashboard memutus unduhan yang sedang berjalan. Node
            // sendiri yang restart setelah pembaruan berhasil dipasang.
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: FORCE UPDATE. Blacklist dihapus, memeriksa rilis terbaru...");
                otaUpdater.requestCheck();
            }
        } else if (doc["cmd"] == "reboot") {
            String target = doc["target_node"] | "all";
            String my_id = String(instance->_configMgr->config.node_id);
            if (target == "all" || target == my_id) {
                Serial.println("[i] Perintah Sistem: REBOOT. Menghapus Blacklist dan Restarting ESP32...");
                Preferences prefs;
                prefs.begin("ota", false);
                prefs.remove("failed_tag");
                prefs.end();
                delay(1000);
                ESP.restart();
            }
        } else if (doc["cmd"] != "ALARM_ON" && doc["cmd"] != "ALARM_UPDATE") {
            // Perintah yang tidak dikenali dicatat, bukan dibuang diam-diam.
            // (ALARM_* dikirim server ke topik yang sama dan ditangani di
            // tempat lain, jadi tidak perlu dicatat di sini.)
            Serial.printf("[i] Perintah tidak dikenal: %s\n", doc["cmd"] | "(tanpa cmd)");
        }
    }
}

bool NetworkManager::isConnected() {
    return mqtt.connected();
}

void NetworkManager::forcePublishStatus() {
    publishStatus("online", true, 0.0, "FLAT");
}

void NetworkManager::publishStatus(String status, bool sensor_ok, float tilt_angle, String pose, bool motion_detected) {
    if (!mqtt.connected()) return;
    char willTopic[64];
    snprintf(willTopic, sizeof(willTopic), "lindu/sensor/%s/status", _configMgr->config.node_id);

    char statusPayload[512];
    snprintf(statusPayload, sizeof(statusPayload),
        "{\"status\":\"%s\",\"node_id\":\"%s\",\"lat\":%.4f,\"lon\":%.4f,\"pose\":\"%s\",\"tilt_angle\":%.1f,\"sensor_ok\":%s,\"fw_version\":\"%s\",\"ota_status\":\"%s\",\"motion_detected\":%s,\"ml_model\":\"%s\"}",
        status.c_str(), _configMgr->config.node_id, _configMgr->config.lat, _configMgr->config.lon, pose.c_str(), tilt_angle, sensor_ok ? "true" : "false", CURRENT_VERSION, otaUpdater.ota_status.c_str(), motion_detected ? "true" : "false",
        mlInference.isReady() ? mlInference.modelVersion() : "none");

    mqtt.publish(willTopic, statusPayload, true);
}

void NetworkManager::publishLog(const char* message) {
    if (!mqtt.connected()) return;
    if (!remote_debug_enabled) return; // Hanya kirim log jika user menekan ENABLE DEBUG di Grafana

    char topic[64];
    snprintf(topic, sizeof(topic), "lindu/sensor/%s/log", _configMgr->config.node_id);
    
    char payload[256];
    snprintf(payload, sizeof(payload), "{\"message\":\"%s\"}", message);
    
    mqtt.publish(topic, payload); // Menggunakan overload string (otomatis QoS 0, retain false)
}
