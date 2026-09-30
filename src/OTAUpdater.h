#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "esp_system.h"

#ifndef CURRENT_VERSION
#define CURRENT_VERSION "v1.3.6"
#endif

class OTAUpdater {
public:
    void begin();
    void loop();
    void checkForUpdate();
    void confirmWorking();

    // Minta cek OTA segera tanpa me-restart node. Dipakai perintah
    // `force_update` dari dashboard. Sekaligus menghapus penanda versi
    // gagal (blacklist) supaya versi itu boleh dicoba lagi.
    void requestCheck();

    String ota_status = "IDLE";

private:
    unsigned long _last_check = 0;
    volatile bool _force_check = false;
    Preferences _prefs;
    String _failed_tag;

    // Alasan kegagalan terakhir, dicatat ke ota_status supaya terlihat di
    // dashboard: http_<kode>, update_begin, tulis_flash, timeout,
    // koneksi_putus, atau validasi.
    String _last_error;

    bool performUpdate(const char* url, const char* tag);
    bool downloadWithRetry(const char* url, const char* tag);
};

extern OTAUpdater otaUpdater;
#endif
