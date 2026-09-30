#include <esp_task_wdt.h>
#include "OTAUpdater.h"
#include "ConfigManager.h"
extern ConfigManager configMgr;
#include "NetworkManager.h"
extern NetworkManager networkMgr;

OTAUpdater otaUpdater;

// GitHub API root cert (DigiCert High Assurance EV Root CA)
const char* rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBs\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSswKQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5j\n" \
"ZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAwMFoXDTMxMTExMDAwMDAwMFowbDEL\n" \
"MAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3\n" \
"LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFuY2Ug\n" \
"RVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm\n" \
"+9S75S0tMqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTW\n" \
"PNt0OKRKzE0lgvdKpVMSOO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEM\n" \
"xChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFB\n" \
"Ik5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQNAQTXKFx01p8VdteZOE3\n" \
"hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUeh10aUAsg\n" \
"EsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQF\n" \
"MAMBAf8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaA\n" \
"FLE+w2kD+L9HAdSYJhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3Nec\n" \
"nzyIZgYIVyHbIUf4KmeqvxgydkAQV8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6z\n" \
"eM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFpmyPInngiK3BD41VHMWEZ71jF\n" \
"hS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJlQwGWQFJQ41HRxGLGv/cREcw\n" \
"nZAm50ydk+28n4RDrWjBfXnL2l9F8c8qZ4L1W4e/4s1+rQ8wT8E2sY0f+J1s3mK3\n" \
"T2h0jZf1N84D1KAnT1Z4u82u3pG/vI0Fp4X1zD8yI2b5a5C4Yh1+J3hD0l+X5H\n" \
"-----END CERTIFICATE-----\n";

void OTAUpdater::begin() {
    _prefs.begin("ota", false);
    _failed_tag = _prefs.getString("failed_tag", "");

    // Penanda ini ditulis sesaat sebelum reboot setelah unduhan sukses. Bila
    // firmware yang sedang berjalan bertag sama, berarti versi itu berhasil
    // boot, jadi penandanya dihapus. Pembersihan lama di main.cpp hanya jalan
    // bila state bootloader PENDING_VERIFY, yang tidak selalu terjadi; akibatnya
    // versi yang baru saja terpasang ikut terblacklist dan menghalangi OTA
    // ulang ke tag yang sama (ERROR_BLACKLISTED yang palsu).
    if (_failed_tag.length() > 0 && _failed_tag == CURRENT_VERSION) {
        Serial.println("[OTA] Penanda blacklist " + _failed_tag + " dihapus: firmware ini berhasil boot.");
        _prefs.remove("failed_tag");
        _failed_tag = "";
    }
    
    // Validasi firmware sekarang dipindah ke baris pertama main.cpp::setup()
    // agar tidak terjadi rollback race condition jika WiFi gagal konek.
}

void OTAUpdater::confirmWorking() {
    // If the app runs long enough without crashing, mark it valid.
    // In our case, begin() already marked it valid.
}


TaskHandle_t otaTaskHandle = NULL;
void otaTask(void *pvParameters) {
    OTAUpdater* updater = (OTAUpdater*)pvParameters;
    updater->checkForUpdate();
    otaTaskHandle = NULL;
    vTaskDelete(NULL);
}

void OTAUpdater::loop() {
    // Cek update tiap 12 jam (atau saat boot + 30 detik), atau segera bila
    // diminta lewat requestCheck().
    if (_force_check || (_last_check == 0 && millis() > 30000) || (millis() - _last_check > 43200000)) {
        _force_check = false;
        _last_check = millis();
        if (otaTaskHandle == NULL) {
            Serial.println("[OTA] Memicu FreeRTOS Background Task di Core 0...");
            xTaskCreatePinnedToCore(otaTask, "OTA_Task", 8192, this, 1, &otaTaskHandle, 0); // Core 0
        }
    }
}

void OTAUpdater::requestCheck() {
    _prefs.remove("failed_tag");
    _failed_tag = "";
    _force_check = true;
}

// Satu gangguan WiFi/TLS sesaat tidak boleh langsung menjadi "gagal update"
// yang bertahan sampai cek berikutnya (12 jam). Dicoba beberapa kali dulu.
#define OTA_MAX_ATTEMPTS 3
#define OTA_RETRY_DELAY_MS 10000

bool OTAUpdater::downloadWithRetry(const char* url, const char* tag) {
    for (int attempt = 1; attempt <= OTA_MAX_ATTEMPTS; attempt++) {
        if (attempt > 1) {
            Serial.print("[OTA] Percobaan ulang ");
            Serial.print(attempt);
            Serial.print("/");
            Serial.print(OTA_MAX_ATTEMPTS);
            Serial.print(" (sebelumnya gagal: ");
            Serial.print(_last_error);
            Serial.println(")");
            ota_status = "DOWNLOADING_FIRMWARE";
            vTaskDelay(pdMS_TO_TICKS(OTA_RETRY_DELAY_MS));
        }
        if (performUpdate(url, tag)) return true;
        // Tidak ada gunanya mengulang bila partisi memang tidak muat.
        if (_last_error == "update_begin") break;
    }
    return false;
}

void OTAUpdater::checkForUpdate() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    ota_status = "CHECKING_GITHUB"; networkMgr.forcePublishStatus();
    Serial.println("[OTA] Mengecek versi terbaru di GitHub...");
    
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    
    String url = String("https://api.github.com/repos/") + configMgr.config.ota_repo + "/releases/latest";
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Wajib agar bisa mengikuti redirect jika Repo pindah organisasi
    http.begin(client, url);
    int httpCode = http.GET();
    
    if (httpCode == 200) {
        // [BULLETPROOF] Menggunakan Stream dan Filter agar hemat RAM (anti-crash walau JSON GitHub raksasa)
        StaticJsonDocument<200> filter;
        filter["tag_name"] = true;
        filter["assets"][0]["name"] = true;
        filter["assets"][0]["browser_download_url"] = true;
        
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
        
        if (!error) {
            String latest_tag = doc["tag_name"].as<String>();
            Serial.println("[OTA] Versi saat ini: " CURRENT_VERSION);
            Serial.println("[OTA] Versi terbaru: " + latest_tag);
            
            if (latest_tag != CURRENT_VERSION && latest_tag != "") {
                if (latest_tag == _failed_tag) {
                    ota_status = "ERROR_BLACKLISTED"; networkMgr.forcePublishStatus();
                    Serial.println("[OTA] SKIPPED! Versi ini (" + latest_tag + ") pernah membuat sistem crash sebelumnya (Blacklisted).");
                    return;
                }
                
                String bin_url = "";
                JsonArray assets = doc["assets"];
                for (JsonVariant asset : assets) {
                    String name = asset["name"].as<String>();
                    if (name.endsWith(".bin") && name.indexOf(BOARD_VARIANT) >= 0) {
                        bin_url = asset["browser_download_url"].as<String>();
                        break;
                    }
                }
                
                if (bin_url != "") {
                    Serial.println("[OTA] Ditemukan file .bin! Mengunduh dari: " + bin_url);
                    networkMgr.publishLog(("Mengunduh Firmware dari: " + bin_url).c_str());
                    
                    if (downloadWithRetry(bin_url.c_str(), latest_tag.c_str())) {
                        // Kita akan menandai failed_tag SETELAH download sukses tapi SEBELUM reboot. 
                        // Jika firmware baru gagal boot (crash), dia akan rollback dan blacklist ini tetap ada.
                        // Jika sukses boot, firmware baru akan menghapus blacklist ini di begin().
                        _prefs.putString("failed_tag", latest_tag);
                        ota_status = "UPDATE_SUCCESS_RESTARTING"; networkMgr.forcePublishStatus();
                        Serial.println("[OTA] Update selesai! Restarting...");
                        networkMgr.publishLog("OTA Sukses! Alat segera di-restart.");
                        delay(1000);
                        ESP.restart();
                    } else {
                        ota_status = "ERROR_UPDATE_FAILED:" + _last_error;
                        networkMgr.forcePublishStatus();
                        Serial.println("[OTA] Update gagal: " + _last_error);
                    }
                } else {
                    ota_status = "ERROR_NO_BIN_FOUND"; networkMgr.forcePublishStatus();
                    Serial.println("[OTA] Gagal: Tidak ada file .bin di GitHub Release ini!");
                }
            } else {
                ota_status = "UP_TO_DATE"; networkMgr.forcePublishStatus();
                Serial.println("[OTA] Anda sudah menggunakan versi terbaru atau sama.");
            }
        } else {
            ota_status = "ERROR_JSON_PARSE";
            Serial.print("[OTA] JSON Parse Failed: ");
            Serial.println(error.c_str());
        }
    } else {
        ota_status = "ERROR_API_" + String(httpCode);
        Serial.printf("[OTA] Gagal menghubungi GitHub API. Kode: %d\n", httpCode);
    }
    http.end();
}

bool OTAUpdater::performUpdate(const char* url, const char* tag) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.begin(client, url);
    int httpCode = http.GET();
    
    if (httpCode != 200) {
        ota_status = "ERROR_DOWNLOAD_" + String(httpCode);
        _last_error = "http_" + String(httpCode);
        http.end();
        Serial.printf("[OTA] Gagal mengunduh firmware. Kode: %d\n", httpCode);
        return false;
    }
    
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength, U_FLASH);

    if (canBegin) {
        ota_status = "DOWNLOADING_FIRMWARE"; networkMgr.forcePublishStatus();
        Serial.println("[OTA] Memulai penulisan ke memori Flash...");

        // Baca & tulis manual per-chunk (BUKAN Update.writeStream() yang blocking panjang)
        // supaya task IDLE0 tetap sempat dijadwalkan dan me-reset Task Watchdog-nya.
        // Tanpa vTaskDelay ini, OTA_Task (pinned Core 0) menahan CPU terlalu lama
        // sampai esp_task_wdt trigger abort() di tengah proses flashing.
        WiFiClient* stream = http.getStreamPtr();
        uint8_t buff[1024];
        size_t written = 0;
        int last_pct_logged = -1;
        unsigned long last_data_ms = millis();

        while (http.connected() && written < (size_t)contentLength) {
            size_t avail = stream->available();
            if (avail > 0) {
                size_t toRead = avail > sizeof(buff) ? sizeof(buff) : avail;
                size_t readBytes = stream->readBytes(buff, toRead);
                size_t writtenNow = Update.write(buff, readBytes);
                if (writtenNow != readBytes) {
                    _last_error = "tulis_flash";
                    Update.abort();
                    http.end();
                    Serial.printf("[OTA] Penulisan flash gagal di offset %u!\n", (unsigned)written);
                    return false;
                }
                written += writtenNow;
                last_data_ms = millis();

                int pct = (int)((written * 100) / contentLength);
                if (pct != last_pct_logged && pct % 10 == 0) {
                    Serial.printf("[OTA] Progress: %d%%\n", pct);
                    last_pct_logged = pct;
                }
            } else if (millis() - last_data_ms > 15000) {
                // Timeout: 15 detik tanpa data baru dari server
                Serial.println("[OTA] Timeout: tidak ada data masuk selama 15 detik.");
                _last_error = "timeout";
                Update.abort();
                http.end();
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(1)); // Yield agar IDLE0/Task Watchdog tetap sehat
        }

        if (written == (size_t)contentLength) {
            Serial.println("[OTA] Penulisan selesai (100%).");
        } else {
            _last_error = "koneksi_putus";
            Update.abort();
            http.end();
            Serial.printf("[OTA] Penulisan gagal! Ditulis: %d/%d\n", (int)written, contentLength);
            return false;
        }

        if (Update.end()) {
            if (Update.isFinished()) {
                Serial.println("[OTA] Update berhasil divalidasi!");
                
                // Update.end() otomatis mengubah boot partition ke firmware baru.
                // JANGAN panggil esp_ota_set_boot_partition() lagi di sini 
                // karena justru akan memutar boot partition kembali ke versi LAMA!
                return true;
            }
        }
    }
    
    _last_error = canBegin ? "validasi" : "update_begin";
    http.end();
    Serial.printf("[OTA] Error Update: %s\n", Update.errorString());
    return false;
}
