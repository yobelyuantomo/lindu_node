#include <Arduino.h>
#include <sys/time.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "ConfigManager.h"
#include "NetworkManager.h"
#include "SensorManager.h"
#include "MLInference.h"
#include "OTAUpdater.h"
#include <WiFi.h>
#include "esp_ota_ops.h"
#include <Preferences.h>

#include <Adafruit_NeoPixel.h>

// --- RESCUE MODE (SAFE MODE) VARIABLES ---
// RTC memory bertahan saat ESP32 crash atau reboot
RTC_DATA_ATTR int boot_crash_count = 0;
bool is_rescue_mode = false;

// Pin & aktuator berbeda per varian board.
// ARDUINO_USB_CDC_ON_BOOT selalu terdefinisi (fallback 0 dari HardwareSerial.h
// core Arduino-ESP32) sehingga harus dicek NILAINYA, bukan defined().
// Bernilai 1 hanya pada env:esp32s3 (lihat platformio.ini).
#if ARDUINO_USB_CDC_ON_BOOT
    // ESP32-S3 DevKitC (hardware asli/lapangan): TIDAK ada perubahan apa pun
    // di jalur ini dibanding main branch - aktuator valve tetap pakai servo.
    #define USE_SERVO_VALVE 1
    #define RGB_PIN 48
    #define SERVO_PIN 5
    #define BUZZER_PIN 6
    // Buzzer S3 (hardware asli/lapangan): active-LOW
    #define BUZZER_ON  LOW
    #define BUZZER_OFF HIGH
    #define BUZZER_ON_DUTY  128
    #define BUZZER_OFF_DUTY 255
    #include <ESP32Servo.h>
    Servo myServo;
#else
    // ESP32 classic / WROOM (unit pengujian): valve & door-lock pakai relay 2-channel
    #define USE_SERVO_VALVE 0
    // LED RGB modul unit pengujian adalah LED RGB analog biasa (katoda umum,
    // 3 kaki PWM terpisah R/G/B), BUKAN NeoPixel/WS2812 satu-pin seperti di S3.
    #define RGB_R_PIN 4
    #define RGB_G_PIN 16
    #define RGB_B_PIN 17
    #define BUZZER_PIN 14
    #define RELAY_DOOR_PIN 25   // Relay CH1 -> Solenoid Door Lock 12V
    #define RELAY_VALVE_PIN 26  // Relay CH2 -> Solenoid Water/Gas Valve 12V
    #define PIR_PIN 19          // Sensor PIR (HC-SR501/sejenis) - OUTPUT sensor ke GPIO (HIGH = ada gerakan), INPUT_PULLDOWN agar aman saat sensor belum terpasang. Dipindah dari GPIO27 ke GPIO19 (uji coba sensor LD2410C sebelumnya, kembali pakai PIR).
    #define OCCUPANCY_WINDOW_MS 600000UL // 10 menit - jika ADA MINIMAL 1 deteksi PIR dalam window ini, pintu auto-unlock
    // Modul relay 2-channel (active LOW): LOW = relay ON (energized), HIGH = relay OFF
    #define RELAY_ON  LOW
    #define RELAY_OFF HIGH
    // Buzzer unit pengujian (classic/WROOM): terbukti active-HIGH secara fisik
    // (dites 2026-09-17: mencabut sinyal GPIO14 membuat buzzer diam, artinya
    // GPIO HIGH = nyala), KEBALIKAN dari asumsi lama "active-LOW".
    #define BUZZER_ON  HIGH
    #define BUZZER_OFF LOW
    #define BUZZER_ON_DUTY  128
    #define BUZZER_OFF_DUTY 0
#endif

#define BOOT_BUTTON_PIN 0

ConfigManager configMgr;
NetworkManager networkMgr;
SensorManager sensorMgr;

#if ARDUINO_USB_CDC_ON_BOOT
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
#else
// Shim minimal yang meniru API Adafruit_NeoPixel (begin/Color/setPixelColor/show)
// agar kode animasi LED di bawah tidak perlu diubah, tapi outputnya lewat
// 3 pin analogWrite/PWM biasa ke LED RGB modul analog (katoda umum).
class AnalogRGB {
public:
    AnalogRGB(uint8_t rPin, uint8_t gPin, uint8_t bPin) : _r(rPin), _g(gPin), _b(bPin) {}
    void begin() {
        pinMode(_r, OUTPUT);
        pinMode(_g, OUTPUT);
        pinMode(_b, OUTPUT);
    }
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    void setPixelColor(uint8_t /*index*/, uint32_t color) { _color = color; }
    void show() {
        analogWrite(_r, (_color >> 16) & 0xFF);
        analogWrite(_g, (_color >> 8) & 0xFF);
        analogWrite(_b, _color & 0xFF);
    }
private:
    uint8_t _r, _g, _b;
    uint32_t _color = 0;
};
AnalogRGB pixels(RGB_R_PIN, RGB_G_PIN, RGB_B_PIN);
#endif

QueueHandle_t eventQueue;
unsigned long global_alarm_until = 0;
bool is_valve_locked = false;
Preferences actPrefs;
#if !ARDUINO_USB_CDC_ON_BOOT
bool is_door_locked = true; // Default: pintu terkunci (khusus unit ESP32 classic)
#endif
bool is_motion_detected = false; // Khusus unit ESP32 classic (sensor PIR belum dipasang di S3), selalu false di S3
unsigned long last_motion_ms = 0; // Timestamp deteksi PIR terakhir (khusus classic), dipakai untuk window occupancy 10 menit
float local_latest_pga = 0.0;
unsigned long local_alarm_until = 0;
unsigned long identify_until = 0;
TaskHandle_t networkTaskHandle;

// Helper: Ambil Waktu Epoch NTP
double getEpochTime() {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) return 0.0;
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// TASK CORE 0: Mengurus Wi-Fi, MQTT, JSON, Edge Computing, dan Animasi LED
void networkTaskCode(void* parameter) {
    networkMgr.begin(&configMgr);
    
    // Sinkronisasi Waktu NTP untuk Timestamp
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    pixels.begin();
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, BUZZER_OFF); // MATI (lihat definisi BUZZER_OFF per varian board)

#if USE_SERVO_VALVE
    // ESP32-S3 PWM Timer Allocation untuk Servo (Hanya alokasi, tidak di-enable)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    // Servo sengaja TIDAK di-attach di sini agar tidak auto-enable saat alat menyala
#else
    pinMode(RELAY_DOOR_PIN, OUTPUT);
    pinMode(RELAY_VALVE_PIN, OUTPUT);
    digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
    digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
    // INPUT_PULLDOWN (bukan INPUT polos): kalau sensor PIR belum/tidak terpasang, GPIO
    // mengambang bisa kebaca HIGH terus oleh noise, membuat occupancy logic mengira
    // ada gerakan terus-menerus dan pintu tidak pernah terkunci. Pull-down memaksa
    // default LOW (tidak ada gerakan) saat tidak ada sensor yang aktif men-drive pin.
    pinMode(PIR_PIN, INPUT_PULLDOWN);
#endif

    float breathAngle = 0;

    SensorEvent ev;
    
    while(1) {
        networkMgr.loop(); // Handle rutin MQTT
        
        // Heartbeat status setiap 10 detik
        static unsigned long last_status = 0;
        if (millis() - last_status >= 10000) {
            networkMgr.publishStatus("online", sensorMgr.sensor_ok, sensorMgr.getTiltAngle(), sensorMgr.getPose(), is_motion_detected);
            last_status = millis();
        }
        
        // Cek status alarm global dan lokal
        bool is_global_alarm = (millis() < global_alarm_until && global_alarm_until > 0);
        bool is_local_alarm = (millis() < local_alarm_until && local_alarm_until > 0);
        
        // Animasi LED Cerdas (Sesuai Status Sensor & WiFi)
        if (WiFi.status() == WL_CONNECTED && networkMgr.isConnected()) {
            otaUpdater.loop();
            breathAngle += 0.05;
            if (breathAngle > 2 * PI) breathAngle -= 2 * PI;
            // Mode "Kamar Tidur": Diturunkan dari pengali 20.0 menjadi 3.0 (Kecerahan max hanya 6/255)
            // LED akan bernafas dengan sangat, sangat redup dan tidak menyilaukan saat lampu kamar dimatikan.
            int brightness = (sin(breathAngle) + 1.0) * 3.0; 
            
            bool hw611_ok = sensorMgr.bme_ok || sensorMgr.bmp_ok;
            
#if USE_SERVO_VALVE
            // LOGIKA VALVE MANUAL RESET (Hanya nyalakan motor saat diperintah sistem)
            static bool last_valve_state = false;
            if (is_valve_locked != last_valve_state) {
                // PRE-WRITE: Set target sudut SEBELUM motor dialiri listrik agar tidak melompat kaget
                int target_angle = is_valve_locked ? 90 : 0;
                myServo.write(target_angle); 
                
                // Beri jeda sangat kecil sebelum attach untuk stabilitas sinyal PWM FreeRTOS
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Nyalakan tenaga motor
                myServo.attach(SERVO_PIN, 500, 2400); 
                myServo.write(target_angle); // Tulis ulang untuk memastikan sinyal PWM terkirim

                // Beri waktu 2.5 detik (cukup panjang) agar motor yang membawa beban fisik berat
                // punya cukup waktu untuk sampai ke tujuan sebelum listriknya dicabut
                vTaskDelay(pdMS_TO_TICKS(2500)); 
                
                // Matikan aliran listrik (Zero-Torque Standby)
                myServo.detach();

                last_valve_state = is_valve_locked;
            }
#else
            // LOGIKA RELAY VALVE (Solenoid Water/Gas Valve 12V)
            // Fail-safe: valve tertutup (OFF) saat is_valve_locked true (gempa/gas bocor/manual)
            static bool last_valve_state = is_valve_locked;
            if (is_valve_locked != last_valve_state) {
                digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
                last_valve_state = is_valve_locked;
            }

            // SENSOR PIR: Deteksi gerakan/orang di lokasi (HIGH = ada gerakan).
            // Dibaca duluan agar window occupancy di bawah selalu pakai data terbaru.
            is_motion_detected = digitalRead(PIR_PIN) == HIGH;
            if (is_motion_detected) {
                last_motion_ms = millis();
            }
            {
                static bool last_pir_debug = false;
                if (is_motion_detected != last_pir_debug) {
                    Serial.printf("[PIR DEBUG] motion=%s\n", is_motion_detected ? "TERDETEKSI" : "tidak ada");
                    last_pir_debug = is_motion_detected;
                }
            }

            // LOGIKA RELAY DOOR LOCK (Solenoid Door Lock 12V)
            // Default: ikut perintah manual (lock_door/unlock_door) atau cancel_alarm.
            // KHUSUS selama alarm gempa aktif (is_global_alarm): pintu HANYA unlock kalau
            // ADA MINIMAL 1 deteksi PIR dalam 10 menit terakhir (asumsi ada orang di lokasi
            // untuk evakuasi); kalau tidak ada yang terdeteksi, pintu tetap terkunci
            // walau alarm aktif. Dievaluasi ulang tiap tick SELAMA alarm masih berlangsung
            // (bukan cuma sekali di saat trigger) supaya tetap responsif kalau ada orang
            // baru terdeteksi di tengah-tengah alarm. Di luar alarm, occupancy PIR tidak
            // dicek sama sekali - status pintu murni ikut perintah manual/cancel_alarm.
            static bool last_door_state = is_door_locked;
            if (is_global_alarm) {
                bool occupancy_recent = (last_motion_ms > 0) && (millis() - last_motion_ms < OCCUPANCY_WINDOW_MS);
                is_door_locked = !occupancy_recent;
            }
            if (is_door_locked != last_door_state) {
                digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
                last_door_state = is_door_locked;
            }
#endif


            if (millis() < identify_until) {
                // IDENTIFY MODE: Berkedip Putih
                if ((millis() / 200) % 2 == 0) pixels.setPixelColor(0, pixels.Color(255, 255, 255));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                // analogWrite (LEDC) dulu, baru digitalWrite - digitalWrite saja TIDAK memutus
                // channel PWM yang sudah dipakai analogWrite (mis. saat alarm sebelumnya), jadi
                // buzzer bisa nyangkut bunyi panjang terus-menerus kalau cuma digitalWrite.
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                digitalWrite(BUZZER_PIN, BUZZER_OFF);
            } else if (is_global_alarm) {
                // KONFIRMASI GEMPA (DARI SERVER): Berkedip Merah Cepat (Strobo) & Buzzer Menyala
                if ((millis() / 100) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_ON_DUTY); // 50% Duty Cycle (Max Volume Tone untuk Speaker)
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY); // MATI
                }
            } else if (sensorMgr.gas_leak_detected) {
                // KEBOCORAN GAS TERDETEKSI (MQ-2): Berkedip Oranye & Buzzer Menyala.
                // Pola blink lebih lambat dari alarm gempa (300ms vs 100ms) supaya
                // operator bisa membedakan jenis alarm dari suara/kedipannya.
                if ((millis() / 300) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 100, 0));
                    analogWrite(BUZZER_PIN, BUZZER_ON_DUTY);
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                }
            } else if (is_local_alarm) {
                // DETEKSI GETARAN LOKAL (Menunggu Konfirmasi Node Lain): HANYA Berkedip Pink
                if ((millis() / 500) % 2 == 0) {
                    pixels.setPixelColor(0, pixels.Color(255, 20, 147)); // Hot Pink
                } else {
                    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
                    // Wajib analogWrite juga: kalau transisi ini datang tepat setelah alarm
                    // gempa/gas (yang menyalakan buzzer via analogWrite) baru dibatalkan,
                    // digitalWrite saja tidak memutus PWM LEDC yang masih aktif di duty terakhir
                    // - itu penyebab buzzer nyangkut bunyi panjang terus-menerus.
                    analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                    digitalWrite(BUZZER_PIN, BUZZER_OFF);
                }
            } else if (otaUpdater.ota_status == "DOWNLOADING_FIRMWARE" || otaUpdater.ota_status == "CHECKING_GITHUB") {
                // MATIKAN BUZZER JIKA SEBELUMNYA NYALA
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY); digitalWrite(BUZZER_PIN, BUZZER_OFF);
                
                // OTA SEDANG MENGUNDUH: Berkedip CYAN (Biru Tosca) sangat cepat layaknya loading
                if ((millis() / 80) % 2 == 0) pixels.setPixelColor(0, pixels.Color(0, 255, 255));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            } else if (otaUpdater.ota_status == "UPDATE_SUCCESS_RESTARTING") {
                // OTA SELESAI & SIAP RESTART: Menyala Ungu Solid
                pixels.setPixelColor(0, pixels.Color(128, 0, 128));
            } else if (!sensorMgr.sensor_ok && !hw611_ok) {
                
                // Semua sensor mati: Berkedip Merah Cepat (Bahaya Fatal)
                if ((millis() / 200) % 2 == 0) pixels.setPixelColor(0, pixels.Color(50, 0, 0));
                else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            } else if (!sensorMgr.sensor_ok) {
                // Akselerometer mati: Bernafas Merah
                pixels.setPixelColor(0, pixels.Color(brightness, 0, 0));
            } else if (!hw611_ok) {
                // Suhu mati: Bernafas Kuning/Oranye
                pixels.setPixelColor(0, pixels.Color(brightness, brightness * 0.5, 0));
            } else {
                // Semua normal: Bernafas Hijau
                pixels.setPixelColor(0, pixels.Color(0, brightness, 0));
            }
        } else {
            // Berkedip Merah Pelan jika Wi-Fi putus
            if ((millis() / 500) % 2 == 0) pixels.setPixelColor(0, pixels.Color(30, 0, 0));
            else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        }
        
        // Matikan speaker sepenuhnya jika kondisi aman
        static bool is_buzzer_active = false;

        if (is_global_alarm || is_local_alarm || sensorMgr.gas_leak_detected) {
            is_buzzer_active = true;
        } else {
            if (is_buzzer_active) {
                // analogWrite dulu untuk memutus PWM LEDC (lihat catatan di cabang is_local_alarm
                // di atas), baru pinMode/digitalWrite untuk memastikan pin benar-benar diam.
                analogWrite(BUZZER_PIN, BUZZER_OFF_DUTY);
                pinMode(BUZZER_PIN, OUTPUT);
                digitalWrite(BUZZER_PIN, BUZZER_OFF); // mematikan arus sepenuhnya
                is_buzzer_active = false;
            }
        }
        
        pixels.show();
        
        // Cek apakah ada data sensor di antrean (Non-Blocking)
        if (xQueueReceive(eventQueue, &ev, 0) == pdTRUE) {
            local_latest_pga = ev.pga;

            // Isi jendela model. Murah: hanya menyalin satu struct ke ring
            // buffer berukuran tetap, tanpa alokasi memori.
            mlInference.addSample(ev, getEpochTime());

            if (ev.pga > 0.12) {
                local_alarm_until = millis() + 5000; // Tahan warna pink selama 5 detik
                
                // ========== OFFLINE FAIL-SAFE (LONE WOLF MODE) ==========
                // Jika MQTT server mati DAN getaran AMAT SANGAT BRUTAL (PGA > 0.60G),
                // ESP32 mengambil alih kekuasaan mutlak: langsung membunyikan sirine dan
                // mengunci katup gas (global_alarm_until diset, sama seperti trigger_siren
                // biasa). Pintu TIDAK dipaksa buka langsung di sini - begitu global_alarm_until
                // aktif, loop occupancy di networkTaskCode yang menentukan: unlock hanya
                // kalau PIR mendeteksi orang dalam 10 menit terakhir, sama seperti alarm biasa.
                // Ini adalah garis pertahanan terakhir saat infrastruktur internet runtuh.
                //
                // Lapisan kecerdasan (Assignment 3) MENURUNKAN ambang itu, dan
                // hanya itu. Model boleh membuat node bertindak LEBIH CEPAT pada
                // guncangan yang dikenalinya sebagai gempa, tetapi tidak pernah
                // diberi hak MEMBATALKAN pemicu PGA > 0.60.
                //
                // Kenapa tidak boleh membatalkan: saat server tak terjangkau,
                // salah menahan aksi pada gempa sungguhan berarti katup gas
                // tetap terbuka di bangunan yang sedang runtuh. Biaya kesalahan
                // ke arah itu jauh lebih besar daripada sirine yang sesekali
                // berbunyi karena palu godam.
                bool lone_wolf = (ev.pga > 0.60);
                const char* alasan = "PGA Ekstrem";

                if (!lone_wolf && mlInference.isReady() && ev.pga > 0.25) {
                    MLResult ml = mlInference.predict();
                    if (ml.valid && strcmp(ml.label, "earthquake") == 0 &&
                        ml.confidence >= 0.90f) {
                        lone_wolf = true;
                        alasan = "Model mengenali pola gempa";
                    }
                }

                if (!networkMgr.isConnected() && lone_wolf) {
                    Serial.printf("[!!!] LONE WOLF MODE: Server offline + %s! Mengambil alih kendali!\n", alasan);
                    char pesan[128];
                    snprintf(pesan, sizeof(pesan),
                             "LONE WOLF MODE DIINTIASI! Server Offline & %s (PGA %.2fG).",
                             alasan, ev.pga);
                    networkMgr.publishLog(pesan);
                    global_alarm_until = millis() + 15000; // Sirine merah 15 detik
                    is_valve_locked = true;
                    actPrefs.putBool("valve_locked", true);
                }
                
                // TICK Dinamis: Volume/Intensitas diwakili oleh durasi (Haptic Feedback)
                static unsigned long last_tick_time = 0;
                if (!is_global_alarm && (millis() - last_tick_time > 500)) { 
                    last_tick_time = millis();
                    
                    // Semakin besar getaran (PGA), semakin lama durasi beep-nya
                    int beep_duration = (int)(ev.pga * 40.0);
                    if (beep_duration < 5) beep_duration = 5;     // Getaran pelan = 5ms (Tik kecil)
                    if (beep_duration > 100) beep_duration = 100; // Getaran keras = 100ms (Bip panjang/keras)
                    
                    digitalWrite(BUZZER_PIN, BUZZER_ON);
                    vTaskDelay(pdMS_TO_TICKS(beep_duration));
                    digitalWrite(BUZZER_PIN, BUZZER_OFF);
                }
            }
            networkMgr.publishEvent(
                ev.pga, ev.ratio, ev.freq_hz,
                ev.accel_x, ev.accel_y, ev.accel_z,
                ev.uptime_ms, getEpochTime(),
                configMgr.config.lat, configMgr.config.lon,
                ev.temperature, ev.pressure
            );
        }
        
        // Beri nafas untuk Watchdog Core 0 (delay 20ms juga membuat animasi breathing lebih smooth)
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    delay(2000); // Tunggu Serial stabil
    
    Serial.println("\n\n==========================================");
    Serial.println(" Lindu.id - Life-Critical Node v2.0 ");
    Serial.println("==========================================");

    // 0. SEGERA tandai firmware sebagai VALID agar tidak di-rollback
    // Ini HARUS dilakukan sebelum WiFi agar OTA tidak loop
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.println("[OTA] Firmware baru terdeteksi! Menandai sebagai VALID...");
            esp_ota_mark_app_valid_cancel_rollback();
            
            // Hapus blacklist karena berhasil boot
            Preferences tempPrefs;
            tempPrefs.begin("ota", false);
            tempPrefs.remove("failed_tag");
            tempPrefs.end();
        }
    }

    // 1. Muat Konfigurasi (EEPROM) & WiFi Captive Portal
    configMgr.begin();
    
    // Nyalakan LED biru saat sedang setup WiFi
    pixels.begin();
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, BUZZER_OFF); // MATI (lihat definisi BUZZER_OFF per varian board)

#if USE_SERVO_VALVE
    // ESP32-S3 PWM Timer Allocation untuk Servo (Hanya alokasi, tidak di-enable)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    // Servo sengaja TIDAK di-attach di sini agar tidak auto-enable saat alat menyala
#else
    pinMode(RELAY_DOOR_PIN, OUTPUT);
    pinMode(RELAY_VALVE_PIN, OUTPUT);
    digitalWrite(RELAY_DOOR_PIN, is_door_locked ? RELAY_OFF : RELAY_ON);
    digitalWrite(RELAY_VALVE_PIN, is_valve_locked ? RELAY_OFF : RELAY_ON);
    // INPUT_PULLDOWN (bukan INPUT polos): kalau sensor PIR belum/tidak terpasang, GPIO
    // mengambang bisa kebaca HIGH terus oleh noise, membuat occupancy logic mengira
    // ada gerakan terus-menerus dan pintu tidak pernah terkunci. Pull-down memaksa
    // default LOW (tidak ada gerakan) saat tidak ada sensor yang aktif men-drive pin.
    pinMode(PIR_PIN, INPUT_PULLDOWN);
#endif

    pixels.setPixelColor(0, pixels.Color(0, 0, 40));
    pixels.show();
    
    Serial.println("[i] Memulai koneksi WiFi...");
    
    // RETRY LOGIC: Coba koneksi WiFi 3x sebelum masuk Captive Portal
    // Ini mengatasi masalah pasca-OTA reboot dimana router belum siap
    bool wifi_ok = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[WiFi] Percobaan %d/3...\n", attempt);
        WiFi.begin(); // Menggunakan memori NVS bawaan ESP32 // Gunakan kredensial dari EEPROM
        
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(500);
            Serial.print(".");
            pixels.setPixelColor(0, (millis() / 300) % 2 ? pixels.Color(0, 0, 40) : pixels.Color(0, 0, 0));
            pixels.show();
        }
        Serial.println();
        
        if (WiFi.status() == WL_CONNECTED) {
            wifi_ok = true;
            Serial.println("[OK] WiFi Terhubung via kredensial tersimpan!");
            networkMgr.publishLog("WiFi Terhubung (Kredensial Tersimpan).");
            break;
        }
        Serial.printf("[!] Gagal percobaan %d. Menunggu 3 detik...\n", attempt);
        delay(3000);
    }
    
    // Jika 3x retry gagal, baru buka Captive Portal sebagai fallback
    if (!wifi_ok) {
        Serial.println("[i] Retry habis. Membuka Captive Portal...");
        networkMgr.publishLog("Gagal WiFi 3x, Membuka Captive Portal.");
        if (!configMgr.startCaptivePortal()) {
            Serial.println("[!] Gagal connect WiFi. Alat akan restart...");
            delay(3000);
            ESP.restart();
        }
    }
    Serial.println("[OK] WiFi Terhubung.");

    // 2. Buat Antrean Pesan Lintas-Core (10 slot pesan @ ~40 byte)
    eventQueue = xQueueCreate(10, sizeof(SensorEvent));

    // 3. Pisahkan Tugas Berat ke Core 0
    xTaskCreatePinnedToCore(
        networkTaskCode,   // Fungsi
        "NetworkTask",     // Nama task
        12288,             // Stack 12KB
        NULL,              // Parameter
        1,                 // Prioritas 1
        &networkTaskHandle,// Handle
        0                  // Kunci ke Core 0
    );
    
    // 4. Inisialisasi Sensor di Core 1
    mlInference.begin();
    sensorMgr.begin(eventQueue);
    otaUpdater.begin();
}

void loop() {
    // --- TOMBOL BOOT (FACTORY RESET) ---
    static unsigned long boot_press_time = 0;
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        if (boot_press_time == 0) {
            boot_press_time = millis();
        } else if (millis() - boot_press_time > 5000) { // Tahan 5 detik
            Serial.println("\n[!] FACTORY RESET VIA TOMBOL BOOT DIMULAI!");
            digitalWrite(BUZZER_PIN, BUZZER_ON); // Bunyikan bel
            configMgr.resetConfig();
            Preferences prefs;
            prefs.begin("ota", false); prefs.clear(); prefs.end();
            delay(1000);
            ESP.restart();
        }
    } else {
        boot_press_time = 0;
    }

    // TASK CORE 1: Membaca Sensor (I2C) & Filter DSP (Real-time murni)
    sensorMgr.loop();  
    
    // Beri sedikit nafas agar Watchdog Timer Core 1 tidak marah
    delay(1);
}
