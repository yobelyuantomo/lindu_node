#ifndef ML_INFERENCE_H
#define ML_INFERENCE_H

#include <Arduino.h>
#include "SensorManager.h"

// Inferensi klasifikasi seismik di sisi node.
//
// Tujuannya satu: membuat Lone Wolf Mode mampu membedakan gempa dari getaran
// kuat non-seismik ketika server tidak terjangkau. Tanpa ini, node yang
// terisolasi hanya punya ambang PGA > 0.60 — yang juga dilewati oleh palu
// godam dan pintu yang dibanting sangat keras.
//
// Modelnya berupa ensemble pohon keputusan yang ditanam sebagai array C
// (EdgeModel.h, dihasilkan oleh ml_training/export_edge_model.py). Tidak ada
// runtime ML yang ditanam; inferensi hanya penelusuran array sehingga waktu
// eksekusinya dapat diprediksi dan tidak pernah mengalokasikan memori.
//
// PERINGATAN KESETARAAN
// ---------------------
// Ekstraksi fitur di bawah ini HARUS menghasilkan angka yang sama dengan
// ml/feature_extractor.py di server. Kalau berbeda, model akan menerima
// masukan yang bukan dilatihkan padanya, dan hasilnya salah TANPA gejala apa
// pun — prediksi tetap keluar dan confidence tetap terlihat wajar.
//
// Kesetaraan itu tidak bisa dibuktikan saat kompilasi. Karena itu tersedia
// perintah MQTT `ml_selftest`: node menerbitkan vektor fitur yang ia hitung,
// lalu server membandingkannya dengan hasil ekstraksinya sendiri atas
// telemetri yang sama. Jalankan itu setelah setiap flash sebelum mempercayai
// keputusan mode mandiri.

#define ML_WINDOW_SECONDS 2.0f
#define ML_MIN_SAMPLES 4
#define ML_MAX_SAMPLES 24   // 2 detik @ 10 Hz, dengan sedikit kelonggaran

// Ambang PGA yang membuat sebuah jendela layak dianalisis. Harus sama dengan
// RULE_PGA_MIN di ml/feature_extractor.py.
#define ML_TRIGGER_PGA 0.12f

struct MLResult {
    bool  valid;          // false bila jendela belum memadai
    int   class_index;
    const char* label;
    float confidence;
    uint32_t latency_us;
};

class MLInference {
public:
    void begin();

    // Masukkan satu sampel ke jendela berjalan. Murah; aman dipanggil pada
    // setiap SensorEvent.
    void addSample(const SensorEvent& ev, double epoch);

    // Jalankan model bila jendela memadai. Tidak pernah mengalokasikan memori.
    MLResult predict();

    // Apakah jendela saat ini terpicu dan cukup panjang untuk dianalisis?
    bool windowReady() const;

    // Isi `out` dengan vektor fitur terakhir yang dihitung. Dipakai
    // `ml_selftest` untuk membuktikan kesetaraan dengan server.
    // Mengembalikan jumlah fitur yang ditulis.
    int lastFeatures(float* out, int max_out) const;

    const char* modelVersion() const;
    bool isReady() const { return _ready; }

private:
    void extractFeatures();

    struct Sample {
        double ts;
        float pga, sta_lta, ax, ay, az, temperature, pressure;
        int freq_hz;
    };

    Sample _buf[ML_MAX_SAMPLES];
    int  _count = 0;
    int  _head = 0;
    bool _ready = false;

    float _features[32];
    int   _n_features = 0;
};

extern MLInference mlInference;

#endif
