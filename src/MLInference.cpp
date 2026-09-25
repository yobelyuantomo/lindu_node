#include "MLInference.h"
#include "EdgeModel.h"
#include <math.h>

MLInference mlInference;

// Urutan fitur di bawah HARUS sama persis dengan FEATURE_NAMES di
// ml/feature_extractor.py. EdgeModel.h membawa salinan daftar namanya, dan
// begin() memverifikasi jumlahnya cocok sebelum model dipakai.
// Dinamai ML_EPS, bukan EPS: header xtensa ESP32 (specreg.h) sudah memakai
// nama EPS sebagai makro, dan tabrakannya baru terlihat saat membangun untuk
// target sungguhan.
static const float ML_EPS = 1e-9f;

void MLInference::begin() {
    _count = 0;
    _head = 0;
    _n_features = 0;

    // Penjagaan paling dasar terhadap train/serve skew: kalau jumlah fitur yang
    // dihitung firmware berbeda dari yang diharapkan model, jangan dipakai
    // sama sekali. Lebih baik kembali ke ambang PGA lama daripada memberi
    // model masukan yang bukan dilatihkan padanya.
    _ready = (EDGE_N_FEATURES == 21);
    if (!_ready) {
        Serial.printf("[ML] NONAKTIF: model mengharap %d fitur, firmware menghitung 21\n",
                      EDGE_N_FEATURES);
        return;
    }
    Serial.printf("[ML] Model edge siap: %s (%d pohon, %d kelas)\n",
                  EDGE_MODEL_VERSION, EDGE_N_TREES, EDGE_N_CLASSES);
}

void MLInference::addSample(const SensorEvent& ev, double epoch) {
    Sample& s = _buf[_head];
    s.ts = epoch;
    s.pga = ev.pga;
    s.sta_lta = ev.ratio;
    s.freq_hz = ev.freq_hz;
    s.ax = ev.accel_x;
    s.ay = ev.accel_y;
    s.az = ev.accel_z;
    s.temperature = ev.temperature;
    s.pressure = ev.pressure;

    _head = (_head + 1) % ML_MAX_SAMPLES;
    if (_count < ML_MAX_SAMPLES) _count++;
}

bool MLInference::windowReady() const {
    if (!_ready || _count < ML_MIN_SAMPLES) return false;

    // Sama seperti is_analyzable_window() di server: jendela hanya dianalisis
    // bila benar-benar terpicu. Di luar itu firmware mengirim 1 Hz, dan model
    // tidak pernah dilatih pada kepadatan sampel serendah itu.
    for (int i = 0; i < _count; i++) {
        if (_buf[i].pga >= ML_TRIGGER_PGA) return true;
    }
    return false;
}

void MLInference::extractFeatures() {
    // Salin jendela ke larik terurut. Jumlahnya paling banyak ML_MAX_SAMPLES,
    // jadi aman di stack dan tidak perlu alokasi.
    Sample w[ML_MAX_SAMPLES];
    int n = _count;
    int start = (_count == ML_MAX_SAMPLES) ? _head : 0;
    for (int i = 0; i < n; i++) {
        w[i] = _buf[(start + i) % ML_MAX_SAMPLES];
    }

    // Waktu relatif terhadap awal jendela.
    double t0 = w[0].ts;
    float t[ML_MAX_SAMPLES];
    for (int i = 0; i < n; i++) t[i] = (float)(w[i].ts - t0);

    // --- amplitudo ---------------------------------------------------------
    float pga_sum = 0.0f, pga_max = 0.0f;
    int peak_idx = 0;
    for (int i = 0; i < n; i++) {
        pga_sum += w[i].pga;
        if (w[i].pga > pga_max) { pga_max = w[i].pga; peak_idx = i; }
    }
    float pga_mean = pga_sum / n;
    float pga_var = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = w[i].pga - pga_mean;
        pga_var += d * d;
    }
    float pga_std = (n > 1) ? sqrtf(pga_var / (n - 1)) : 0.0f;
    float pga_crest = pga_max / (pga_mean + ML_EPS);

    // --- energi, diintegrasikan terhadap waktu -----------------------------
    // Dijumlahkan dengan bobot selisih waktu, BUKAN per sampel, supaya nilainya
    // tidak ikut berubah saat laju telemetri berpindah 1 Hz <-> 10 Hz.
    float energy = 0.0f, duration_above = 0.0f;
    for (int i = 1; i < n; i++) {
        float dt = t[i] - t[i - 1];
        if (dt <= 0.0f) continue;
        energy += w[i].pga * w[i].pga * dt;
        if (w[i].pga >= ML_TRIGGER_PGA) duration_above += dt;
    }

    float time_to_peak = t[peak_idx] - t[0];
    float rise = w[peak_idx].pga - w[0].pga;
    float rise_rate = (time_to_peak > 0.0f) ? rise / (time_to_peak + ML_EPS) : 0.0f;

    // --- frekuensi ---------------------------------------------------------
    float f_sum = 0.0f, f_min = 1e9f, f_max = -1e9f;
    for (int i = 0; i < n; i++) {
        float f = (float)w[i].freq_hz;
        f_sum += f;
        if (f < f_min) f_min = f;
        if (f > f_max) f_max = f;
    }
    float f_mean = f_sum / n;
    float f_var = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = (float)w[i].freq_hz - f_mean;
        f_var += d * d;
    }
    float f_std = (n > 1) ? sqrtf(f_var / (n - 1)) : 0.0f;

    // --- rasio STA/LTA -----------------------------------------------------
    float s_sum = 0.0f, s_max = -1e9f;
    for (int i = 0; i < n; i++) {
        s_sum += w[i].sta_lta;
        if (w[i].sta_lta > s_max) s_max = w[i].sta_lta;
    }
    float s_mean = s_sum / n;
    float s_var = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = w[i].sta_lta - s_mean;
        s_var += d * d;
    }
    float s_std = (n > 1) ? sqrtf(s_var / (n - 1)) : 0.0f;

    // --- geometri getaran --------------------------------------------------
    float h_sum = 0.0f, v_sum = 0.0f, m_sum = 0.0f, m_max = 0.0f;
    for (int i = 0; i < n; i++) {
        float h = sqrtf(w[i].ax * w[i].ax + w[i].ay * w[i].ay);
        float m = sqrtf(w[i].ax * w[i].ax + w[i].ay * w[i].ay + w[i].az * w[i].az);
        h_sum += h;
        v_sum += fabsf(w[i].az);
        m_sum += m;
        if (m > m_max) m_max = m;
    }
    float hv_ratio = (h_sum / n) / ((v_sum / n) + ML_EPS);

    // --- lingkungan --------------------------------------------------------
    float pressure_delta = (n >= 2) ? (w[n - 1].pressure - w[0].pressure) : 0.0f;
    float temp_sum = 0.0f;
    float gas_max = 0.0f;  // node ini tidak mengirim gas_raw lewat SensorEvent
    for (int i = 0; i < n; i++) temp_sum += w[i].temperature;

    // Urutan WAJIB sama dengan FEATURE_NAMES di ml/feature_extractor.py.
    int k = 0;
    _features[k++] = pga_max;
    _features[k++] = pga_mean;
    _features[k++] = pga_std;
    _features[k++] = pga_crest;
    _features[k++] = energy;
    _features[k++] = duration_above;
    _features[k++] = rise_rate;
    _features[k++] = time_to_peak;
    _features[k++] = f_mean;
    _features[k++] = f_min;
    _features[k++] = f_max;
    _features[k++] = f_std;
    _features[k++] = s_max;
    _features[k++] = s_mean;
    _features[k++] = s_std;
    _features[k++] = hv_ratio;
    _features[k++] = m_max;
    _features[k++] = m_sum / n;
    _features[k++] = pressure_delta;
    _features[k++] = temp_sum / n;
    _features[k++] = gas_max;
    _n_features = k;
}

MLResult MLInference::predict() {
    MLResult r = {false, -1, "unknown", 0.0f, 0};
    if (!windowReady()) return r;

    uint32_t mulai = micros();
    extractFeatures();

    float confidence = 0.0f;
    int idx = edge_model_predict(_features, &confidence);

    r.valid = true;
    r.class_index = idx;
    r.label = EDGE_CLASS_NAMES[idx];
    r.confidence = confidence;
    r.latency_us = micros() - mulai;
    return r;
}

int MLInference::lastFeatures(float* out, int max_out) const {
    int n = (_n_features < max_out) ? _n_features : max_out;
    for (int i = 0; i < n; i++) out[i] = _features[i];
    return n;
}

const char* MLInference::modelVersion() const {
    return EDGE_MODEL_VERSION;
}
