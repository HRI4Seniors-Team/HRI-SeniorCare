/**
 * @file  speech_emotion.cc
 * @brief Lightweight Speech Emotion Recognition — ESP32-S3 implementation
 * ========================================================================
 * Project : SIEVOX — Multimodal Elderly Care Robot
 * ========================================================================
 *
 * Memory budget (approximate):
 *   frame_buf_     :  960 floats  x  4 B =   3.8 KB
 *   ring_buf_      : 4800 floats  x  4 B =  19.2 KB
 *   mel_filters_   :   26 x 257   x  4 B =  26.7 KB
 *   FFT scratch    :  512 complex x  8 B =   4.0 KB
 *   TFLite arena   :                       ~30.0 KB (optional)
 *   --------------------------------------------------------------------
 *   TOTAL          :                       ~54-84 KB
 *
 * This fits comfortably within the ESP32-S3's 512 KB SRAM even alongside
 * the existing AudioService and Wi-Fi stack.
 */

#include "speech_emotion.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <esp_heap_caps.h>

#if defined(__GNUC__) || defined(__clang__)
#include <alloca.h>
#endif

// If TFLite Micro is available in your build, uncomment:
// #include "tensorflow/lite/micro/micro_interpreter.h"
// #include "tensorflow/lite/micro/all_ops_resolver.h"

#define TAG "SER"

// =====================================================================
//  Utility: Hann window, Mel scale conversions
// =====================================================================

static inline float hz_to_mel(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static inline float mel_to_hz(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static void apply_hann_window(float* buf, int n) {
    for (int i = 0; i < n; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
        buf[i] *= w;
    }
}

// =====================================================================
//  Simple in-place real FFT (radix-2 DIT, power-of-two only)
//  For production, consider esp-dsp's dsps_fft2r_fc32.
// =====================================================================

static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

/**
 * Compute |FFT|^2 (power spectrum) of a real signal in-place.
 * Input:  real[fft_size]  (zero-padded if needed)
 * Output: power[fft_size/2 + 1] written into the same buffer.
 */
static void compute_power_spectrum(float* data, int fft_size) {
    // --- Bit-reversal permutation ---
    for (int i = 1, j = 0; i < fft_size; i++) {
        int bit = fft_size >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    // --- Cooley-Tukey butterfly (real-valued input trick) ---
    // A production system should use esp-dsp for hardware-accelerated FFT.

    // Placeholder: compute magnitude via DFT for small sizes
    // (replaced by esp-dsp in production build)
    float* power = data;    // overwrite in-place
    int half = fft_size / 2 + 1;

    static float* scratch = nullptr;
    if (!scratch) {
        scratch = (float*)heap_caps_malloc(fft_size * sizeof(float),
                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!scratch) scratch = (float*)malloc(fft_size * sizeof(float));
    }
    memcpy(scratch, data, fft_size * sizeof(float));

    for (int k = 0; k < half; k++) {
        float re = 0.0f, im = 0.0f;
        for (int n = 0; n < fft_size; n++) {
            float angle = -2.0f * M_PI * k * n / fft_size;
            re += scratch[n] * cosf(angle);
            im += scratch[n] * sinf(angle);
        }
        power[k] = re * re + im * im;
    }
}


// =====================================================================
//  Constructor / Destructor
// =====================================================================

SpeechEmotionAnalyser::SpeechEmotionAnalyser(const SERConfig& cfg)
    : cfg_(cfg),
      frame_buf_(nullptr), ring_buf_(nullptr),
      ring_write_pos_(0), ring_read_pos_(0), ring_size_(0),
      mel_filters_(nullptr), fft_size_(0),
      pitch_sum_(0), pitch_sq_sum_(0), energy_sum_(0), frame_count_(0),
      probs_({0.0f, 0.0f, 1.0f, 0.0f}),   // default = neutral
      ready_(false),
      tflite_model_buf_(nullptr), tflite_interpreter_(nullptr),
      use_tflite_(false)
{
}

SpeechEmotionAnalyser::~SpeechEmotionAnalyser() {
    free(frame_buf_);
    free(ring_buf_);
    if (mel_filters_) {
        for (int i = 0; i < cfg_.num_mel_filters; i++) free(mel_filters_[i]);
        free(mel_filters_);
    }
    free(tflite_model_buf_);
    // tflite_interpreter_ is arena-managed, no explicit free needed
}


// =====================================================================
//  Init — allocate buffers, build Mel filter bank
// =====================================================================

esp_err_t SpeechEmotionAnalyser::Init() {
    frame_samples_ = cfg_.sample_rate * cfg_.frame_size_ms / 1000;
    hop_samples_   = cfg_.sample_rate * cfg_.hop_size_ms   / 1000;
    fft_size_      = next_pow2(frame_samples_);

    ESP_LOGI(TAG, "Frame=%d samples, Hop=%d, FFT=%d",
             frame_samples_, hop_samples_, fft_size_);

    // Allocate frame buffer (windowed samples + zero-padding for FFT)
    frame_buf_ = (float*)calloc(fft_size_, sizeof(float));
    if (!frame_buf_) {
        ESP_LOGE(TAG, "Failed to allocate frame_buf_");
        return ESP_ERR_NO_MEM;
    }

    // Ring buffer: hold ~300 ms of audio (enough for 10 overlapping frames)
    ring_size_ = cfg_.sample_rate * 300 / 1000;
    ring_buf_  = (float*)calloc(ring_size_, sizeof(float));
    if (!ring_buf_) {
        ESP_LOGE(TAG, "Failed to allocate ring_buf_");
        return ESP_ERR_NO_MEM;
    }

    // ── Build triangular Mel filter bank ───────────────────────────
    int num_bins = fft_size_ / 2 + 1;
    float mel_low  = hz_to_mel(0.0f);
    float mel_high = hz_to_mel(cfg_.sample_rate / 2.0f);

    // Mel centre frequencies
    int n_filters = cfg_.num_mel_filters;
    float* mel_points = (float*)alloca((n_filters + 2) * sizeof(float));
    for (int i = 0; i < n_filters + 2; i++) {
        mel_points[i] = mel_to_hz(mel_low + (mel_high - mel_low) * i / (n_filters + 1));
    }

    // Convert to FFT bin indices
    int* bin_idx = (int*)alloca((n_filters + 2) * sizeof(int));
    for (int i = 0; i < n_filters + 2; i++) {
        bin_idx[i] = (int)floorf((fft_size_ + 1) * mel_points[i] / cfg_.sample_rate);
        if (bin_idx[i] >= num_bins) bin_idx[i] = num_bins - 1;
    }

    // Allocate and fill triangular filters
    mel_filters_ = (float**)calloc(n_filters, sizeof(float*));
    for (int m = 0; m < n_filters; m++) {
        mel_filters_[m] = (float*)calloc(num_bins, sizeof(float));
        for (int k = bin_idx[m]; k < bin_idx[m + 1]; k++) {
            int denom = bin_idx[m + 1] - bin_idx[m];
            mel_filters_[m][k] = denom > 0 ? (float)(k - bin_idx[m]) / denom : 0;
        }
        for (int k = bin_idx[m + 1]; k < bin_idx[m + 2]; k++) {
            int denom = bin_idx[m + 2] - bin_idx[m + 1];
            mel_filters_[m][k] = denom > 0 ? (float)(bin_idx[m + 2] - k) / denom : 0;
        }
    }

    ESP_LOGI(TAG, "Mel filter bank built: %d filters, %d bins", n_filters, num_bins);

    // ── Optional: load TFLite Micro model ──────────────────────────
    if (cfg_.tflite_model_path && strlen(cfg_.tflite_model_path) > 0) {
        ESP_LOGI(TAG, "TFLite model path: %s (loading not yet implemented)", cfg_.tflite_model_path);
        // TODO: Read model from SPIFFS, create interpreter with ~30 KB arena
        use_tflite_ = false;   // set to true once model is loaded
    }

    ESP_LOGI(TAG, "SER engine initialised (heuristic mode)");
    return ESP_OK;
}


// =====================================================================
//  FeedAudio — accumulate PCM, extract features, update probs
// =====================================================================

void SpeechEmotionAnalyser::FeedAudio(const int16_t* pcm, size_t num_samples) {
    // Convert int16 -> float and write into ring buffer
    for (size_t i = 0; i < num_samples; i++) {
        ring_buf_[ring_write_pos_] = (float)pcm[i] / 32768.0f;
        ring_write_pos_ = (ring_write_pos_ + 1) % ring_size_;
    }

    // Process complete frames (with overlap = frame - hop)
    while (true) {
        // Check if we have enough samples for one frame
        int avail = (ring_write_pos_ - ring_read_pos_ + ring_size_) % ring_size_;
        if (avail < frame_samples_) break;

        // Copy frame from ring buffer
        for (int j = 0; j < frame_samples_; j++) {
            int idx = (ring_read_pos_ + j) % ring_size_;
            frame_buf_[j] = ring_buf_[idx];
        }
        // Zero-pad remainder for FFT
        for (int j = frame_samples_; j < fft_size_; j++) {
            frame_buf_[j] = 0.0f;
        }

        // Pre-emphasis filter: y[n] = x[n] - alpha*x[n-1]
        for (int j = frame_samples_ - 1; j > 0; j--) {
            frame_buf_[j] -= cfg_.pre_emphasis * frame_buf_[j - 1];
        }

        // ── Feature 1: RMS energy ─────────────────────────────────
        float rms = ComputeRMS(frame_buf_, frame_samples_);
        energy_sum_ += rms;

        // ── Feature 2: Pitch (F0) via autocorrelation ─────────────
        float pitch = EstimatePitch(frame_buf_, frame_samples_);
        if (pitch > 0.0f) {
            pitch_sum_    += pitch;
            pitch_sq_sum_ += pitch * pitch;
        }

        // ── Feature 3: MFCC (for optional TFLite path) ───────────
        float mfcc[13] = {0};
        if (use_tflite_) {
            apply_hann_window(frame_buf_, frame_samples_);
            ComputeMFCC(frame_buf_, fft_size_, mfcc);
            RunTFLiteInference(mfcc, cfg_.num_mfcc);
        }

        frame_count_++;

        // Advance read pointer by hop_size
        ring_read_pos_ = (ring_read_pos_ + hop_samples_) % ring_size_;
    }

    // Update heuristic probabilities every 5 frames (~75 ms)
    if (frame_count_ > 0 && frame_count_ % 5 == 0 && !use_tflite_) {
        UpdateHeuristicProbs();
    }
}


// =====================================================================
//  Feature extraction primitives
// =====================================================================

float SpeechEmotionAnalyser::ComputeRMS(const float* frame, int len) const {
    float sum = 0.0f;
    for (int i = 0; i < len; i++) {
        sum += frame[i] * frame[i];
    }
    return sqrtf(sum / len);
}

float SpeechEmotionAnalyser::EstimatePitch(const float* frame, int len) const {
    /**
     * Autocorrelation-based pitch estimator.
     *
     * We compute the normalised autocorrelation R(tau) for lag values
     * corresponding to [pitch_min_hz, pitch_max_hz], then pick the
     * lag with the highest peak.  If the peak is below 0.3 (low
     * periodicity -> unvoiced), we return 0 (no pitch detected).
     */
    int lag_min = cfg_.sample_rate / cfg_.pitch_max_hz;
    int lag_max = cfg_.sample_rate / cfg_.pitch_min_hz;
    if (lag_max >= len) lag_max = len - 1;

    float best_corr = 0.0f;
    int   best_lag  = 0;

    // Energy normalisation term
    float e0 = 0.0f;
    for (int i = 0; i < len; i++) e0 += frame[i] * frame[i];
    if (e0 < 1e-10f) return 0.0f;       // silence

    for (int lag = lag_min; lag <= lag_max; lag++) {
        float corr = 0.0f;
        float e1   = 0.0f;
        for (int i = 0; i < len - lag; i++) {
            corr += frame[i] * frame[i + lag];
            e1   += frame[i + lag] * frame[i + lag];
        }
        float norm = sqrtf(e0 * e1);
        if (norm > 0) corr /= norm;

        if (corr > best_corr) {
            best_corr = corr;
            best_lag  = lag;
        }
    }

    // Periodicity threshold — below 0.3 is likely unvoiced (noise / breath)
    if (best_corr < 0.3f || best_lag == 0) return 0.0f;

    return (float)cfg_.sample_rate / (float)best_lag;
}

void SpeechEmotionAnalyser::ComputeMFCC(const float* frame, int len, float* mfcc_out) const {
    /**
     * MFCC extraction:
     *   1. Power spectrum via FFT
     *   2. Apply Mel filter bank
     *   3. Log compression
     *   4. DCT-II (Type 2 discrete cosine transform)
     */
    int num_bins = len / 2 + 1;

    // Make a working copy for FFT (frame_buf_ is already zero-padded)
    float* spec = (float*)alloca(len * sizeof(float));
    memcpy(spec, frame, len * sizeof(float));

    compute_power_spectrum(spec, len);

    // Apply Mel filters -> log Mel energies
    float* mel_energy = (float*)alloca(cfg_.num_mel_filters * sizeof(float));
    for (int m = 0; m < cfg_.num_mel_filters; m++) {
        float sum = 0.0f;
        for (int k = 0; k < num_bins; k++) {
            sum += spec[k] * mel_filters_[m][k];
        }
        mel_energy[m] = logf(sum + 1e-10f);     // log compression
    }

    // DCT-II -> MFCC
    for (int i = 0; i < cfg_.num_mfcc; i++) {
        float sum = 0.0f;
        for (int j = 0; j < cfg_.num_mel_filters; j++) {
            sum += mel_energy[j] * cosf(M_PI * i * (j + 0.5f) / cfg_.num_mel_filters);
        }
        mfcc_out[i] = sum;
    }
}


// =====================================================================
//  Heuristic emotion mapping (no TFLite required)
// =====================================================================

void SpeechEmotionAnalyser::UpdateHeuristicProbs() {
    /**
     * Rule-based mapping from prosodic features -> emotion probabilities.
     *
     * Mapping rationale (based on speech emotion literature):
     * +----------+----------------+--------------------------+
     * | Emotion  | Pitch (F0)     | Energy (RMS)             |
     * +----------+----------------+--------------------------+
     * | Happy    | High, variable | Medium-high              |
     * | Sad      | Low, flat      | Low                      |
     * | Neutral  | Medium         | Medium                   |
     * | Anger    | High, variable | High (often highest)     |
     * +----------+----------------+--------------------------+
     */

    if (frame_count_ < 3) return;       // too few frames to estimate

    float avg_energy = energy_sum_ / frame_count_;
    float avg_pitch  = (pitch_sum_ > 0) ? pitch_sum_ / frame_count_ : 0.0f;
    float pitch_var  = 0.0f;
    if (frame_count_ > 1 && pitch_sum_ > 0) {
        float mean_sq = pitch_sq_sum_ / frame_count_;
        float sq_mean = (pitch_sum_ / frame_count_) * (pitch_sum_ / frame_count_);
        pitch_var = sqrtf(fmaxf(0.0f, mean_sq - sq_mean));
    }

    // ── Compute unnormalised logits ────────────────────────────────
    float logits[kNumEmotions] = {0.0f};

    // --- SAD ---
    if (avg_pitch > 0 && avg_pitch < cfg_.pitch_low_thresh) logits[kEmotionSad] += 1.5f;
    if (avg_energy < cfg_.energy_low_thresh)                logits[kEmotionSad] += 1.5f;
    if (pitch_var < cfg_.pitch_var_thresh * 0.5f)           logits[kEmotionSad] += 0.8f;

    // --- HAPPY ---
    if (avg_pitch > cfg_.pitch_high_thresh)                 logits[kEmotionHappy] += 1.2f;
    if (avg_energy > cfg_.energy_low_thresh &&
        avg_energy < cfg_.energy_high_thresh)               logits[kEmotionHappy] += 0.8f;
    if (pitch_var > cfg_.pitch_var_thresh)                  logits[kEmotionHappy] += 1.0f;

    // --- ANGER ---
    if (avg_energy > cfg_.energy_high_thresh)               logits[kEmotionAnger] += 2.0f;
    if (avg_pitch > cfg_.pitch_high_thresh)                 logits[kEmotionAnger] += 0.8f;
    if (pitch_var > cfg_.pitch_var_thresh * 1.5f)           logits[kEmotionAnger] += 0.5f;

    // --- NEUTRAL ---
    logits[kEmotionNeutral] += 0.5f;     // slight prior towards neutral (elderly baseline)
    if (avg_pitch > cfg_.pitch_low_thresh &&
        avg_pitch < cfg_.pitch_high_thresh)                 logits[kEmotionNeutral] += 1.0f;
    if (avg_energy > cfg_.energy_low_thresh &&
        avg_energy < cfg_.energy_high_thresh)               logits[kEmotionNeutral] += 1.0f;

    // ── Softmax normalisation ──────────────────────────────────────
    float max_logit = *std::max_element(logits, logits + kNumEmotions);
    float exp_sum = 0.0f;
    float exp_vals[kNumEmotions];
    for (size_t i = 0; i < kNumEmotions; i++) {
        exp_vals[i] = expf(logits[i] - max_logit);
        exp_sum += exp_vals[i];
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < kNumEmotions; i++) {
        probs_[i] = exp_vals[i] / exp_sum;
    }
    ready_ = true;

    ESP_LOGD(TAG, "SER: pitch=%.0fHz(var=%.0f) energy=%.4f -> H=%.2f S=%.2f N=%.2f A=%.2f",
             avg_pitch, pitch_var, avg_energy,
             probs_[0], probs_[1], probs_[2], probs_[3]);
}


// =====================================================================
//  TFLite Micro inference (placeholder)
// =====================================================================

void SpeechEmotionAnalyser::RunTFLiteInference(const float* mfcc, int num_mfcc) {
    // TODO: When TFLite model is trained and exported:
    //   1. Quantise MFCC inputs to INT8
    //   2. Copy into interpreter input tensor
    //   3. Invoke()
    //   4. Read softmax output -> probs_
    ESP_LOGW(TAG, "TFLite inference not yet implemented -- using heuristic fallback");
    UpdateHeuristicProbs();
}


// =====================================================================
//  Public getters
// =====================================================================

std::array<float, kNumEmotions> SpeechEmotionAnalyser::GetEmotionProbs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return probs_;
}

bool SpeechEmotionAnalyser::IsReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ready_;
}

void SpeechEmotionAnalyser::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    pitch_sum_    = 0.0f;
    pitch_sq_sum_ = 0.0f;
    energy_sum_   = 0.0f;
    frame_count_  = 0;
    ready_        = false;
    probs_        = {0.0f, 0.0f, 1.0f, 0.0f};   // reset to neutral
    ring_read_pos_  = ring_write_pos_;            // discard buffered audio
}
