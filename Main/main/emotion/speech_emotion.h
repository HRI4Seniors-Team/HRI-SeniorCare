/**
 * @file  speech_emotion.h
 * @brief Lightweight Speech Emotion Recognition (SER) for ESP32-S3
 * ========================================================================
 * Project : SIEVOX — Multimodal Elderly Care Robot
 * Target  : ESP32-S3 (ESP-IDF v5.x, ~512 KB SRAM budget)
 * ========================================================================
 *
 * Strategy overview
 * -----------------
 * Full neural SER (e.g. wav2vec2) is infeasible on ESP32-S3.
 * We instead extract 3 hand-crafted prosodic features from the raw
 * PCM audio stream and map them to an emotion probability vector
 * using a small rule-based heuristic + optional TFLite Micro model.
 *
 * Feature set
 * -----------
 *   1. **Pitch (F0)**     — estimated via autocorrelation on 30 ms frames.
 *      Low/flat pitch -> sad;  high/variable pitch -> happy or anger.
 *   2. **Energy (RMS)**   — root mean square of the frame.
 *      Low energy -> sad;  high energy -> anger or happy.
 *   3. **MFCC[0..12]**    — 13 Mel-Frequency Cepstral Coefficients.
 *      Fed into a tiny 3-layer INT8 TFLite Micro classifier (optional).
 *
 * Output
 * ------
 * A normalised probability array over 4 emotions:
 *   [happy, sad, neutral, anger]
 * identical in label order to the Vision module.
 *
 * Integration
 * -----------
 * Instantiate `SpeechEmotionAnalyser` in application.cc, feed it PCM
 * frames from AudioService, and poll `GetEmotionProbs()` before each
 * D-S fusion cycle.
 */

#ifndef SPEECH_EMOTION_H
#define SPEECH_EMOTION_H

#include <array>
#include <cstdint>
#include <cstddef>
#include <mutex>

#include <esp_err.h>

// Number of discrete emotion categories (must match Vision side)
constexpr size_t kNumEmotions = 4;

// Emotion index constants — order matters, must be identical everywhere
enum EmotionIndex : size_t {
    kEmotionHappy   = 0,
    kEmotionSad     = 1,
    kEmotionNeutral = 2,
    kEmotionAnger   = 3,
};

/**
 * @brief Configuration for the SER feature extraction pipeline.
 */
struct SERConfig {
    int     sample_rate     = 16000;    ///< Expected input sample rate (Hz)
    int     frame_size_ms   = 30;       ///< Analysis frame length (ms)
    int     hop_size_ms     = 15;       ///< Hop between consecutive frames (ms)
    int     num_mfcc        = 13;       ///< Number of MFCC coefficients
    int     num_mel_filters = 26;       ///< Number of Mel filter-bank channels
    float   pre_emphasis    = 0.97f;    ///< Pre-emphasis filter coefficient

    // Pitch estimation
    int     pitch_min_hz    = 60;       ///< Minimum detectable F0 (elderly voice)
    int     pitch_max_hz    = 400;      ///< Maximum detectable F0

    // Heuristic thresholds (tuned for elderly Mandarin speakers)
    float   energy_low_thresh    = 0.015f;   ///< Below -> likely sad
    float   energy_high_thresh   = 0.08f;    ///< Above -> likely anger/happy
    float   pitch_low_thresh     = 120.0f;   ///< Below -> likely sad
    float   pitch_high_thresh    = 250.0f;   ///< Above -> likely happy/anger
    float   pitch_var_thresh     = 40.0f;    ///< Variability boundary

    // TFLite Micro model (optional — set path to "" to disable)
    const char* tflite_model_path = "";      ///< Path in SPIFFS / LittleFS
};


/**
 * @brief Lightweight speech emotion analyser for ESP32-S3.
 *
 * Thread-safe: internal mutex protects the probability vector so it can
 * be read from the main event loop while the audio task feeds frames.
 */
class SpeechEmotionAnalyser {
public:
    explicit SpeechEmotionAnalyser(const SERConfig& cfg = SERConfig{});
    ~SpeechEmotionAnalyser();

    // Non-copyable
    SpeechEmotionAnalyser(const SpeechEmotionAnalyser&) = delete;
    SpeechEmotionAnalyser& operator=(const SpeechEmotionAnalyser&) = delete;

    /**
     * @brief  Initialise DSP buffers, Mel filter bank, and (optionally)
     *         load the TFLite Micro model.
     * @return ESP_OK on success.
     */
    esp_err_t Init();

    /**
     * @brief  Feed a chunk of raw PCM-16 mono audio.
     *
     * Internally accumulates samples into frames, extracts features,
     * and updates the emotion probability vector.
     *
     * @param  pcm      Pointer to signed 16-bit PCM samples.
     * @param  num_samples  Number of samples (not bytes).
     */
    void FeedAudio(const int16_t* pcm, size_t num_samples);

    /**
     * @brief  Return the current emotion probability vector.
     *
     * Safe to call from any task.  Returns the most recent estimate.
     * Values sum to ~1.0.
     *
     * @return Array of [happy, sad, neutral, anger].
     */
    std::array<float, kNumEmotions> GetEmotionProbs() const;

    /**
     * @brief  Return true if sufficient audio has been analysed since
     *         the last Reset() to produce a meaningful estimate.
     */
    bool IsReady() const;

    /**
     * @brief  Clear internal accumulators (call at start of each
     *         utterance / VAD segment).
     */
    void Reset();

private:
    SERConfig cfg_;

    // DSP state
    int   frame_samples_;       ///< Samples per analysis frame
    int   hop_samples_;         ///< Samples per hop
    float* frame_buf_;          ///< Windowed frame buffer
    float* ring_buf_;           ///< Circular input accumulator
    int    ring_write_pos_;
    int    ring_read_pos_;
    int    ring_size_;

    // Mel filter bank (pre-computed)
    float** mel_filters_;       ///< [num_mel_filters x (fft_size/2 + 1)]
    int     fft_size_;

    // Running statistics over the current utterance
    float  pitch_sum_;
    float  pitch_sq_sum_;
    float  energy_sum_;
    int    frame_count_;

    // Output
    mutable std::mutex mutex_;
    std::array<float, kNumEmotions> probs_;
    bool ready_;

    // Internal DSP methods
    float EstimatePitch(const float* frame, int len) const;
    float ComputeRMS(const float* frame, int len) const;
    void  ComputeMFCC(const float* frame, int len, float* mfcc_out) const;
    void  UpdateHeuristicProbs();

    // Optional TFLite Micro
    void* tflite_model_buf_;    ///< Raw model bytes (heap-allocated)
    void* tflite_interpreter_;  ///< TfLiteMicro interpreter
    bool  use_tflite_;
    void  RunTFLiteInference(const float* mfcc, int num_mfcc);
};

#endif // SPEECH_EMOTION_H