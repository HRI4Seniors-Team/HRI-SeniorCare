/**
 * @file  ds_fusion_engine.h
 * @brief Dempster-Shafer Evidence Theory — Multimodal Emotion Fusion
 * ========================================================================
 * Project : SIEVOX — Multimodal Elderly Care Robot
 * Target  : ESP32-S3 (ESP-IDF v5.x)
 * ========================================================================
 *
 * Mathematical background
 * -----------------------
 * Dempster-Shafer (D-S) theory generalises Bayesian probability by
 * allowing belief to be assigned to *sets* of hypotheses rather than
 * only singletons.  For our use case the frame of discernment is:
 *
 *     Theta = { Happy, Sad, Neutral, Anger }
 *
 * Each sensor (Vision, Audio) produces a **Basic Probability Assignment**
 * (BPA, also called a mass function)  m : 2^Theta -> [0, 1]  where:
 *
 *     Sum m(A) = 1  for all A subset of Theta
 *     m(null)   = 0
 *
 * For simplicity we restrict BPAs to *singletons* plus the full set Theta
 * (representing "uncertainty"), which keeps the computation O(N^2) where
 * N = |Theta| = 4.
 *
 * Dempster's Rule of Combination (orthogonal sum):
 *
 *     m12(A) = [ Sum_{B n C = A} m1(B)*m2(C) ] / (1 - K)
 *
 * where the **conflict factor** K is:
 *
 *     K = Sum_{B n C = null} m1(B)*m2(C)
 *
 * K in [0, 1) measures how much the two sources disagree.
 * If K -> 1, the sources are almost fully contradictory and the fused
 * result becomes unreliable.  We handle this with a configurable
 * conflict threshold:
 *
 *   - K < threshold -> use standard Dempster's Rule
 *   - K >= threshold -> fall back to a weighted average (proportional
 *     to each sensor's historical reliability).
 *
 * This guards against the well-known "Zadeh's paradox" edge case
 * (e.g., face smiling but voice crying) which would otherwise produce
 * counter-intuitive results.
 *
 * The "uncertainty" mass m(Theta) is the complement of the sensor's total
 * confidence in its singleton assignments.  A low-confidence detector
 * (e.g., no face detected) naturally has high m(Theta), which means it
 * contributes less information to the fusion — exactly the desired
 * behaviour.
 */

#ifndef DS_FUSION_ENGINE_H
#define DS_FUSION_ENGINE_H

#include <array>
#include <cstdint>
#include <string>
#include <mutex>
#include "speech_emotion.h"   // for kNumEmotions, EmotionIndex

// =====================================================================
//  Data types
// =====================================================================

/**
 * @brief Basic Probability Assignment (BPA) for our frame of discernment.
 *
 * singletons[i] = mass assigned to emotion i   (i = 0..3)
 * uncertainty    = mass assigned to the full set Theta (don't know)
 *
 * Invariant: sum(singletons) + uncertainty == 1.0
 */
struct BPA {
    std::array<float, kNumEmotions> singletons = {0};
    float uncertainty = 1.0f;          ///< m(Theta), default = total ignorance

    /** Construct from a raw probability array + confidence factor. */
    static BPA FromProbArray(const std::array<float, kNumEmotions>& probs,
                             float confidence);
};

/**
 * @brief The fused result after applying Dempster's Rule.
 */
struct FusionResult {
    std::array<float, kNumEmotions> belief;     ///< Belief (Bel) per emotion
    float          conflict;                    ///< Normalized conflict C [0, 1]
    float          conflict_raw;                ///< Raw D-S conflict K
    float          conflict_max;                ///< Max reachable K under confidence discounts
    EmotionIndex   dominant;                    ///< argmax(belief)
    float          dominant_score;              ///< belief[dominant]
    bool           high_conflict;               ///< true if normalized C >= threshold
    int64_t        timestamp_ms;                ///< fusion wall-clock (ms)
};


// =====================================================================
//  Fusion Engine Configuration
// =====================================================================

struct DSFusionConfig {
    /** Conflict threshold above which Dempster's Rule is replaced by
     *  weighted averaging.  Typical range: 0.7 – 0.9. */
    float conflict_threshold = 0.85f;

    /** Maximum age (ms) of a Vision BPA before it's treated as stale
     *  and replaced with a uniform "uncertain" distribution. */
    int64_t vision_stale_timeout_ms = 2000;

    /** Same for Audio. */
    int64_t audio_stale_timeout_ms = 1500;

    /** Reliability weights for fallback averaging when K is high.
     *  Higher weight -> more trust in that modality.
     *  Need not sum to 1 (will be normalised internally). */
    float vision_reliability = 0.6f;
    float audio_reliability  = 0.4f;

    /** Overall confidence discount applied to each sensor's BPA.
     *  vision_confidence = 0.8 means 80% of the probability mass goes
     *  to singletons, 20% goes to m(Theta).
     *  Lower values make the sensor less influential in fusion. */
    float vision_confidence = 0.75f;
    float audio_confidence  = 0.70f;
};


// =====================================================================
//  Fusion Engine
// =====================================================================

/**
 * @brief Dempster-Shafer fusion engine for combining Vision and Audio
 *        emotion estimates on the ESP32-S3.
 *
 * Thread-safety: all public methods are protected by an internal mutex.
 * The Audio task can call UpdateAudio() concurrently with the UART RX
 * task calling UpdateVision(), while the main loop polls Fuse().
 */
class DSFusionEngine {
public:
    explicit DSFusionEngine(const DSFusionConfig& cfg = DSFusionConfig{});

    /** @brief  Reconfigure the fusion engine with a new config in-place.
     *
     *  This is the preferred way to apply a custom config after the engine
     *  has been default-constructed (avoids the std::mutex assignment issue).
     */
    void Reconfigure(const DSFusionConfig& cfg);

    /**
     * @brief  Update the Vision modality BPA.
     *
     * Called from the UART receive handler when a new JSON emotion
     * packet arrives from the K210.
     *
     * @param probs       Normalised probability array [happy, sad, neutral, anger]
     * @param face_detected  If false, confidence is reduced to near-zero.
     */
    void UpdateVision(const std::array<float, kNumEmotions>& probs,
                      bool face_detected);

    /**
     * @brief  Update the Audio modality BPA.
     *
     * Called from the audio processing task after SpeechEmotionAnalyser
     * produces a new estimate.
     *
     * @param probs  Normalised probability array [happy, sad, neutral, anger]
     */
    void UpdateAudio(const std::array<float, kNumEmotions>& probs);

    /**
     * @brief  Run Dempster's Rule of Combination and return the fused result.
     *
     * Handles stale data: if either modality is older than its timeout,
     * its BPA is replaced with maximum uncertainty.
     *
     * @return FusionResult with belief, conflict, and dominant emotion.
     */
    FusionResult Fuse();

    /** Get the last fused result without re-computing. */
    FusionResult GetLastResult() const;

    /** Return a human-readable emotion label for the given index. */
    static const char* EmotionLabel(EmotionIndex idx);

private:
    DSFusionConfig cfg_;

    mutable std::mutex mutex_;
    BPA      vision_bpa_;
    BPA      audio_bpa_;
    int64_t  vision_updated_ms_;
    int64_t  audio_updated_ms_;
    FusionResult last_result_;

    /** Core D-S combination of two BPAs. */
    FusionResult CombineBPAs(const BPA& m1, const BPA& m2) const;

    /** Weighted-average fallback for high-conflict cases. */
    FusionResult FallbackAverage(const BPA& vision, const BPA& audio) const;

    /** Current time in milliseconds (monotonic). */
    static int64_t NowMs();
};

#endif // DS_FUSION_ENGINE_H
