/**
 * @file  ds_fusion_engine.cc
 * @brief Dempster-Shafer Evidence Theory — Implementation
 * ========================================================================
 * Project : SIEVOX — Multimodal Elderly Care Robot
 * ========================================================================
 *
 * +----------------------------------------------------------------------+
 * |                   D-S COMBINATION — WORKED EXAMPLE                  |
 * +----------------------------------------------------------------------+
 * |                                                                      |
 * |  Frame of discernment Theta = {H, S, N, A}  (Happy,Sad,Neutral,Anger)
 * |                                                                      |
 * |  Vision BPA  m1:                                                     |
 * |      m1({H}) = 0.60   (face is smiling)                             |
 * |      m1({S}) = 0.05                                                 |
 * |      m1({N}) = 0.10                                                 |
 * |      m1({A}) = 0.00                                                 |
 * |      m1(Theta)   = 0.25   (25% uncertainty / low confidence)        |
 * |                                                                      |
 * |  Audio BPA  m2:                                                      |
 * |      m2({H}) = 0.10                                                 |
 * |      m2({S}) = 0.56   (voice is trembling / low pitch)              |
 * |      m2({N}) = 0.04                                                 |
 * |      m2({A}) = 0.00                                                 |
 * |      m2(Theta)   = 0.30                                             |
 * |                                                                      |
 * |  Step 1 — Compute pairwise products m1(B)*m2(C) for all B,C:       |
 * |                                                                      |
 * |           m2({H})  m2({S})  m2({N})  m2({A})  m2(Theta)           |
 * |  m1({H})   0.060    0.336X   0.024X   0.000    0.180               |
 * |  m1({S})   0.005X   0.028    0.002X   0.000    0.015               |
 * |  m1({N})   0.010X   0.056X   0.004    0.000    0.030               |
 * |  m1({A})   0.000    0.000    0.000    0.000    0.000               |
 * |  m1(Theta) 0.025    0.070    0.010    0.000    0.075               |
 * |                                                                      |
 * |  X = conflicting pair (B n C = null)                                 |
 * |                                                                      |
 * |  K = sum of all X cells = 0.336+0.024+0.005+0.002+0.010+0.056      |
 * |    = 0.433                                                           |
 * |                                                                      |
 * |  Step 2 — Sum agreeing masses for each singleton:                   |
 * |    raw({H}) = 0.060 + 0.180 + 0.025 = 0.265                        |
 * |    raw({S}) = 0.028 + 0.015 + 0.070 = 0.113                        |
 * |    raw({N}) = 0.004 + 0.030 + 0.010 = 0.044                        |
 * |    raw({A}) = 0.000                                                 |
 * |    raw(Theta)   = 0.075                                             |
 * |                                                                      |
 * |  Step 3 — Normalise by (1 - K):                                     |
 * |    m12({H}) = 0.265 / 0.567 = 0.467                                |
 * |    m12({S}) = 0.113 / 0.567 = 0.199                                |
 * |    m12({N}) = 0.044 / 0.567 = 0.078                                |
 * |    m12({A}) = 0.000 / 0.567 = 0.000                                |
 * |    m12(Theta)   = 0.075 / 0.567 = 0.132                            |
 * |                                                                      |
 * |  -> Despite the face smiling (H=0.60) and voice suggesting sadness  |
 * |    (S=0.56), the fused belief in Happiness (0.467) outweighs Sad    |
 * |    (0.199) because the vision module was more confident overall.     |
 * |    K=0.433 is moderate — the system trusts the fusion.              |
 * |                                                                      |
 * |  -> In the "hidden depression" scenario (elderly person masks with  |
 * |    a smile but voice trembles), a high K value (>0.85) triggers     |
 * |    our special handling: the system falls back to weighted average   |
 * |    and flags the *conflict itself* as clinically significant.       |
 * +----------------------------------------------------------------------+
 */

#include "ds_fusion_engine.h"

#include <algorithm>
#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "DS_FUSION"

// Emotion label strings (matches EmotionIndex order)
static const char* const EMOTION_LABELS[] = {
    "happy", "sad", "neutral", "anger"
};


// =====================================================================
//  BPA factory
// =====================================================================

BPA BPA::FromProbArray(const std::array<float, kNumEmotions>& probs,
                       float confidence) {
    /**
     * Convert a probability array into a BPA by scaling each
     * singleton mass by a confidence factor in [0, 1].
     *
     * The remaining mass (1 - confidence) is assigned to Theta,
     * representing "I'm not sure."
     *
     * This is the standard "simple support function" construction
     * commonly used in D-S applications.
     */
    BPA bpa;
    confidence = std::clamp(confidence, 0.0f, 1.0f);

    float total_prob = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        total_prob += probs[i];
    }

    // Guard against degenerate input
    if (total_prob < 1e-6f) {
        bpa.uncertainty = 1.0f;
        return bpa;
    }

    // Normalise probabilities (in case they don't sum to exactly 1)
    float scale = confidence / total_prob;
    for (size_t i = 0; i < kNumEmotions; i++) {
        bpa.singletons[i] = probs[i] * scale;
    }
    bpa.uncertainty = 1.0f - confidence;

    return bpa;
}


// =====================================================================
//  Constructor
// =====================================================================

DSFusionEngine::DSFusionEngine(const DSFusionConfig& cfg)
    : cfg_(cfg),
      vision_updated_ms_(0),
      audio_updated_ms_(0)
{
    // Start with maximum-uncertainty BPAs (no information)
    vision_bpa_.uncertainty = 1.0f;
    audio_bpa_.uncertainty  = 1.0f;

    // Default result: neutral with zero confidence
    last_result_.belief    = {0.0f, 0.0f, 1.0f, 0.0f};
    last_result_.conflict  = 0.0f;
    last_result_.conflict_raw = 0.0f;
    last_result_.conflict_max = 0.0f;
    last_result_.dominant  = kEmotionNeutral;
    last_result_.dominant_score = 0.0f;
    last_result_.high_conflict  = false;
    last_result_.timestamp_ms   = 0;

    ESP_LOGI(TAG, "D-S Fusion Engine created (conflict_thresh=%.2f, "
             "vision_conf=%.2f, audio_conf=%.2f)",
             cfg_.conflict_threshold, cfg_.vision_confidence,
             cfg_.audio_confidence);
}


// =====================================================================
//  Reconfigure
// =====================================================================

void DSFusionEngine::Reconfigure(const DSFusionConfig& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg_ = cfg;
    ESP_LOGI(TAG, "D-S Fusion Engine reconfigured (conflict_thresh=%.2f, "
             "vision_conf=%.2f, audio_conf=%.2f)",
             cfg_.conflict_threshold, cfg_.vision_confidence,
             cfg_.audio_confidence);
}


// =====================================================================
//  Update modalities
// =====================================================================

void DSFusionEngine::UpdateVision(
    const std::array<float, kNumEmotions>& probs,
    bool face_detected)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // If no face was detected, drastically reduce confidence
    // so that the Audio modality dominates the fusion.
    float conf = face_detected ? cfg_.vision_confidence : 0.05f;

    vision_bpa_ = BPA::FromProbArray(probs, conf);
    vision_updated_ms_ = NowMs();

    ESP_LOGD(TAG, "Vision BPA updated: H=%.3f S=%.3f N=%.3f A=%.3f Theta=%.3f (face=%d)",
             vision_bpa_.singletons[0], vision_bpa_.singletons[1],
             vision_bpa_.singletons[2], vision_bpa_.singletons[3],
             vision_bpa_.uncertainty, face_detected);
}

void DSFusionEngine::UpdateAudio(const std::array<float, kNumEmotions>& probs) {
    std::lock_guard<std::mutex> lock(mutex_);

    audio_bpa_ = BPA::FromProbArray(probs, cfg_.audio_confidence);
    audio_updated_ms_ = NowMs();

    ESP_LOGD(TAG, "Audio BPA updated: H=%.3f S=%.3f N=%.3f A=%.3f Theta=%.3f",
             audio_bpa_.singletons[0], audio_bpa_.singletons[1],
             audio_bpa_.singletons[2], audio_bpa_.singletons[3],
             audio_bpa_.uncertainty);
}


// =====================================================================
//  Fuse — main entry point
// =====================================================================

FusionResult DSFusionEngine::Fuse() {
    std::lock_guard<std::mutex> lock(mutex_);

    int64_t now = NowMs();

    // ── Handle stale data ──────────────────────────────────────────
    // If a modality hasn't been updated within its timeout window,
    // replace its BPA with total uncertainty (m(Theta) = 1).
    // This means a stale sensor effectively contributes nothing.

    BPA v_bpa = vision_bpa_;
    BPA a_bpa = audio_bpa_;

    if (vision_updated_ms_ == 0 ||
        (now - vision_updated_ms_) > cfg_.vision_stale_timeout_ms) {
        v_bpa = BPA{};   // m(Theta) = 1
        ESP_LOGD(TAG, "Vision BPA is stale -- using max uncertainty");
    }
    if (audio_updated_ms_ == 0 ||
        (now - audio_updated_ms_) > cfg_.audio_stale_timeout_ms) {
        a_bpa = BPA{};
        ESP_LOGD(TAG, "Audio BPA is stale -- using max uncertainty");
    }

    // ── Combine ────────────────────────────────────────────────────
    FusionResult result = CombineBPAs(v_bpa, a_bpa);
    result.timestamp_ms = now;

    last_result_ = result;
    return result;
}

FusionResult DSFusionEngine::GetLastResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_result_;
}


// =====================================================================
//  Core D-S combination (Dempster's Rule)
// =====================================================================

FusionResult DSFusionEngine::CombineBPAs(const BPA& m1, const BPA& m2) const {
    /**
     * Implements the orthogonal sum (Dempster's Rule of Combination)
     * for the restricted case where focal elements are either singletons
     * or the full frame Theta.
     *
     * For two BPAs m1, m2 with focal elements {thetai} and Theta:
     *
     *   raw(thetai) = m1(thetai)*m2(thetai)       // both agree on thetai
     *               + m1(thetai)*m2(Theta)         // m1 specific, m2 uncertain
     *               + m1(Theta)*m2(thetai)         // m2 specific, m1 uncertain
     *
     *   raw(Theta)  = m1(Theta)*m2(Theta)          // both uncertain
     *
     *   K           = Sum_{i!=j} m1(thetai)*m2(thetaj) // conflict
     *
     *   m12(A)      = raw(A) / (1 - K)             // normalisation
     *
     * Complexity: O(N^2) where N = |Theta| = 4 -> 16 multiplications.
     */

    FusionResult result;

    // ── Step 1: Compute conflict K ─────────────────────────────────
    float K = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        for (size_t j = 0; j < kNumEmotions; j++) {
            if (i != j) {
                K += m1.singletons[i] * m2.singletons[j];
            }
        }
    }

    float singleton_mass_1 = 0.0f;
    float singleton_mass_2 = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        singleton_mass_1 += m1.singletons[i];
        singleton_mass_2 += m2.singletons[i];
    }
    const float max_k = singleton_mass_1 * singleton_mass_2;
    const float normalized_conflict = (max_k > 1e-6f)
        ? std::clamp(K / max_k, 0.0f, 1.0f) : 0.0f;

    result.conflict_raw = K;
    result.conflict_max = max_k;
    result.conflict = normalized_conflict;
    result.high_conflict = (normalized_conflict >= cfg_.conflict_threshold);

    // ── High-conflict guard (Zadeh's paradox protection) ───────────
    if (result.high_conflict) {
        ESP_LOGW(TAG, "High conflict detected: C=%.4f (K=%.4f, Kmax=%.4f) >= %.2f -- "
                 "falling back to weighted average",
                 normalized_conflict, K, max_k, cfg_.conflict_threshold);
        return FallbackAverage(m1, m2);
    }

    // ── Step 2: Compute raw combined masses ────────────────────────
    float norm = 1.0f - K;
    if (norm < 1e-8f) {
        // Nearly total conflict — should have been caught above
        ESP_LOGE(TAG, "Normalisation factor near zero (K=%.6f)", K);
        return FallbackAverage(m1, m2);
    }

    float inv_norm = 1.0f / norm;
    float belief_sum = 0.0f;

    for (size_t i = 0; i < kNumEmotions; i++) {
        float raw_i = m1.singletons[i] * m2.singletons[i]   // agree on thetai
                    + m1.singletons[i] * m2.uncertainty       // m1 specific
                    + m1.uncertainty   * m2.singletons[i];    // m2 specific
        result.belief[i] = raw_i * inv_norm;
        belief_sum += result.belief[i];
    }

    // Residual uncertainty = m1(Theta)*m2(Theta) / (1-K)
    float fused_uncertainty = (m1.uncertainty * m2.uncertainty) * inv_norm;

    // ── Step 3: Distribute residual uncertainty proportionally ──────
    if (belief_sum > 1e-6f && fused_uncertainty > 1e-6f) {
        for (size_t i = 0; i < kNumEmotions; i++) {
            result.belief[i] += fused_uncertainty * (result.belief[i] / belief_sum);
        }
    } else if (fused_uncertainty > 0.5f) {
        // Both sensors are highly uncertain — distribute equally
        for (size_t i = 0; i < kNumEmotions; i++) {
            result.belief[i] = 1.0f / kNumEmotions;
        }
    }

    // ── Final normalisation to exactly 1.0 ─────────────────────────
    float total = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) total += result.belief[i];
    if (total > 0) {
        for (size_t i = 0; i < kNumEmotions; i++) result.belief[i] /= total;
    }

    // ── Find dominant emotion ──────────────────────────────────────
    result.dominant = kEmotionNeutral;
    result.dominant_score = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        if (result.belief[i] > result.dominant_score) {
            result.dominant_score = result.belief[i];
            result.dominant = static_cast<EmotionIndex>(i);
        }
    }

    ESP_LOGI(TAG, "Fused: H=%.3f S=%.3f N=%.3f A=%.3f | C=%.3f K=%.3f -> %s (%.1f%%)",
             result.belief[0], result.belief[1],
             result.belief[2], result.belief[3],
             normalized_conflict, K, EMOTION_LABELS[result.dominant],
             result.dominant_score * 100.0f);

    return result;
}


// =====================================================================
//  Fallback: weighted average for high-conflict scenarios
// =====================================================================

FusionResult DSFusionEngine::FallbackAverage(const BPA& vision,
                                              const BPA& audio) const {
    /**
     * When conflict is too high, Dempster's Rule can produce
     * counter-intuitive results (Zadeh's paradox).
     *
     * Instead, we compute a simple reliability-weighted average of
     * the two BPA singleton masses.  This is more conservative but
     * always produces a sensible output.
     *
     * Importantly, the high-conflict flag itself carries clinical
     * information: "face says happy, voice says sad" is a strong
     * indicator of masked/hidden depression — our core use case.
     */

    FusionResult result;
    float w_v = cfg_.vision_reliability;
    float w_a = cfg_.audio_reliability;
    float w_total = w_v + w_a;
    if (w_total < 1e-6f) w_total = 1.0f;

    float total = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        result.belief[i] = (vision.singletons[i] * w_v +
                            audio.singletons[i]  * w_a) / w_total;
        total += result.belief[i];
    }

    // Normalise to account for uncertainty mass that was discarded
    if (total > 0) {
        for (size_t i = 0; i < kNumEmotions; i++) {
            result.belief[i] /= total;
        }
    } else {
        // Both fully uncertain
        for (size_t i = 0; i < kNumEmotions; i++) {
            result.belief[i] = 1.0f / kNumEmotions;
        }
    }

    // Recompute conflict from original BPAs
    float K = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        for (size_t j = 0; j < kNumEmotions; j++) {
            if (i != j) K += vision.singletons[i] * audio.singletons[j];
        }
    }
    float singleton_mass_v = 0.0f;
    float singleton_mass_a = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        singleton_mass_v += vision.singletons[i];
        singleton_mass_a += audio.singletons[i];
    }
    const float max_k = singleton_mass_v * singleton_mass_a;
    const float normalized_conflict = (max_k > 1e-6f)
        ? std::clamp(K / max_k, 0.0f, 1.0f) : 0.0f;
    result.conflict_raw = K;
    result.conflict_max = max_k;
    result.conflict = normalized_conflict;
    result.high_conflict = (normalized_conflict >= cfg_.conflict_threshold);

    // Find dominant
    result.dominant = kEmotionNeutral;
    result.dominant_score = 0.0f;
    for (size_t i = 0; i < kNumEmotions; i++) {
        if (result.belief[i] > result.dominant_score) {
            result.dominant_score = result.belief[i];
            result.dominant = static_cast<EmotionIndex>(i);
        }
    }

    ESP_LOGW(TAG, "Fallback avg: H=%.3f S=%.3f N=%.3f A=%.3f | C=%.3f K=%.3f -> %s",
             result.belief[0], result.belief[1],
             result.belief[2], result.belief[3],
             normalized_conflict, K, EMOTION_LABELS[result.dominant]);

    return result;
}


// =====================================================================
//  Utility
// =====================================================================

const char* DSFusionEngine::EmotionLabel(EmotionIndex idx) {
    if (idx < kNumEmotions) return EMOTION_LABELS[idx];
    return "unknown";
}

int64_t DSFusionEngine::NowMs() {
    return esp_timer_get_time() / 1000;    // us -> ms
}
