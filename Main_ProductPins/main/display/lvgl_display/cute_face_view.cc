#include "cute_face_view.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_timer.h>

namespace {

constexpr int kRootSize = 210;
constexpr int kFaceBaseSize = 142;

// 眨眼时序（毫秒）
constexpr uint32_t kBlinkGapMinMs = 1800;
constexpr uint32_t kBlinkGapMaxMs = 6000;
constexpr uint32_t kDoubleGapMinMs = 140;
constexpr uint32_t kDoubleGapMaxMs = 260;
constexpr uint32_t kEyeLagMs = 14;      // 左眼比右眼晚一点点，避免同步眨眼

constexpr float kMoodBlendMs = 300.0f;
constexpr uint32_t kDtClampMs = 120;

// 打破镜像对称的固定偏移（像素）
constexpr int kLeftEyeBiasY = -1;
constexpr int kRightEyeBiasY = 1;
constexpr int kRightEyeBiasX = 1;
constexpr int kRightCheekBias = 2;

inline int clamp_int(int value, int lo, int hi) {
    return std::min(std::max(value, lo), hi);
}

inline float clamp01(float value) {
    return std::min(std::max(value, 0.0f), 1.0f);
}

inline float lerp_f(float a, float b, float t) {
    return a + (b - a) * t;
}

inline float smoothstep01(float t) {
    t = clamp01(t);
    return t * t * (3.0f - 2.0f * t);
}

inline float ease_out_quad(float t) {
    t = clamp01(t);
    return 1.0f - (1.0f - t) * (1.0f - t);
}

inline float wave01(uint32_t now_ms, float period_ms, float phase_ms = 0.0f) {
    const float phase = (static_cast<float>(now_ms) + phase_ms) / period_ms;
    const float raw = static_cast<float>(std::sin(phase * 6.2831853f));
    return 0.5f + 0.5f * raw;
}

inline uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// xorshift32：不依赖 libc rand，体积小且行为可复现
uint32_t g_rng_state = 0x9E3779B9u;

inline uint32_t RandU32() {
    g_rng_state ^= g_rng_state << 13;
    g_rng_state ^= g_rng_state >> 17;
    g_rng_state ^= g_rng_state << 5;
    return g_rng_state;
}

inline uint32_t RandRange(uint32_t lo, uint32_t hi) {
    if (hi <= lo) {
        return lo;
    }
    return lo + (RandU32() % (hi - lo + 1));
}

inline float RandF(float lo, float hi) {
    return lo + (hi - lo) * static_cast<float>(RandU32() >> 8) * (1.0f / 16777216.0f);
}

}  // namespace

CuteFaceView::CuteFaceView(lv_obj_t* parent)
    : parent_(parent) {
    g_rng_state = static_cast<uint32_t>(esp_timer_get_time()) | 1u;
    const uint32_t now = NowMs();
    now_ms_ = now;
    last_refresh_ms_ = now;
    blink_next_ms_ = now + RandRange(700, 2400);
    gaze_next_ms_ = now + RandRange(400, 1600);

    Build();

    // 首帧直接落在当前情绪的目标值上，避免从默认参数滑入
    mood_cur_ = ComputeMoodParams();
    mood_prev_ = mood_cur_;
    mood_view_ = mood_cur_;
    mood_blend_ = 1.0f;

    Refresh(now);
    timer_ = lv_timer_create([](lv_timer_t* timer) {
        auto* self = static_cast<CuteFaceView*>(lv_timer_get_user_data(timer));
        if (self != nullptr) {
            self->Refresh(NowMs());
        }
    }, timer_period_ms_, this);
}

CuteFaceView::~CuteFaceView() {
    if (timer_ != nullptr) {
        lv_timer_delete(timer_);
        timer_ = nullptr;
    }
    if (root_ != nullptr) {
        lv_obj_del(root_);
        root_ = nullptr;
    }
}

void CuteFaceView::SetVisible(bool visible) {
    visible_ = visible;
    if (root_ == nullptr) {
        return;
    }
    if (visible) {
        lv_obj_remove_flag(root_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(root_);
        if (timer_ != nullptr) {
            lv_timer_resume(timer_);
        }
    } else {
        lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
        if (timer_ != nullptr) {
            lv_timer_pause(timer_);
        }
    }
}

void CuteFaceView::SetEmotion(const char* emotion) {
    UpdateEmotion(emotion, NowMs());
}

void CuteFaceView::UpdateEmotion(const char* emotion, uint32_t now_ms) {
    const Mood next = ParseEmotion(emotion);
    if (next != mood_) {
        mood_ = next;
        mood_changed_ms_ = now_ms;
        // 从当前视觉状态出发过渡：中途再次切换也不会跳
        mood_prev_ = mood_view_;
        mood_cur_ = ComputeMoodParams();
        mood_blend_ = 0.0f;
        gaze_next_ms_ = now_ms + RandRange(120, 600);
    }
    Refresh(now_ms);
}

void CuteFaceView::SetListening(bool listening) {
    listening_ = listening;
    if (listening) {
        speaking_ = false;
        // 聆听时注视前方并暂停扫视，视觉上像在看着对方
        gaze_tx_ = 0.0f;
        gaze_ty_ = -0.5f;
        gaze_next_ms_ = NowMs() + RandRange(1500, 3000);
    }
    mood_prev_ = mood_view_;
    mood_cur_ = ComputeMoodParams();
    mood_blend_ = 0.0f;
    Refresh(NowMs());
}

void CuteFaceView::SetSpeaking(bool speaking) {
    speaking_ = speaking;
    if (speaking) {
        listening_ = false;
    }
    mood_prev_ = mood_view_;
    mood_cur_ = ComputeMoodParams();
    mood_blend_ = 0.0f;
    Refresh(NowMs());
}

void CuteFaceView::Build() {
    if (parent_ == nullptr) {
        return;
    }

    root_ = lv_obj_create(parent_);
    lv_obj_set_size(root_, kRootSize, kRootSize);
    lv_obj_center(root_);
    lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(root_, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_style_bg_opa(root_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(root_, 0, 0);
    lv_obj_set_style_pad_all(root_, 0, 0);
    lv_obj_set_style_radius(root_, 0, 0);

    hair_back_ = lv_obj_create(root_);
    hair_left_ = lv_obj_create(root_);
    hair_right_ = lv_obj_create(root_);
    hair_top_ = lv_obj_create(root_);
    left_ear_ = lv_obj_create(root_);
    right_ear_ = lv_obj_create(root_);
    neck_ = lv_obj_create(root_);
    coat_ = lv_obj_create(root_);
    collar_left_ = lv_obj_create(root_);
    collar_right_ = lv_obj_create(root_);

    face_ = lv_obj_create(root_);
    lv_obj_set_style_radius(face_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(face_, palette_.face, 0);
    lv_obj_set_style_bg_opa(face_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(face_, lv_color_hex(0xC78370), 0);
    lv_obj_set_style_border_width(face_, 2, 0);
    lv_obj_set_style_pad_all(face_, 0, 0);
    lv_obj_clear_flag(face_, LV_OBJ_FLAG_SCROLLABLE);

    bang_left_ = lv_obj_create(root_);
    bang_mid_ = lv_obj_create(root_);
    bang_right_ = lv_obj_create(root_);
    hair_highlight_ = lv_obj_create(root_);
    accessory_outer_ = lv_obj_create(root_);
    accessory_inner_ = lv_obj_create(root_);
    antenna_ = lv_obj_create(root_);
    antenna_tip_ = lv_obj_create(root_);

    left_eye_ = lv_obj_create(root_);
    right_eye_ = lv_obj_create(root_);
    left_pupil_ = lv_obj_create(root_);
    right_pupil_ = lv_obj_create(root_);
    left_brow_ = lv_obj_create(root_);
    right_brow_ = lv_obj_create(root_);

    // 新增：眉中段/内段与瞳孔高光
    left_brow_mid_ = lv_obj_create(root_);
    left_brow_tip_ = lv_obj_create(root_);
    right_brow_mid_ = lv_obj_create(root_);
    right_brow_tip_ = lv_obj_create(root_);
    left_hl_ = lv_obj_create(root_);
    right_hl_ = lv_obj_create(root_);

    mouth_outer_ = lv_obj_create(root_);
    mouth_inner_ = lv_obj_create(root_);
    tongue_ = lv_obj_create(root_);
    left_cheek_ = lv_obj_create(root_);
    right_cheek_ = lv_obj_create(root_);
    tear_ = lv_obj_create(root_);

    lv_obj_t* boxes[] = {
        hair_back_, hair_left_, hair_right_, hair_top_,
        bang_left_, bang_mid_, bang_right_, hair_highlight_,
        left_ear_, right_ear_, neck_, coat_, collar_left_, collar_right_,
        accessory_outer_, accessory_inner_, antenna_, antenna_tip_,
        left_eye_, right_eye_, left_pupil_, right_pupil_,
        left_brow_, left_brow_mid_, left_brow_tip_,
        right_brow_, right_brow_mid_, right_brow_tip_,
        left_hl_, right_hl_,
        mouth_outer_, mouth_inner_,
        tongue_, left_cheek_, right_cheek_, tear_,
    };

    for (lv_obj_t* obj : boxes) {
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(obj, 0, 0);
        lv_obj_set_style_pad_all(obj, 0, 0);
    }

    lv_obj_set_style_radius(hair_back_, 55, 0);
    lv_obj_set_style_radius(hair_left_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(hair_right_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(hair_top_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(bang_left_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(bang_mid_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(bang_right_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(hair_highlight_, LV_RADIUS_CIRCLE, 0);

    lv_obj_t* hair_parts[] = {
        hair_back_, hair_left_, hair_right_, hair_top_,
        bang_left_, bang_mid_, bang_right_
    };
    for (lv_obj_t* obj : hair_parts) {
        lv_obj_set_style_bg_color(obj, palette_.hair, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    }
    lv_obj_set_style_bg_color(hair_left_, palette_.hair_mid, 0);
    lv_obj_set_style_bg_color(hair_highlight_, palette_.hair_light, 0);
    lv_obj_set_style_bg_opa(hair_highlight_, 120, 0);

    lv_obj_set_style_radius(left_ear_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(right_ear_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_ear_, lv_color_hex(0xFFD6C4), 0);
    lv_obj_set_style_bg_color(right_ear_, lv_color_hex(0xFFD6C4), 0);
    lv_obj_set_style_bg_opa(left_ear_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_opa(right_ear_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(neck_, 10, 0);
    lv_obj_set_style_bg_color(neck_, lv_color_hex(0xFFD7C2), 0);
    lv_obj_set_style_bg_opa(neck_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(coat_, 18, 0);
    lv_obj_set_style_bg_color(coat_, palette_.coat, 0);
    lv_obj_set_style_bg_opa(coat_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(coat_, palette_.coat_shadow, 0);
    lv_obj_set_style_border_width(coat_, 2, 0);

    lv_obj_set_style_radius(collar_left_, 6, 0);
    lv_obj_set_style_radius(collar_right_, 6, 0);
    lv_obj_set_style_bg_color(collar_left_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(collar_right_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(collar_left_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_opa(collar_right_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(accessory_outer_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(accessory_outer_, palette_.accent, 0);
    lv_obj_set_style_bg_opa(accessory_outer_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(accessory_inner_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(accessory_inner_, palette_.hair, 0);
    lv_obj_set_style_bg_opa(accessory_inner_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(antenna_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(antenna_, lv_color_hex(0x7D5147), 0);
    lv_obj_set_style_bg_opa(antenna_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(antenna_tip_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(antenna_tip_, lv_color_hex(0xFF8F5A), 0);
    lv_obj_set_style_bg_opa(antenna_tip_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(left_eye_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_eye_, palette_.eye_white, 0);
    lv_obj_set_style_bg_opa(left_eye_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(right_eye_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(right_eye_, palette_.eye_white, 0);
    lv_obj_set_style_bg_opa(right_eye_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(left_pupil_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_pupil_, palette_.eye, 0);
    lv_obj_set_style_bg_opa(left_pupil_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(right_pupil_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(right_pupil_, palette_.eye, 0);
    lv_obj_set_style_bg_opa(right_pupil_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(left_brow_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_brow_, palette_.outline, 0);
    lv_obj_set_style_bg_opa(left_brow_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(right_brow_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(right_brow_, palette_.outline, 0);
    lv_obj_set_style_bg_opa(right_brow_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(left_hl_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_hl_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(left_hl_, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(right_hl_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(right_hl_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(right_hl_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(mouth_outer_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(mouth_outer_, palette_.mouth, 0);
    lv_obj_set_style_bg_opa(mouth_outer_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(mouth_inner_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(mouth_inner_, palette_.mouth_inner, 0);
    lv_obj_set_style_bg_opa(mouth_inner_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(tongue_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tongue_, palette_.blush, 0);
    lv_obj_set_style_bg_opa(tongue_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(left_cheek_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(left_cheek_, palette_.cheek, 0);
    lv_obj_set_style_bg_opa(left_cheek_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(right_cheek_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(right_cheek_, palette_.cheek, 0);
    lv_obj_set_style_bg_opa(right_cheek_, LV_OPA_COVER, 0);

    lv_obj_set_style_radius(tear_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tear_, palette_.tear, 0);
    lv_obj_set_style_bg_opa(tear_, LV_OPA_COVER, 0);
}

void CuteFaceView::Refresh(uint32_t now_ms) {
    if (root_ == nullptr) {
        return;
    }
    if (!visible_) {
        now_ms_ = now_ms;
        last_refresh_ms_ = now_ms;
        return;
    }

    now_ms_ = now_ms;
    uint32_t dt = (now_ms > last_refresh_ms_) ? (now_ms - last_refresh_ms_) : 0;
    if (dt > kDtClampMs) {
        dt = kDtClampMs;
    }
    last_refresh_ms_ = now_ms;

    UpdateBlink(now_ms);
    UpdateGaze(now_ms, dt);
    UpdateSpeechEnvelope(now_ms, dt);
    BlendMood(dt);

    const float blink_l = BlinkAmount(now_ms, kEyeLagMs);
    const float blink_r = BlinkAmount(now_ms, 0);
    const float blink = std::max(blink_l, blink_r);

    UpdatePalette();

    // 呼吸：两轴不同频，极缓，避免整脸像在匀速缩放
    const float br = static_cast<float>(std::sin(static_cast<float>(now_ms) / 1450.0f));
    const float br2 = static_cast<float>(std::sin(static_cast<float>(now_ms) / 930.0f + 0.8f));
    const int breathe = static_cast<int>(std::round(br * 1.6f));
    const int breathe_x = static_cast<int>(std::round(br2 * 0.7f));

    const int face_size = kFaceBaseSize + static_cast<int>(std::round(speech_env_ * 3.0f)) +
                          (listening_ ? 1 : 0) + breathe;
    const int face_x = (kRootSize - face_size) / 2 + breathe_x;
    const int face_y = (kRootSize - face_size) / 2;

    UpdateFaceGeometry(face_size, face_x, face_y);
    UpdateCharacterFrame(face_size, face_x, face_y, now_ms);

    UpdateEyes(face_x, face_y, face_size, blink_r);
    UpdateBrows(face_x, face_y, face_size, blink);
    UpdateCheeks(face_x, face_y, face_size);
    UpdateMouth(face_x, face_y, face_size, blink, speech_env_);
    UpdateTear(face_x, face_y, face_size, blink_l);

    // 眨眼与情绪过渡期间提高刷新率：33ms 只有两三帧，眨眼会顿
    const uint32_t want = (blink_phase_ != 0 || mood_blend_ < 1.0f) ? 16u : 33u;
    if (want != timer_period_ms_) {
        timer_period_ms_ = want;
        if (timer_ != nullptr) {
            lv_timer_set_period(timer_, timer_period_ms_);
        }
    }
}

void CuteFaceView::UpdateBlink(uint32_t now_ms) {
    switch (blink_phase_) {
        case 0:
            if (now_ms >= blink_next_ms_) {
                // 每次长度都不同，避免出现固定节拍
                blink_close_ms_ = 80 + RandRange(0, 50);
                blink_hold_ms_ = 24 + RandRange(0, 36);
                blink_open_ms_ = 96 + RandRange(0, 74);
                blink_phase_ = 1;
                blink_t_ms_ = now_ms;
            }
            break;
        case 1:
            if (now_ms - blink_t_ms_ >= blink_close_ms_) {
                blink_phase_ = 2;
                blink_t_ms_ += blink_close_ms_;
            }
            break;
        case 2:
            if (now_ms - blink_t_ms_ >= blink_hold_ms_) {
                blink_phase_ = 3;
                blink_t_ms_ += blink_hold_ms_;
            }
            break;
        case 3:
            if (now_ms - blink_t_ms_ >= blink_open_ms_ + kEyeLagMs) {
                blink_phase_ = 0;
                blink_t_ms_ = now_ms;
                if (blink_double_pending_) {
                    blink_double_pending_ = false;
                    blink_next_ms_ = blink_double_ms_;
                } else if (RandRange(0, 99) < 14) {
                    // 约 14% 概率补一次快速连眨
                    blink_double_pending_ = true;
                    blink_double_ms_ = now_ms + RandRange(kDoubleGapMinMs, kDoubleGapMaxMs);
                    blink_next_ms_ = blink_double_ms_;
                } else {
                    blink_next_ms_ = now_ms + RandRange(kBlinkGapMinMs, kBlinkGapMaxMs);
                }
            }
            break;
        default:
            blink_phase_ = 0;
            break;
    }
}

float CuteFaceView::BlinkAmount(uint32_t now_ms, uint32_t lag_ms) const {
    if (blink_phase_ == 0) {
        return 0.0f;
    }
    // 把时间轴整体延后 lag_ms，就得到另一只眼的滞后曲线
    const uint32_t t = (now_ms >= lag_ms) ? (now_ms - lag_ms) : 0;
    const uint32_t e = (t >= blink_t_ms_) ? (t - blink_t_ms_) : 0;
    switch (blink_phase_) {
        case 1:
            return smoothstep01(static_cast<float>(e) / static_cast<float>(blink_close_ms_));
        case 2:
            return 1.0f;
        case 3:
            return 1.0f - ease_out_quad(static_cast<float>(e) / static_cast<float>(blink_open_ms_));
        default:
            return 0.0f;
    }
}

void CuteFaceView::UpdateGaze(uint32_t now_ms, uint32_t dt) {
    if (now_ms >= gaze_next_ms_) {
        gaze_next_ms_ = now_ms + RandRange(1300, 4300);
        const float ang = RandF(0.0f, 6.2831853f);
        const float r = RandF(0.45f, 1.0f);
        gaze_tx_ = std::cos(ang) * 3.6f * r;
        gaze_ty_ = std::sin(ang) * 1.9f * r;
    }
    // 快速扫视：位移约 75ms 内完成，之后靠微漂移维持
    const float k = clamp01(static_cast<float>(dt) / 75.0f);
    gaze_x_ = lerp_f(gaze_x_, gaze_tx_, k);
    gaze_y_ = lerp_f(gaze_y_, gaze_ty_, k);
    gaze_drift_x_ = 0.45f * static_cast<float>(std::sin(static_cast<float>(now_ms) / 910.0f));
    gaze_drift_y_ = 0.30f * static_cast<float>(std::sin(static_cast<float>(now_ms) / 1370.0f + 1.7f));
}

void CuteFaceView::UpdateSpeechEnvelope(uint32_t now_ms, uint32_t dt) {
    float target = 0.0f;
    if (speaking_) {
        // 两个不同周期叠加 + 抖动，避免等周期开合的机械感
        const float a = wave01(now_ms, 190.0f);
        const float b = wave01(now_ms, 137.0f, 61.0f);
        target = clamp01(0.28f + 0.42f * a + 0.30f * b + RandF(-0.12f, 0.12f));
    } else if (listening_) {
        target = 0.06f + 0.05f * wave01(now_ms, 900.0f);
    }
    // 张开快、闭合慢
    const float tau = (target > speech_env_) ? 60.0f : 160.0f;
    speech_env_ = lerp_f(speech_env_, target, clamp01(static_cast<float>(dt) / tau));
}

void CuteFaceView::BlendMood(uint32_t dt) {
    if (mood_blend_ >= 1.0f) {
        mood_view_ = mood_cur_;
        return;
    }
    mood_blend_ = clamp01(mood_blend_ + static_cast<float>(dt) / kMoodBlendMs);
    if (mood_blend_ >= 1.0f) {
        mood_view_ = mood_cur_;
        return;
    }
    mood_view_ = BlendParams(mood_prev_, mood_cur_, smoothstep01(mood_blend_));
}

CuteFaceView::MoodParams CuteFaceView::ComputeMoodParams() const {
    MoodParams p{};
    // 中性基线
    p.eye_w = 42.0f;
    p.eye_h = 38.0f;
    p.pupil = 15.0f;
    p.eye_dy = 0.0f;
    p.brow_dy = 0.0f;
    p.brow_w = 30.0f;
    p.brow_thick = 5.0f;
    p.brow_arch = 3.0f;
    p.brow_tip = -1.0f;
    p.mouth_w = 44.0f;
    p.mouth_h = 14.0f;
    p.inner_h = 6.0f;
    p.cheek_w = 20.0f;
    p.cheek_h = 12.0f;
    p.cheek_opa = 70.0f;

    switch (mood_) {
        case Mood::Happy:
            p.eye_w = 43.0f; p.eye_h = 24.0f; p.pupil = 14.0f; p.eye_dy = -1.0f;
            p.brow_dy = -5.0f; p.brow_w = 26.0f; p.brow_arch = 4.0f; p.brow_tip = -3.0f;
            p.mouth_w = 56.0f; p.mouth_h = 20.0f; p.inner_h = 9.0f;
            p.cheek_w = 25.0f; p.cheek_h = 14.0f; p.cheek_opa = 165.0f;
            break;
        case Mood::Sad:
            p.eye_w = 41.0f; p.eye_h = 35.0f; p.pupil = 14.0f; p.eye_dy = 2.0f;
            p.brow_dy = 3.0f; p.brow_arch = 1.0f; p.brow_tip = -6.0f;
            p.mouth_w = 32.0f; p.mouth_h = 12.0f; p.inner_h = 5.0f;
            p.cheek_w = 18.0f; p.cheek_h = 11.0f; p.cheek_opa = 85.0f;
            break;
        case Mood::Angry:
            p.eye_w = 42.0f; p.eye_h = 29.0f; p.pupil = 14.0f; p.eye_dy = 1.0f;
            p.brow_dy = 2.0f; p.brow_arch = 1.0f; p.brow_tip = 5.0f; p.brow_thick = 6.0f;
            p.mouth_w = 42.0f; p.mouth_h = 10.0f; p.inner_h = 4.0f;
            p.cheek_opa = 120.0f;
            break;
        case Mood::Sleepy:
            p.eye_w = 38.0f; p.eye_h = 14.0f; p.pupil = 7.0f; p.eye_dy = 7.0f;
            p.brow_dy = 4.0f; p.brow_w = 24.0f; p.brow_thick = 4.0f;
            p.brow_arch = 0.0f; p.brow_tip = 0.0f;
            p.mouth_w = 36.0f; p.mouth_h = 8.0f; p.inner_h = 4.0f;
            p.cheek_w = 19.0f; p.cheek_h = 11.0f; p.cheek_opa = 80.0f;
            break;
        case Mood::Surprised:
            p.eye_w = 45.0f; p.eye_h = 42.0f; p.pupil = 16.0f; p.eye_dy = -2.0f;
            p.brow_dy = -7.0f; p.brow_arch = 5.0f; p.brow_tip = -2.0f;
            p.mouth_w = 46.0f; p.mouth_h = 18.0f; p.inner_h = 11.0f;
            p.cheek_w = 22.0f; p.cheek_h = 13.0f; p.cheek_opa = 130.0f;
            break;
        case Mood::Neutral:
        default:
            break;
    }

    if (listening_) {
        p.eye_w += 1.0f;
        p.eye_h += 2.0f;
        p.pupil += 1.0f;
        p.brow_dy -= 3.0f;
        p.mouth_w += 3.0f;
        p.mouth_h += 2.0f;
        p.inner_h += 3.0f;
        p.cheek_opa += 20.0f;
    }
    return p;
}

CuteFaceView::MoodParams CuteFaceView::BlendParams(const MoodParams& a, const MoodParams& b, float t) {
    MoodParams r{};
    r.eye_w = lerp_f(a.eye_w, b.eye_w, t);
    r.eye_h = lerp_f(a.eye_h, b.eye_h, t);
    r.pupil = lerp_f(a.pupil, b.pupil, t);
    r.eye_dy = lerp_f(a.eye_dy, b.eye_dy, t);
    r.brow_dy = lerp_f(a.brow_dy, b.brow_dy, t);
    r.brow_w = lerp_f(a.brow_w, b.brow_w, t);
    r.brow_thick = lerp_f(a.brow_thick, b.brow_thick, t);
    r.brow_arch = lerp_f(a.brow_arch, b.brow_arch, t);
    r.brow_tip = lerp_f(a.brow_tip, b.brow_tip, t);
    r.mouth_w = lerp_f(a.mouth_w, b.mouth_w, t);
    r.mouth_h = lerp_f(a.mouth_h, b.mouth_h, t);
    r.inner_h = lerp_f(a.inner_h, b.inner_h, t);
    r.cheek_w = lerp_f(a.cheek_w, b.cheek_w, t);
    r.cheek_h = lerp_f(a.cheek_h, b.cheek_h, t);
    r.cheek_opa = lerp_f(a.cheek_opa, b.cheek_opa, t);
    return r;
}

void CuteFaceView::UpdatePalette() {
    if (mood_ == Mood::Angry) {
        palette_.face = lv_color_hex(0xFFD9C9);
        palette_.cheek = lv_color_hex(0xEF8B94);
        palette_.accent = lv_color_hex(0xFF735D);
    } else if (mood_ == Mood::Sad) {
        palette_.face = lv_color_hex(0xFFE0D2);
        palette_.cheek = lv_color_hex(0xC8DDF2);
        palette_.accent = lv_color_hex(0x82B8F2);
    } else if (mood_ == Mood::Happy) {
        palette_.face = lv_color_hex(0xFFE4CF);
        palette_.cheek = lv_color_hex(0xF5A0A8);
        palette_.accent = lv_color_hex(0xFF914D);
    } else if (mood_ == Mood::Surprised) {
        palette_.face = lv_color_hex(0xFFE8D5);
        palette_.cheek = lv_color_hex(0xF6A6B6);
        palette_.accent = lv_color_hex(0xFFB44F);
    } else {
        palette_.face = lv_color_hex(0xFFE4CF);
        palette_.cheek = lv_color_hex(0xEFA5AD);
        palette_.accent = lv_color_hex(0xFF914D);
    }

    if (face_ != nullptr) {
        lv_obj_set_style_bg_color(face_, palette_.face, 0);
        lv_obj_set_style_border_color(face_, palette_.outline, 0);
    }
    if (left_cheek_ != nullptr) {
        lv_obj_set_style_bg_color(left_cheek_, palette_.cheek, 0);
        lv_obj_set_style_bg_color(right_cheek_, palette_.cheek, 0);
    }
    if (accessory_outer_ != nullptr) {
        lv_obj_set_style_bg_color(accessory_outer_, palette_.accent, 0);
        lv_obj_set_style_bg_color(antenna_tip_, palette_.accent, 0);
    }
}

void CuteFaceView::UpdateCharacterFrame(int face_size, int face_x, int face_y, uint32_t now_ms) {
    const int breathe = static_cast<int>(std::round((wave01(now_ms, 1800.0f) - 0.5f) * 2.0f));
    const int hair_x = face_x - 15;
    const int hair_y = face_y - 17 + breathe;
    const int hair_w = face_size + 30;
    const int hair_h = face_size + 32;

    lv_obj_set_pos(hair_back_, hair_x, hair_y);
    lv_obj_set_size(hair_back_, hair_w, hair_h);
    lv_obj_set_style_radius(hair_back_, 56, 0);

    lv_obj_set_pos(hair_left_, hair_x - 2, hair_y + 28);
    lv_obj_set_size(hair_left_, 55, 94);
    lv_obj_set_pos(hair_right_, hair_x + hair_w - 48, hair_y + 23);
    lv_obj_set_size(hair_right_, 52, 94);
    lv_obj_set_pos(hair_top_, hair_x + 29, hair_y - 4);
    lv_obj_set_size(hair_top_, 96, 58);

    lv_obj_set_pos(left_ear_, face_x - 8, face_y + face_size * 42 / 100);
    lv_obj_set_size(left_ear_, 20, 28);
    lv_obj_set_pos(right_ear_, face_x + face_size - 12, face_y + face_size * 42 / 100);
    lv_obj_set_size(right_ear_, 20, 28);

    lv_obj_set_pos(neck_, face_x + face_size / 2 - 15, face_y + face_size - 9);
    lv_obj_set_size(neck_, 30, 35);
    lv_obj_set_pos(coat_, face_x + face_size / 2 - 58, face_y + face_size + 18);
    lv_obj_set_size(coat_, 116, 54);
    lv_obj_set_pos(collar_left_, face_x + face_size / 2 - 30, face_y + face_size + 21);
    lv_obj_set_size(collar_left_, 24, 27);
    lv_obj_set_pos(collar_right_, face_x + face_size / 2 + 6, face_y + face_size + 21);
    lv_obj_set_size(collar_right_, 24, 27);

    lv_obj_set_pos(bang_left_, face_x + 15, face_y - 5);
    lv_obj_set_size(bang_left_, 50, 42);
    lv_obj_set_pos(bang_mid_, face_x + 45, face_y - 9);
    lv_obj_set_size(bang_mid_, 54, 47);
    lv_obj_set_pos(bang_right_, face_x + 82, face_y - 3);
    lv_obj_set_size(bang_right_, 45, 40);
    lv_obj_set_pos(hair_highlight_, hair_x + 22, hair_y + 17);
    lv_obj_set_size(hair_highlight_, 27, 10);

    const int acc_x = face_x + face_size - 19;
    const int acc_y = face_y - 8 + breathe;
    lv_obj_set_pos(accessory_outer_, acc_x, acc_y);
    lv_obj_set_size(accessory_outer_, 34, 42);
    lv_obj_set_pos(accessory_inner_, acc_x + 7, acc_y + 7);
    lv_obj_set_size(accessory_inner_, 20, 28);
    lv_obj_set_pos(antenna_, acc_x + 26, acc_y - 11);
    lv_obj_set_size(antenna_, 6, 22);
    lv_obj_set_pos(antenna_tip_, acc_x + 29, acc_y - 15);
    lv_obj_set_size(antenna_tip_, 8, 8);
}

void CuteFaceView::UpdateFaceGeometry(int face_size, int face_x, int face_y) {
    if (face_ == nullptr) {
        return;
    }
    lv_obj_set_size(face_, face_size, face_size);
    lv_obj_set_pos(face_, face_x, face_y);
    lv_obj_set_style_radius(face_, LV_RADIUS_CIRCLE, 0);
}

void CuteFaceView::DrawEye(lv_obj_t* eye, lv_obj_t* pupil, lv_obj_t* highlight,
                           int cx, int cy, int w, int h, int pupil_size,
                           float blink, int converge) {
    // 挤压：闭合时变矮的同时变宽，中心基本不动，并带一点下垂
    const float open = clamp01(1.0f - blink);
    const int droop = static_cast<int>(std::round(blink * 2.0f));
    int eye_h = static_cast<int>(std::round(static_cast<float>(h) * (0.07f + 0.93f * open)));
    eye_h = std::max(3, eye_h);
    const int eye_w = std::max(6, static_cast<int>(std::round(static_cast<float>(w) * (1.0f + 0.09f * blink))));
    const int eye_x = cx - eye_w / 2;
    const int eye_y = cy - eye_h / 2 + droop;

    lv_obj_set_pos(eye, eye_x, eye_y);
    lv_obj_set_size(eye, eye_w, eye_h);
    lv_obj_set_style_radius(eye, LV_RADIUS_CIRCLE, 0);

    const bool bright = (mood_ == Mood::Surprised) || listening_;
    const lv_color_t white = bright ? lv_color_hex(0xFFFDF8) : palette_.eye_white;
    lv_obj_set_style_bg_color(eye, white, 0);
    lv_obj_set_style_bg_opa(eye, LV_OPA_COVER, 0);

    // 瞳孔位置 = 扫视目标 + 微漂移 + 会聚偏置，并夹在眼白内
    const int gx = static_cast<int>(std::round(gaze_x_ + gaze_drift_x_)) + converge;
    const int gy = static_cast<int>(std::round(gaze_y_ + gaze_drift_y_));
    const int max_dx = std::max(0, (eye_w - pupil_size) / 2 - 2);
    const int max_dy = std::max(0, (eye_h - pupil_size) / 2 - 1);
    const int px = eye_x + eye_w / 2 - pupil_size / 2 + clamp_int(gx, -max_dx, max_dx);
    const int py = eye_y + eye_h / 2 - pupil_size / 2 + clamp_int(gy, -max_dy, max_dy);

    lv_obj_set_pos(pupil, px, py);
    lv_obj_set_size(pupil, pupil_size, pupil_size);
    lv_obj_set_style_radius(pupil, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(pupil, bright ? palette_.eye_glow : palette_.eye, 0);
    lv_obj_set_style_bg_opa(pupil, LV_OPA_COVER, 0);

    const bool show = (blink < 0.72f);
    SetVisibleObj(pupil, show);

    // 高光：钉在瞳孔左上偏内，不居中
    const int hl_size = std::max(3, pupil_size / 3);
    const int hl_x = px + std::max(1, pupil_size / 6);
    const int hl_y = py + std::max(1, pupil_size / 6);
    lv_obj_set_pos(highlight, hl_x, hl_y);
    lv_obj_set_size(highlight, hl_size, hl_size);
    SetVisibleObj(highlight, show);
}

void CuteFaceView::DrawBrow(lv_obj_t* outer, lv_obj_t* mid, lv_obj_t* tip,
                            int cx, int y, int seg_w, int thick,
                            int arch, int tip_dy) {
    // 三段重叠拼拱形：外端 → 中段抬高 → 内端按情绪偏移
    const int step = std::max(4, seg_w - 2);
    const int y_outer = y;
    const int y_mid = y - arch;
    const int y_tip = y + tip_dy;
    const bool hide_sleepy = (mood_ == Mood::Sleepy) && !listening_;

    lv_obj_t* segs[] = {outer, mid, tip};
    lv_obj_set_pos(outer, cx - step - seg_w / 2, y_outer);
    lv_obj_set_pos(mid, cx - seg_w / 2, y_mid);
    lv_obj_set_pos(tip, cx + step - seg_w / 2, y_tip);

    for (lv_obj_t* obj : segs) {
        lv_obj_set_size(obj, seg_w, thick);
        lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(obj, palette_.outline, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        SetVisibleObj(obj, !hide_sleepy);
    }
}

void CuteFaceView::UpdateEyes(int face_x, int face_y, int face_size, float blink) {
    // 传进来的 blink 是右眼量；左眼单独算，产生 14ms 的错峰
    const float blink_r = blink;
    const float blink_l = BlinkAmount(now_ms_, kEyeLagMs);

    const int cx = face_x + face_size / 2;
    const int eye_cy = face_y + face_size * 41 / 100 + static_cast<int>(std::round(mood_view_.eye_dy));
    const int eye_w = static_cast<int>(std::round(mood_view_.eye_w));
    const int eye_h = static_cast<int>(std::round(mood_view_.eye_h));
    const int pupil = std::max(3, static_cast<int>(std::round(mood_view_.pupil)));
    const int dx = face_size * 19 / 100;

    DrawEye(left_eye_, left_pupil_, left_hl_, cx - dx, eye_cy + kLeftEyeBiasY,
            eye_w, eye_h, pupil, blink_l, +1);
    DrawEye(right_eye_, right_pupil_, right_hl_, cx + dx + kRightEyeBiasX, eye_cy + kRightEyeBiasY,
            eye_w, eye_h, pupil, blink_r, -1);
}

void CuteFaceView::UpdateBrows(int face_x, int face_y, int face_size, float blink) {
    const int cx = face_x + face_size / 2;
    const int base_y = face_y + face_size * 21 / 100 +
                       static_cast<int>(std::round(mood_view_.brow_dy)) -
                       (blink > 0.55f ? 1 : 0);
    const int thick = std::max(3, static_cast<int>(std::round(mood_view_.brow_thick)));
    const int span = static_cast<int>(std::round(mood_view_.brow_w));
    const int seg_w = std::max(6, span / 3 + 2);
    const int arch = static_cast<int>(std::round(mood_view_.brow_arch));
    const int tip = static_cast<int>(std::round(mood_view_.brow_tip));
    const int dx = face_size * 19 / 100;

    DrawBrow(left_brow_, left_brow_mid_, left_brow_tip_, cx - dx, base_y,
             seg_w, thick, arch, tip);
    // 右侧整体下移 1px、内端再偏 1px，避免左右镜像
    DrawBrow(right_brow_, right_brow_mid_, right_brow_tip_, cx + dx, base_y + 1,
             seg_w, thick, arch, tip + 1);
}

void CuteFaceView::UpdateMouth(int face_x, int face_y, int face_size, float blink, float speech) {
    const int cx = face_x + face_size / 2;
    const float env = speech;
    const bool happy = mood_ == Mood::Happy;
    const bool sad = mood_ == Mood::Sad;
    const bool angry = mood_ == Mood::Angry;

    const int outer_w = static_cast<int>(std::round(mood_view_.mouth_w + env * 4.0f));
    const int outer_h = static_cast<int>(std::round(mood_view_.mouth_h + env * 6.0f));
    const int inner_w = std::max(6, outer_w - 12);
    const int inner_h = std::max(3, static_cast<int>(std::round(mood_view_.inner_h + env * 10.0f)));
    const int center_y = face_y + face_size * 71 / 100 +
                         static_cast<int>(std::round((env - 0.3f) * 3.0f));

    const int mouth_x = cx - outer_w / 2;
    const int mouth_y = center_y - outer_h / 2;

    lv_obj_set_pos(mouth_outer_, mouth_x, mouth_y);
    lv_obj_set_size(mouth_outer_, outer_w, outer_h);
    lv_obj_set_style_radius(mouth_outer_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(mouth_outer_, palette_.mouth, 0);

    const int inner_x = mouth_x + (outer_w - inner_w) / 2;
    int inner_y = mouth_y + (outer_h - inner_h) / 2;
    if (happy) {
        inner_y += 2;
    } else if (sad) {
        inner_y -= 2;
    } else if (angry) {
        inner_y += 1;
    }
    if (blink > 0.4f) {
        inner_y += 1;
    }

    lv_obj_set_pos(mouth_inner_, inner_x, inner_y);
    lv_obj_set_size(mouth_inner_, inner_w, inner_h);
    lv_obj_set_style_radius(mouth_inner_, LV_RADIUS_CIRCLE, 0);

    if (speaking_ && env > 0.25f) {
        lv_obj_set_style_bg_color(mouth_inner_, lv_color_hex(0x8B2530), 0);
        SetVisibleObj(tongue_, inner_h > 10);
    } else if (happy) {
        lv_obj_set_style_bg_color(mouth_inner_, lv_color_hex(0xFFF2E6), 0);
        SetVisibleObj(tongue_, false);
    } else if (sad) {
        lv_obj_set_style_bg_color(mouth_inner_, lv_color_hex(0xFFF7F0), 0);
        SetVisibleObj(tongue_, false);
    } else if (angry) {
        lv_obj_set_style_bg_color(mouth_inner_, lv_color_hex(0xFFD6D6), 0);
        SetVisibleObj(tongue_, false);
    } else {
        lv_obj_set_style_bg_color(mouth_inner_, lv_color_hex(0xFFF5EA), 0);
        SetVisibleObj(tongue_, false);
    }

    lv_obj_set_style_bg_color(mouth_outer_, palette_.mouth, 0);

    if (speaking_) {
        const int tongue_w = std::max(10, inner_w / 3);
        const int tongue_h = std::max(4, inner_h / 2);
        lv_obj_set_pos(tongue_, mouth_x + outer_w / 2 - tongue_w / 2, mouth_y + outer_h / 2 + 1);
        lv_obj_set_size(tongue_, tongue_w, tongue_h);
        lv_obj_set_style_radius(tongue_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(tongue_, palette_.blush, 0);
    }
}

void CuteFaceView::UpdateCheeks(int face_x, int face_y, int face_size) {
    const int cx = face_x + face_size / 2;
    const int cy = face_y + face_size * 58 / 100;
    const int w = std::max(8, static_cast<int>(std::round(mood_view_.cheek_w)));
    const int h = std::max(6, static_cast<int>(std::round(mood_view_.cheek_h)));
    const int dx = face_size * 24 / 100;
    const int opa = clamp_int(static_cast<int>(std::round(mood_view_.cheek_opa)), 0, 255);

    lv_obj_set_pos(left_cheek_, cx - dx - w / 2, cy + 1);
    lv_obj_set_size(left_cheek_, w, h);
    lv_obj_set_pos(right_cheek_, cx + dx - w / 2 + kRightCheekBias, cy - 1);
    lv_obj_set_size(right_cheek_, w, h);

    lv_obj_set_style_bg_color(left_cheek_, palette_.cheek, 0);
    lv_obj_set_style_bg_color(right_cheek_, palette_.cheek, 0);
    lv_obj_set_style_bg_opa(left_cheek_, opa, 0);
    lv_obj_set_style_bg_opa(right_cheek_, clamp_int(opa + 8, 0, 255), 0);
}

void CuteFaceView::UpdateTear(int face_x, int face_y, int face_size, float blink) {
    const bool crying = mood_ == Mood::Sad;
    if (!crying) {
        SetVisibleObj(tear_, false);
        return;
    }

    const int tear_w = 6;
    const int tear_h = 12;
    const int tear_x = face_x + face_size * 78 / 100;
    const int tear_y = face_y + face_size * 38 / 100 + static_cast<int>(std::round((1.0f - blink) * 2.0f));
    lv_obj_set_pos(tear_, tear_x, tear_y);
    lv_obj_set_size(tear_, tear_w, tear_h);
    lv_obj_set_style_radius(tear_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(tear_, palette_.tear, 0);
    lv_obj_set_style_bg_opa(tear_, LV_OPA_COVER, 0);
    SetVisibleObj(tear_, true);
}

CuteFaceView::Mood CuteFaceView::ParseEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return Mood::Neutral;
    }
    if (std::strcmp(emotion, "happy") == 0 || std::strcmp(emotion, "laughing") == 0 || std::strcmp(emotion, "loving") == 0) {
        return Mood::Happy;
    }
    if (std::strcmp(emotion, "sad") == 0 || std::strcmp(emotion, "crying") == 0) {
        return Mood::Sad;
    }
    if (std::strcmp(emotion, "angry") == 0 || std::strcmp(emotion, "scorn") == 0) {
        return Mood::Angry;
    }
    if (std::strcmp(emotion, "sleepy") == 0 || std::strcmp(emotion, "relaxed") == 0) {
        return Mood::Sleepy;
    }
    if (std::strcmp(emotion, "surprised") == 0 || std::strcmp(emotion, "shocked") == 0 || std::strcmp(emotion, "panic") == 0) {
        return Mood::Surprised;
    }
    return Mood::Neutral;
}

void CuteFaceView::SetVisibleObj(lv_obj_t* obj, bool visible) {
    if (obj == nullptr) {
        return;
    }
    if (visible) {
        lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
}