#include "cute_face_view.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr int kRootSize = 210;
constexpr int kFaceBaseSize = 142;
constexpr int kBlinkPeriodMs = 3600;
constexpr int kSpeechPeriodMs = 260;

inline int clamp_int(int value, int lo, int hi) {
    return std::min(std::max(value, lo), hi);
}

inline float clamp01(float value) {
    return std::min(std::max(value, 0.0f), 1.0f);
}

inline float wave01(uint32_t now_ms, float period_ms, float phase_ms = 0.0f) {
    const float phase = (static_cast<float>(now_ms) + phase_ms) / period_ms;
    const float raw = static_cast<float>(std::sin(phase * 6.2831853f));
    return 0.5f + 0.5f * raw;
}
}

CuteFaceView::CuteFaceView(lv_obj_t* parent)
    : parent_(parent) {
    blink_seed_ms_ = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) % kBlinkPeriodMs;
    Build();
    Refresh(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
    timer_ = lv_timer_create([](lv_timer_t* timer) {
        auto* self = static_cast<CuteFaceView*>(lv_timer_get_user_data(timer));
        if (self != nullptr) {
            self->Refresh(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
        }
    }, 40, this);
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
    UpdateEmotion(emotion, static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
}

void CuteFaceView::UpdateEmotion(const char* emotion, uint32_t now_ms) {
    const Mood next = ParseEmotion(emotion);
    if (next != mood_) {
        mood_ = next;
        mood_changed_ms_ = now_ms;
        blink_seed_ms_ = (mood_changed_ms_ + 127U) % kBlinkPeriodMs;
    }
    Refresh(now_ms);
}

void CuteFaceView::SetListening(bool listening) {
    listening_ = listening;
    if (listening) {
        speaking_ = false;
    }
    Refresh(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
}

void CuteFaceView::SetSpeaking(bool speaking) {
    speaking_ = speaking;
    if (speaking) {
        listening_ = false;
    }
    Refresh(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
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
        left_brow_, right_brow_, mouth_outer_, mouth_inner_,
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
    if (root_ == nullptr || !visible_) {
        return;
    }

    UpdatePalette();

    const float idle_breathe = 0.5f + 0.5f * static_cast<float>(std::sin(static_cast<float>(now_ms) / 1300.0f));
    const float motion = speaking_ ? wave01(now_ms, static_cast<float>(kSpeechPeriodMs)) : 0.0f;
    const int face_size = kFaceBaseSize + (speaking_ ? 2 : 0) + (listening_ ? 1 : 0) + static_cast<int>(std::round((idle_breathe - 0.5f) * 4.0f));
    const int face_x = (kRootSize - face_size) / 2;
    const int face_y = (kRootSize - face_size) / 2 + (speaking_ ? static_cast<int>(std::round((motion - 0.5f) * 3.0f)) : 0) + (listening_ ? -1 : 0);

    UpdateFaceGeometry(face_size, face_x, face_y);
    UpdateCharacterFrame(face_size, face_x, face_y, now_ms);

    float blink = 0.0f;
    UpdateBlink(now_ms);
    const uint32_t blink_ms = (now_ms + blink_seed_ms_) % kBlinkPeriodMs;
    if (blink_ms < 90U) {
        blink = static_cast<float>(blink_ms) / 90.0f;
    } else if (blink_ms < 130U) {
        blink = 1.0f;
    } else if (blink_ms < 220U) {
        blink = 1.0f - static_cast<float>(blink_ms - 130U) / 90.0f;
    }

    UpdateEyes(face_x, face_y, face_size, blink);
    UpdateBrows(face_x, face_y, face_size, blink);
    UpdateCheeks(face_x, face_y, face_size);
    UpdateMouth(face_x, face_y, face_size, blink, motion);
    UpdateTear(face_x, face_y, face_size, blink);
}

void CuteFaceView::UpdateBlink(uint32_t now_ms) {
    if (now_ms - last_blink_ms_ > 600U) {
        last_blink_ms_ = now_ms;
    }
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

void CuteFaceView::UpdateEyes(int face_x, int face_y, int face_size, float blink) {
    const bool asleep = mood_ == Mood::Sleepy;
    const bool excited = mood_ == Mood::Surprised || listening_;
    const bool smiling = mood_ == Mood::Happy;
    const bool angry = mood_ == Mood::Angry;

    int eye_w = excited ? 35 : (smiling ? 33 : 34);
    int eye_h = excited ? 34 : (smiling ? 18 : 31);
    if (angry) {
        eye_h = 24;
    }
    if (asleep) {
        eye_h = 11;
        eye_w = 30;
    }
    const int blink_h = std::max(4, static_cast<int>(std::round(eye_h * (1.0f - 0.82f * blink))));
    const int pupil_size = asleep ? 5 : (excited ? 13 : (smiling ? 11 : 12));
    const int eye_y = face_y + face_size * 34 / 100 + (asleep ? 6 : 0) - (excited ? 1 : 0) + (angry ? 1 : 0);
    const int eye_left_x = face_x + face_size * 31 / 100 - eye_w / 2;
    const int eye_right_x = face_x + face_size * 69 / 100 - eye_w / 2;

    lv_obj_set_pos(left_eye_, eye_left_x, eye_y);
    lv_obj_set_size(left_eye_, eye_w, blink_h);
    lv_obj_set_pos(right_eye_, eye_right_x, eye_y);
    lv_obj_set_size(right_eye_, eye_w, blink_h);

    const int pupil_offset_y = std::max(0, blink_h / 2 - pupil_size / 2 - 2 + (angry ? 1 : 0));
    lv_obj_set_pos(left_pupil_, eye_left_x + eye_w / 2 - pupil_size / 2, eye_y + pupil_offset_y);
    lv_obj_set_size(left_pupil_, pupil_size, pupil_size);
    lv_obj_set_pos(right_pupil_, eye_right_x + eye_w / 2 - pupil_size / 2, eye_y + pupil_offset_y);
    lv_obj_set_size(right_pupil_, pupil_size, pupil_size);

    const bool hide_pupils = blink > 0.76f || asleep;
    SetVisibleObj(left_pupil_, !hide_pupils);
    SetVisibleObj(right_pupil_, !hide_pupils);
    lv_obj_set_style_bg_color(left_pupil_, excited ? palette_.eye_glow : palette_.eye, 0);
    lv_obj_set_style_bg_color(right_pupil_, excited ? palette_.eye_glow : palette_.eye, 0);

    if (listening_ || excited) {
        lv_obj_set_style_bg_color(left_eye_, lv_color_hex(0xFFFDF8), 0);
        lv_obj_set_style_bg_color(right_eye_, lv_color_hex(0xFFFDF8), 0);
    } else {
        lv_obj_set_style_bg_color(left_eye_, palette_.eye_white, 0);
        lv_obj_set_style_bg_color(right_eye_, palette_.eye_white, 0);
    }

    // Eye sparkle: a tiny upper highlight to avoid the "staring dead-center" look.
    lv_obj_set_style_radius(left_pupil_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_radius(right_pupil_, LV_RADIUS_CIRCLE, 0);
    if (!hide_pupils) {
        lv_obj_set_style_shadow_width(left_pupil_, 0, 0);
        lv_obj_set_style_shadow_width(right_pupil_, 0, 0);
    }
}

void CuteFaceView::UpdateBrows(int face_x, int face_y, int face_size, float blink) {
    const bool happy = mood_ == Mood::Happy;
    const bool sad = mood_ == Mood::Sad;
    const bool angry = mood_ == Mood::Angry;
    const bool listening = listening_;
    const int brow_w = happy ? 24 : 28;
    const int brow_h = 5;
    int base_y = face_y + face_size * 28 / 100;
    if (listening) {
        base_y -= 3;
    }
    if (happy) {
        base_y -= 4;
    }
    if (sad) {
        base_y += 4;
    }
    if (angry) {
        base_y += 2;
    }

    const int left_x = face_x + face_size * 24 / 100;
    const int right_x = face_x + face_size * 56 / 100;

    lv_obj_set_pos(left_brow_, left_x, base_y + (sad ? 4 : 0) - (blink > 0.6f ? 1 : 0));
    lv_obj_set_size(left_brow_, brow_w, brow_h);
    lv_obj_set_pos(right_brow_, right_x, base_y + (angry ? -3 : 0));
    lv_obj_set_size(right_brow_, brow_w, brow_h);

    lv_obj_set_style_bg_color(left_brow_, angry ? lv_color_hex(0x2B3742) : palette_.outline, 0);
    lv_obj_set_style_bg_color(right_brow_, angry ? lv_color_hex(0x2B3742) : palette_.outline, 0);
    lv_obj_set_style_radius(left_brow_, brow_h, 0);
    lv_obj_set_style_radius(right_brow_, brow_h, 0);

    SetVisibleObj(left_brow_, !(mood_ == Mood::Sleepy && !listening_));
    SetVisibleObj(right_brow_, !(mood_ == Mood::Sleepy && !listening_));
}

void CuteFaceView::UpdateMouth(int face_x, int face_y, int face_size, float blink, float speech) {
    const bool happy = mood_ == Mood::Happy;
    const bool sad = mood_ == Mood::Sad;
    const bool angry = mood_ == Mood::Angry;
    const bool sleepy = mood_ == Mood::Sleepy;
    const bool surprised = mood_ == Mood::Surprised;
    const bool active = speaking_ || listening_;

    int outer_w = active ? 50 : 42;
    int outer_h = 14;
    int inner_w = 38;
    int inner_h = 6;
    int center_y = face_y + face_size * 70 / 100;

    if (happy) {
        outer_w = 54;
        outer_h = 18;
        inner_w = 40;
        inner_h = 8;
        center_y += 1;
    } else if (sad) {
        outer_w = 32;
        outer_h = 12;
        inner_w = 22;
        inner_h = 5;
        center_y += 2;
    } else if (angry) {
        outer_w = 40;
        outer_h = 10;
        inner_w = 32;
        inner_h = 4;
    } else if (sleepy) {
        outer_w = 36;
        outer_h = 8;
        inner_w = 28;
        inner_h = 4;
    } else if (surprised || listening_) {
        outer_w = 44;
        outer_h = 16;
        inner_w = 34;
        inner_h = 10;
    }

    if (speaking_) {
        const int open_h = 8 + static_cast<int>(std::round(12.0f * speech));
        outer_h = 15 + static_cast<int>(std::round(8.0f * speech));
        inner_h = open_h;
        inner_w = outer_w - 12;
        center_y += static_cast<int>(std::round((speech - 0.5f) * 6.0f));
    }

    const int mouth_x = face_x + face_size / 2 - outer_w / 2;
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
    } else if (listening_) {
        inner_y += 1;
    }
    if (speaking_) {
        inner_y += static_cast<int>(std::round((speech - 0.5f) * 3.0f));
    }

    lv_obj_set_pos(mouth_inner_, inner_x, inner_y);
    lv_obj_set_size(mouth_inner_, inner_w, inner_h);
    lv_obj_set_style_radius(mouth_inner_, LV_RADIUS_CIRCLE, 0);

    if (speaking_) {
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
    const bool cheerful = mood_ == Mood::Happy || speaking_ || listening_;
    const bool shy = mood_ == Mood::Sad || mood_ == Mood::Sleepy;
    const int cheek_size = cheerful ? 17 : 13;
    const int cheek_w = cheek_size + 8;
    const int cheek_h = cheek_size;
    const int cheek_y = face_y + face_size * 60 / 100;
    const int left_x = face_x + face_size * 17 / 100;
    const int right_x = face_x + face_size * 66 / 100;

    lv_obj_set_pos(left_cheek_, left_x, cheek_y);
    lv_obj_set_size(left_cheek_, cheek_w, cheek_h);
    lv_obj_set_pos(right_cheek_, right_x, cheek_y);
    lv_obj_set_size(right_cheek_, cheek_w, cheek_h);
    lv_obj_set_style_bg_opa(left_cheek_, cheerful ? 150 : (shy ? 80 : 60), 0);
    lv_obj_set_style_bg_opa(right_cheek_, cheerful ? 150 : (shy ? 80 : 60), 0);
    lv_obj_set_style_bg_color(left_cheek_, palette_.cheek, 0);
    lv_obj_set_style_bg_color(right_cheek_, palette_.cheek, 0);
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
