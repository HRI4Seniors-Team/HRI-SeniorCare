#pragma once

#include <lvgl.h>
#include <esp_timer.h>

#include <string>

class CuteFaceView {
public:
    explicit CuteFaceView(lv_obj_t* parent);
    ~CuteFaceView();

    void SetEmotion(const char* emotion);
    void UpdateEmotion(const char* emotion, uint32_t now_ms);
    void SetListening(bool listening);
    void SetSpeaking(bool speaking);
    void SetVisible(bool visible);

private:
    enum class Mood {
        Neutral,
        Happy,
        Sad,
        Angry,
        Sleepy,
        Surprised,
    };

    struct Palette {
        lv_color_t face = lv_color_hex(0xFFE4CF);
        lv_color_t outline = lv_color_hex(0x6B4036);
        lv_color_t hair = lv_color_hex(0x3B2322);
        lv_color_t hair_mid = lv_color_hex(0x5A3430);
        lv_color_t hair_light = lv_color_hex(0x8A5B45);
        lv_color_t eye = lv_color_hex(0x5B321D);
        lv_color_t eye_glow = lv_color_hex(0xD78A30);
        lv_color_t eye_white = lv_color_hex(0xFFF8F0);
        lv_color_t cheek = lv_color_hex(0xF3A2A8);
        lv_color_t mouth = lv_color_hex(0x71313A);
        lv_color_t mouth_inner = lv_color_hex(0xE76D74);
        lv_color_t tear = lv_color_hex(0x8FD2FF);
        lv_color_t blush = lv_color_hex(0xFFCCD2);
        lv_color_t coat = lv_color_hex(0xFFF6E7);
        lv_color_t coat_shadow = lv_color_hex(0xE8D7C4);
        lv_color_t accent = lv_color_hex(0xFF914D);
    };

    void Build();
    void Refresh(uint32_t now_ms);
    void UpdateBlink(uint32_t now_ms);
    void UpdatePalette();
    void UpdateCharacterFrame(int face_size, int face_x, int face_y, uint32_t now_ms);
    void UpdateFaceGeometry(int face_size, int face_x, int face_y);
    void UpdateEyes(int face_x, int face_y, int face_size, float blink);
    void UpdateBrows(int face_x, int face_y, int face_size, float blink);
    void UpdateMouth(int face_x, int face_y, int face_size, float blink, float speech);
    void UpdateCheeks(int face_x, int face_y, int face_size);
    void UpdateTear(int face_x, int face_y, int face_size, float blink);
    static Mood ParseEmotion(const char* emotion);
    static void SetVisibleObj(lv_obj_t* obj, bool visible);

    lv_obj_t* parent_ = nullptr;
    lv_obj_t* root_ = nullptr;
    lv_obj_t* hair_back_ = nullptr;
    lv_obj_t* hair_left_ = nullptr;
    lv_obj_t* hair_right_ = nullptr;
    lv_obj_t* hair_top_ = nullptr;
    lv_obj_t* bang_left_ = nullptr;
    lv_obj_t* bang_mid_ = nullptr;
    lv_obj_t* bang_right_ = nullptr;
    lv_obj_t* hair_highlight_ = nullptr;
    lv_obj_t* left_ear_ = nullptr;
    lv_obj_t* right_ear_ = nullptr;
    lv_obj_t* neck_ = nullptr;
    lv_obj_t* coat_ = nullptr;
    lv_obj_t* collar_left_ = nullptr;
    lv_obj_t* collar_right_ = nullptr;
    lv_obj_t* accessory_outer_ = nullptr;
    lv_obj_t* accessory_inner_ = nullptr;
    lv_obj_t* antenna_ = nullptr;
    lv_obj_t* antenna_tip_ = nullptr;
    lv_obj_t* face_ = nullptr;
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* left_pupil_ = nullptr;
    lv_obj_t* right_pupil_ = nullptr;
    lv_obj_t* left_brow_ = nullptr;
    lv_obj_t* right_brow_ = nullptr;
    lv_obj_t* mouth_outer_ = nullptr;
    lv_obj_t* mouth_inner_ = nullptr;
    lv_obj_t* tongue_ = nullptr;
    lv_obj_t* left_cheek_ = nullptr;
    lv_obj_t* right_cheek_ = nullptr;
    lv_obj_t* tear_ = nullptr;
    lv_timer_t* timer_ = nullptr;

    Palette palette_;
    Mood mood_ = Mood::Neutral;
    bool listening_ = false;
    bool speaking_ = false;
    bool visible_ = true;
    uint32_t last_blink_ms_ = 0;
    uint32_t blink_seed_ms_ = 0;
    uint32_t mood_changed_ms_ = 0;
};
