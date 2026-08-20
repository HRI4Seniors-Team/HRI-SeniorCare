#include "servo_tracker.h"

#include <algorithm>
#include <atomic>
#include <cmath>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr const char* TAG = "ServoTracker";

// Current build target is BREAD_COMPACT_WIFI. GPIO40 is configured as
// VOLUME_UP_BUTTON_GPIO there, so the first SG90 signal pin is GPIO41.
// The second servo uses GPIO18, which is currently unused by this board.
constexpr gpio_num_t kServoPanPin = GPIO_NUM_41;
constexpr gpio_num_t kServoTiltPin = GPIO_NUM_18;

constexpr ledc_mode_t kServoSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kServoTimer = LEDC_TIMER_2;
constexpr ledc_channel_t kServoPanChannel = LEDC_CHANNEL_6;
constexpr ledc_channel_t kServoTiltChannel = LEDC_CHANNEL_7;
constexpr ledc_timer_bit_t kServoDutyResolution = LEDC_TIMER_13_BIT;
constexpr uint32_t kServoFreqHz = 50;
constexpr uint32_t kServoPeriodUs = 20000;
constexpr uint32_t kServoMaxDuty = (1U << 13) - 1;

constexpr int kServoMinAngle = 20;
constexpr int kServoMaxAngle = 160;
constexpr int kPulseMinUs = 500;
constexpr int kPulseMaxUs = 2500;

constexpr int kFrameWidth = 320;
constexpr int kFrameHeight = 240;
constexpr int kFrameCenterX = kFrameWidth / 2;
constexpr int kFrameCenterY = kFrameHeight / 2;
constexpr int kDeadbandPx = 8;
constexpr float kGainDegPerPx = 0.090f;
constexpr float kMaxStepDeg = 9.0f;
constexpr int kMinMoveDeg = 1;
constexpr int64_t kMinUpdateIntervalUs = 80 * 1000;
constexpr int64_t kNoFaceReturnDelayUs = 2000 * 1000;

std::atomic<bool> g_initialized{false};
std::atomic<bool> g_self_test_active{true};
int g_current_angle = 90;
int g_current_tilt_angle = 90;
int64_t g_last_update_us = 0;
int64_t g_last_face_us = 0;

uint32_t AngleToDuty(int angle)
{
    angle = std::clamp(angle, 0, 180);
    const int pulse_us = kPulseMinUs + (kPulseMaxUs - kPulseMinUs) * angle / 180;
    return static_cast<uint32_t>((static_cast<uint64_t>(pulse_us) * kServoMaxDuty) / kServoPeriodUs);
}

void WriteServoAngle(int angle)
{
    angle = std::clamp(angle, kServoMinAngle, kServoMaxAngle);
    g_current_angle = angle;
    const uint32_t duty = AngleToDuty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(kServoSpeedMode, kServoPanChannel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(kServoSpeedMode, kServoPanChannel));
    ESP_LOGI(TAG, "SERVO_PAN angle=%d duty=%lu pin=%d", angle, static_cast<unsigned long>(duty), kServoPanPin);
}

void WriteTiltServoAngle(int angle)
{
    angle = std::clamp(angle, kServoMinAngle, kServoMaxAngle);
    g_current_tilt_angle = angle;
    const uint32_t duty = AngleToDuty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(kServoSpeedMode, kServoTiltChannel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(kServoSpeedMode, kServoTiltChannel));
    ESP_LOGI(TAG, "SERVO_TILT angle=%d duty=%lu pin=%d", angle, static_cast<unsigned long>(duty), kServoTiltPin);
}

void ServoSelfTestTask(void*)
{
    constexpr int pan_sequence[] = {90, 80, 90, 100, 90};
    constexpr int tilt_sequence[] = {90, 100, 90, 80, 90};

    ESP_LOGI(TAG, "SERVO_TEST start: pan GPIO%d tilt GPIO%d, 50Hz", kServoPanPin, kServoTiltPin);
    for (int angle : pan_sequence) {
        WriteServoAngle(angle);
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
    for (int angle : tilt_sequence) {
        WriteTiltServoAngle(angle);
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
    ESP_LOGI(TAG, "SERVO_TEST done: holding center at 90 degrees");
    g_self_test_active.store(false);
    vTaskDelete(nullptr);
}

}  // namespace

void InitializeServoSelfTest()
{
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kServoSpeedMode;
    timer_config.duty_resolution = kServoDutyResolution;
    timer_config.timer_num = kServoTimer;
    timer_config.freq_hz = kServoFreqHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t pan_config = {};
    pan_config.gpio_num = kServoPanPin;
    pan_config.speed_mode = kServoSpeedMode;
    pan_config.channel = kServoPanChannel;
    pan_config.intr_type = LEDC_INTR_DISABLE;
    pan_config.timer_sel = kServoTimer;
    pan_config.duty = AngleToDuty(90);
    pan_config.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&pan_config));

    ledc_channel_config_t tilt_config = pan_config;
    tilt_config.gpio_num = kServoTiltPin;
    tilt_config.channel = kServoTiltChannel;
    ESP_ERROR_CHECK(ledc_channel_config(&tilt_config));
    g_initialized.store(true);

    xTaskCreate(ServoSelfTestTask, "servo_self_test", 3072, nullptr, 2, nullptr);
}

void ServoTrackerOnVisionPacket(const VisionEmotionPacket& pkt)
{
    if (!g_initialized.load() || g_self_test_active.load()) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    if (now_us - g_last_update_us < kMinUpdateIntervalUs) {
        return;
    }

    if (!pkt.face_detected || pkt.bbox[2] <= 0 || pkt.bbox[3] <= 0) {
        if (g_last_face_us > 0 && now_us - g_last_face_us > kNoFaceReturnDelayUs) {
            bool returning = false;
            if (std::abs(g_current_angle - 90) > 2) {
                const int step = (g_current_angle > 90) ? -3 : 3;
                WriteServoAngle(g_current_angle + step);
                returning = true;
            }
            if (std::abs(g_current_tilt_angle - 90) > 2) {
                const int step = (g_current_tilt_angle > 90) ? -3 : 3;
                WriteTiltServoAngle(g_current_tilt_angle + step);
                returning = true;
            }
            if (returning) {
                ESP_LOGI(TAG, "SERVO_TRACK no_face return pan=%d tilt=%d", g_current_angle, g_current_tilt_angle);
            }
            g_last_update_us = now_us;
        }
        return;
    }

    g_last_face_us = now_us;

    const int face_center_x = pkt.bbox[0] + pkt.bbox[2] / 2;
    const int face_center_y = pkt.bbox[1] + pkt.bbox[3] / 2;
    const int error_x = face_center_x - kFrameCenterX;
    const int error_y = face_center_y - kFrameCenterY;

    bool moved = false;

    if (std::abs(error_x) > kDeadbandPx) {
        float step_x = error_x * kGainDegPerPx;
        step_x = std::clamp(step_x, -kMaxStepDeg, kMaxStepDeg);
        if (std::abs(step_x) < kMinMoveDeg) {
            step_x = (step_x < 0.0f) ? -kMinMoveDeg : kMinMoveDeg;
        }
        const int next_angle = std::clamp(
            g_current_angle + static_cast<int>(std::round(step_x)),
            kServoMinAngle,
            kServoMaxAngle);
        if (next_angle != g_current_angle) {
            WriteServoAngle(next_angle);
            moved = true;
        }
    }

    if (std::abs(error_y) > kDeadbandPx) {
        float step_y = error_y * kGainDegPerPx;
        step_y = std::clamp(step_y, -kMaxStepDeg, kMaxStepDeg);
        if (std::abs(step_y) < kMinMoveDeg) {
            step_y = (step_y < 0.0f) ? -kMinMoveDeg : kMinMoveDeg;
        }
        // Invert this sign if the physical tilt direction is reversed.
        const int next_tilt = std::clamp(
            g_current_tilt_angle - static_cast<int>(std::round(step_y)),
            kServoMinAngle,
            kServoMaxAngle);
        if (next_tilt != g_current_tilt_angle) {
            WriteTiltServoAngle(next_tilt);
            moved = true;
        }
    }

    g_last_update_us = now_us;

    ESP_LOGI(TAG, "SERVO_TRACK face cx=%d cy=%d errx=%d erry=%d pan=%d tilt=%d moved=%d bbox=[%d,%d,%d,%d]",
             face_center_x, face_center_y, error_x, error_y,
             g_current_angle, g_current_tilt_angle, moved ? 1 : 0,
             pkt.bbox[0], pkt.bbox[1], pkt.bbox[2], pkt.bbox[3]);
}
