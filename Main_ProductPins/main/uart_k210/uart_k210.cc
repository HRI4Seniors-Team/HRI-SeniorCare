/**
 * @file  uart_k210.cc
 * @brief UART bridge to the MaixCam visual node with JSON emotion parsing
 * ========================================================================
 * Project : SIEVOX — ESP32-S3 side
 */

#include "uart_k210.h"

#include <esp_log.h>
#include <cJSON.h>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <inttypes.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "UART_VISION"

// CRC-8 lookup table (polynomial 0x07, matches the MaixCam node)
static uint8_t crc8_table[256];
static bool crc8_table_init = false;

static void init_crc8_table() {
    if (crc8_table_init) return;
    for (int i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
        crc8_table[i] = crc;
    }
    crc8_table_init = true;
}


// =====================================================================
//  Init
// =====================================================================

void UartK210::Init() {
    init_crc8_table();

    uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_, TX_PIN, RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialised: TX=%d, RX=%d, Baud=%d", TX_PIN, RX_PIN, BAUD_RATE);
}


// =====================================================================
//  Send
// =====================================================================

void UartK210::SendData(const char* data, size_t len) {
    uart_write_bytes(UART_NUM_, data, len);
    ESP_LOGD(TAG, "-> MaixCam: %.*s", (int)len, data);
}

void UartK210::SendCommand(const std::string& cmd) {
    std::string line = cmd;
    if (line.empty() || line.back() != '\n') {
        line += '\n';
    }
    SendData(line.c_str(), line.size());
}


// =====================================================================
//  Receive (raw)
// =====================================================================

int UartK210::ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_NUM_, buffer, max_len, pdMS_TO_TICKS(timeout_ms));
}


// =====================================================================
//  CRC-8 computation
// =====================================================================

uint8_t UartK210::ComputeCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}


// =====================================================================
//  JSON packet parser
// =====================================================================

bool UartK210::ParseEmotionPacket(const char* json_line, VisionEmotionPacket& out) {
    /**
     * Expected JSON format from MaixCam:
     * {
     *   "seq": 42, "ts": 123456,
     *   "face": true,
     *   "bbox": [80, 60, 120, 120],
     *   "emo": [0.10, 0.70, 0.15, 0.05],
     *   "pitch": 45.0, "roll": 12.3,
     *   "trk": true,
     *   "crc": "A3"
     * }
     */

    cJSON* root = cJSON_Parse(json_line);
    if (!root) {
        // Parse failures are aggregated by the receive task.  Per-line
        // warnings can themselves starve the UART task during an overrun.
        return false;
    }

    bool ok = true;

    // ── Sequence number ────────────────────────────────────────────
    cJSON* j_seq = cJSON_GetObjectItem(root, "seq");
    out.seq = cJSON_IsNumber(j_seq) ? (uint32_t)j_seq->valueint : 0;

    // ── Timestamp ──────────────────────────────────────────────────
    cJSON* j_ts = cJSON_GetObjectItem(root, "ts");
    out.ts = cJSON_IsNumber(j_ts) ? (uint32_t)j_ts->valueint : 0;

    // ── Face detected ──────────────────────────────────────────────
    cJSON* j_face = cJSON_GetObjectItem(root, "face");
    out.face_detected = cJSON_IsTrue(j_face);

    // ── Bounding box ───────────────────────────────────────────────
    memset(out.bbox, 0, sizeof(out.bbox));
    cJSON* j_bbox = cJSON_GetObjectItem(root, "bbox");
    if (cJSON_IsArray(j_bbox) && cJSON_GetArraySize(j_bbox) == 4) {
        for (int i = 0; i < 4; i++) {
            cJSON* item = cJSON_GetArrayItem(j_bbox, i);
            out.bbox[i] = cJSON_IsNumber(item) ? item->valueint : 0;
        }
    }

    // ── Emotion probability array ──────────────────────────────────
    out.emo_probs.fill(0.0f);
    cJSON* j_emo = cJSON_GetObjectItem(root, "emo");
    if (cJSON_IsArray(j_emo) && cJSON_GetArraySize(j_emo) == (int)kVisionEmotions) {
        for (size_t i = 0; i < kVisionEmotions; i++) {
            cJSON* item = cJSON_GetArrayItem(j_emo, (int)i);
            out.emo_probs[i] = cJSON_IsNumber(item) ? (float)item->valuedouble : 0.0f;
        }
    } else {
        ok = false;
    }

    // ── Gimbal state ───────────────────────────────────────────────
    cJSON* j_quality = cJSON_GetObjectItem(root, "quality");
    out.vision_quality = cJSON_IsNumber(j_quality)
        ? std::clamp((float)j_quality->valuedouble, 0.0f, 1.0f)
        : (out.face_detected ? 1.0f : 0.05f);

    cJSON* j_other_mass = cJSON_GetObjectItem(root, "other_mass");
    out.other_mass = cJSON_IsNumber(j_other_mass)
        ? std::clamp((float)j_other_mass->valuedouble, 0.0f, 1.0f)
        : 0.0f;

    cJSON* j_trial = cJSON_GetObjectItem(root, "trial");
    if (cJSON_IsString(j_trial) && j_trial->valuestring) {
        out.trial_id.assign(j_trial->valuestring, 0, 31);
    }

    cJSON* j_lat = cJSON_GetObjectItem(root, "lat_ms");
    if (cJSON_IsObject(j_lat)) {
        auto read_latency = [j_lat](const char* name) -> float {
            cJSON* value = cJSON_GetObjectItem(j_lat, name);
            return cJSON_IsNumber(value)
                ? std::max(0.0f, (float)value->valuedouble) : 0.0f;
        };
        out.capture_ms = read_latency("capture");
        out.detect_ms = read_latency("detect");
        out.align_ms = read_latency("align");
        out.fer_ms = read_latency("fer");
        out.total_ms = read_latency("total");
    }

    cJSON* j_period = cJSON_GetObjectItem(root, "period_ms");
    out.period_ms = cJSON_IsNumber(j_period) ? std::max(0.0f, (float)j_period->valuedouble) : 0.0f;
    cJSON* j_fps = cJSON_GetObjectItem(root, "fps");
    out.fps = cJSON_IsNumber(j_fps) ? std::max(0.0f, (float)j_fps->valuedouble) : 0.0f;

    cJSON* j_pitch = cJSON_GetObjectItem(root, "pitch");
    out.pitch = cJSON_IsNumber(j_pitch) ? (float)j_pitch->valuedouble : 0.0f;

    cJSON* j_roll = cJSON_GetObjectItem(root, "roll");
    out.roll = cJSON_IsNumber(j_roll) ? (float)j_roll->valuedouble : 0.0f;

    cJSON* j_trk = cJSON_GetObjectItem(root, "trk");
    out.tracking = cJSON_IsTrue(j_trk);

    // ── CRC-8 validation ───────────────────────────────────────────
    out.crc_valid = false;
    cJSON* j_crc = cJSON_GetObjectItem(root, "crc");
    if (cJSON_IsString(j_crc) && j_crc->valuestring) {
        uint8_t expected_crc = (uint8_t)strtoul(j_crc->valuestring, nullptr, 16);

        // New packets compute CRC over the exact bytes before ,"crc":".
        // This avoids cross-language float reformatting during JSON parsing.
        const char* marker = strstr(json_line, ",\"crc\":\"");
        uint8_t raw_crc = 0;
        if (marker) {
            raw_crc = ComputeCRC8((const uint8_t*)json_line,
                                  (size_t)(marker - json_line));
            out.crc_valid = (raw_crc == expected_crc);
        }

        // Legacy fallback for packets produced by the previous sender.
        if (!out.crc_valid) {
            cJSON_DeleteItemFromObject(root, "crc");
            char* payload_str = cJSON_PrintUnformatted(root);
            if (payload_str) {
                uint8_t legacy_crc = ComputeCRC8(
                    (const uint8_t*)payload_str, strlen(payload_str));
                out.crc_valid = (legacy_crc == expected_crc);
                cJSON_free(payload_str);
            }
        }
    } else {
        // No CRC field — accept anyway but flag
        ESP_LOGD(TAG, "Packet has no CRC field (seq=%" PRIu32 ")", out.seq);
        out.crc_valid = true;   // lenient: accept legacy packets
    }

    cJSON_Delete(root);
    return ok;
}


// =====================================================================
//  Background receive task
// =====================================================================

void UartK210::StartReceiveTask() {
    xTaskCreate([](void* param) {
        UartK210* uart = static_cast<UartK210*>(param);
        static uint8_t buffer[BUF_SIZE];
        size_t index = 0;
        bool dropping_frame = false;
        uint32_t line_count = 0;
        uint32_t parse_failures = 0;
        uint32_t crc_failures = 0;
        uint32_t overflow_frames = 0;
        uint32_t resync_events = 0;
        uint32_t raw_bytes = 0;
        uint32_t rx_timeouts = 0;
        uint8_t last_byte = 0;
        const int64_t summary_interval_us = 10 * 1000 * 1000;
        int64_t last_summary_us = esp_timer_get_time();

        ESP_LOGI(TAG, "Receive task started");

        while (true) {
            uint8_t byte;
            int len = uart->ReceiveData(&byte, 1, 0);

            if (len > 0) {
                raw_bytes += (uint32_t)len;
                uart->stats_.raw_bytes = raw_bytes;
                last_byte = byte;
                if (byte == '\n') {
                    if (dropping_frame) {
                        overflow_frames++;
                        dropping_frame = false;
                        index = 0;
                        continue;
                    }
                    // ── Complete line received ─────────────────────
                    buffer[index] = '\0';

                    if (index == 0) {
                        // Empty line, ignore
                        continue;
                    }
                    line_count++;
                    uart->stats_.lines = line_count;

                    // Check if it's a JSON packet (starts with '{')
                    if (buffer[0] == '{') {
                        VisionEmotionPacket pkt{};
                        bool parsed = uart->ParseEmotionPacket(
                            (const char*)buffer, pkt);

                        if (parsed) {
                            // ── Dropped packet detection ───────────
                            if (uart->last_seq_ >= 0) {
                                int32_t expected = uart->last_seq_ + 1;
                                int32_t delta = (int32_t)pkt.seq - expected;
                                if (delta > 0) {
                                    uart->drop_count_ += delta;
                                    ESP_LOGW(TAG, "Dropped %" PRId32 " packet(s) "
                                             "(seq %" PRId32 "->%" PRIu32 ", total drops=%d)",
                                             delta, uart->last_seq_,
                                             pkt.seq, uart->drop_count_);
                                }
                            }
                            uart->last_seq_ = (int32_t)pkt.seq;
                            uart->stats_.last_seq = pkt.seq;
                            uart->stats_.last_maix_ts_ms = pkt.ts;
                            uart->stats_.last_rx_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
                            uart->stats_.last_total_ms = pkt.total_ms;
                            uart->stats_.last_period_ms = pkt.period_ms;
                            uart->stats_.last_fps = pkt.fps;

                            ESP_LOGD(TAG, "Pkt seq=%" PRIu32 " face=%d emo=[%.2f,%.2f,%.2f,%.2f]",
                                     pkt.seq, pkt.face_detected,
                                     pkt.emo_probs[0], pkt.emo_probs[1],
                                     pkt.emo_probs[2], pkt.emo_probs[3]);

                            // Do not let a corrupted packet affect fusion, but
                            // retain its HWCSV row for link-quality analysis.
                            if (pkt.crc_valid && uart->vision_callback_) {
                                uart->stats_.valid_packets++;
                                uart->vision_callback_(pkt);
                            } else if (!pkt.crc_valid) {
                                crc_failures++;
                                uart->stats_.crc_failures = crc_failures;
                            }
                        } else {
                            parse_failures++;
                            uart->stats_.parse_failures = parse_failures;
                        }
                    } else {
                        // Plain-text line (e.g., ACK from MaixCam)
                        ESP_LOGI(TAG, "<- MaixCam (text): %s", buffer);
                    }

                    index = 0;

                } else if (dropping_frame) {
                    continue;
                } else if (index == 0 && byte != '{') {
                    // Wait for the start of a JSON frame. Do not resync on
                    // nested '{' characters inside a valid JSON object, e.g.
                    // the "lat_ms":{...} field from MaixCam.
                    resync_events++;
                    continue;
                } else if (index < BUF_SIZE - 1) {
                    buffer[index++] = byte;
                } else {
                    // Drop the rest of this overlong frame until its newline.
                    // This prevents parsing a suffix as an independent frame.
                    dropping_frame = true;
                    index = 0;
                }
            } else {
                rx_timeouts++;
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_summary_us >= summary_interval_us) {
                ESP_LOGI(TAG,
                         "UART summary: raw=%" PRIu32
                         " timeouts=%" PRIu32
                         " partial=%u"
                         " last=0x%02X"
                         " lines=%" PRIu32
                         " parse_fail=%" PRIu32
                         " crc_fail=%" PRIu32
                         " overflow=%" PRIu32
                         " resync=%" PRIu32
                         " drops=%d",
                         raw_bytes, rx_timeouts, (unsigned)index,
                         (unsigned)last_byte, line_count,
                         parse_failures, crc_failures,
                         overflow_frames, resync_events, uart->drop_count_);
                last_summary_us = now_us;
            }
            uart->stats_.dropped_packets = (uint32_t)std::max(0, uart->drop_count_);
        }
    }, "uart_k210_rx", 8192, this, 5, NULL);
}
