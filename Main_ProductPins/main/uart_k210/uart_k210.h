/**
 * @file  uart_k210.h
 * @brief UART bridge to the MaixCam visual node with JSON emotion parsing
 * ========================================================================
 * Project : SIEVOX — ESP32-S3 side
 * Changes from original:
 *   1. JSON emotion packet parsing (ParseEmotionPacket)
 *   2. CRC-8 validation of incoming packets
 *   3. VisionPacketCallback for delivering parsed results to fusion engine
 *   4. Sequence-number tracking for dropped-packet detection
 *   5. RX-only product wiring; MaixCam streams packets proactively
 *   6. Non-blocking receive with line buffering
 */

#ifndef UART_K210_H
#define UART_K210_H

#include <driver/uart.h>
#include <functional>
#include <array>
#include <string>
#include <cstdint>

// Must match MaixCam side: [happy, sad, neutral, anger]
constexpr size_t kVisionEmotions = 4;

/**
 * @brief Parsed emotion telemetry packet from MaixCam.
 */
struct VisionEmotionPacket {
    uint32_t seq = 0;                                ///< Sequence number
    uint32_t ts = 0;                                 ///< Vision-node uptime (ms)
    bool     face_detected = false;                  ///< Face in frame?
    int      bbox[4] = {0, 0, 0, 0};                 ///< [x, y, w, h] or zeros
    std::array<float, kVisionEmotions> emo_probs;    ///< [H, S, N, A]
    float    vision_quality = 0.0f;                  ///< Four-class retained mass
    float    other_mass = 0.0f;                     ///< Excluded seven-class mass
    float    capture_ms = 0.0f;
    float    detect_ms = 0.0f;
    float    align_ms = 0.0f;
    float    fer_ms = 0.0f;
    float    total_ms = 0.0f;
    float    period_ms = 0.0f;
    float    fps = 0.0f;
    std::string trial_id = "unassigned";
    float    pitch = 0.0f;                           ///< Gimbal pitch
    float    roll = 0.0f;                            ///< Gimbal roll
    bool     tracking = false;                       ///< Tracking enabled?
    bool     crc_valid = false;                      ///< CRC check passed?
};

struct UartVisionStats {
    uint32_t raw_bytes = 0;
    uint32_t lines = 0;
    uint32_t valid_packets = 0;
    uint32_t parse_failures = 0;
    uint32_t crc_failures = 0;
    uint32_t dropped_packets = 0;
    uint32_t last_seq = 0;
    uint32_t last_maix_ts_ms = 0;
    uint32_t last_rx_uptime_ms = 0;
    float last_total_ms = 0.0f;
    float last_period_ms = 0.0f;
    float last_fps = 0.0f;
};

/**
 * @brief Callback type invoked when a valid emotion packet is received.
 */
using VisionPacketCallback = std::function<void(const VisionEmotionPacket&)>;


class UartK210 {
public:
    void Init();
    void SendData(const char* data, size_t len);
    int  ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms);

    /**
     * @brief Start the background receive task.
     *
     * Spawns a FreeRTOS task that continuously reads UART lines,
     * parses JSON emotion packets, validates CRC, and invokes the
     * registered callback for each valid packet.
     *
     * Plain-text lines (command ACKs from MaixCam) are logged but not
     * forwarded to the callback.
     */
    void StartReceiveTask();

    /**
     * @brief Register a callback for incoming emotion packets.
     * @param cb  Function to call with each parsed VisionEmotionPacket.
     */
    void SetVisionCallback(VisionPacketCallback cb) {
        vision_callback_ = std::move(cb);
    }

    /**
     * @brief Send a command to MaixCam (e.g., "GET_STATE\n").
     * Convenience wrapper that appends \n if missing.
     */
    void SendCommand(const std::string& cmd);
    UartVisionStats GetStats() const { return stats_; }

private:
    static constexpr uart_port_t UART_NUM_ = UART_NUM_1;
    static constexpr int TX_PIN    = UART_PIN_NO_CHANGE;
    static constexpr int RX_PIN    = 38;
    static constexpr int BAUD_RATE = 115200;
    // Room for the largest telemetry frame; overlong frames are still
    // explicitly discarded by the receive task.
    static constexpr int BUF_SIZE  = 4096;

    VisionPacketCallback vision_callback_;

    // Sequence tracking for dropped-packet detection
    int32_t last_seq_    = -1;
    int     drop_count_  = 0;
    UartVisionStats stats_{};

    /**
     * @brief Parse a JSON line into a VisionEmotionPacket.
     * @param json_line  Null-terminated JSON string (one line).
     * @param out        Output struct.
     * @return true if parsing succeeded and CRC is valid.
     */
    bool ParseEmotionPacket(const char* json_line, VisionEmotionPacket& out);

    /**
     * @brief Compute CRC-8 (poly 0x07) over a byte string.
     */
    static uint8_t ComputeCRC8(const uint8_t* data, size_t len);
};

#endif // UART_K210_H
