"""Resona visual node for MaixCAM-Pro (MaixPy 4).

Required device models:
  /root/models/yolov8n_face.mud
  /root/models/face_emotion.mud

The script follows Sipeed's official face-emotion example and emits one
newline-delimited JSON packet per inference.  Probabilities are ordered as
[happy, sad, neutral, anger] to match the ESP32-S3 fusion implementation.
"""

import json
import time

from maix import app, camera, display, err, image, nn, pinmap, uart


FACE_MODEL = "/root/models/yolov8n_face.mud"
EMOTION_MODEL = "/root/models/face_emotion.mud"

UART_DEVICE = "/dev/ttyS0"
UART_TX_PIN = "A16"
UART_RX_PIN = "A17"
UART_TX_FUNCTION = "UART0_TX"
UART_RX_FUNCTION = "UART0_RX"
UART_BAUD = 115200

SHOW_PREVIEW = True
DETECT_CONFIDENCE = 0.50
DETECT_IOU = 0.45
CROP_SCALE = 0.90
PACKET_INTERVAL_MS = 100
PROTOCOL_VERSION = 1


def monotonic_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    if hasattr(time, "monotonic_ns"):
        return time.monotonic_ns() // 1000000
    return int(time.time() * 1000)


def elapsed_ms(start_ms):
    now = monotonic_ms()
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, start_ms)
    return now - start_ms


def crc8(data):
    """CRC-8, polynomial 0x07, initial value 0x00."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def encode_packet(packet):
    payload = json.dumps(packet, separators=(",", ":"))
    checksum = crc8(payload[:-1].encode("ascii"))
    return payload[:-1] + ',"crc":"{:02X}"}}\n'.format(checksum)


def dense_scores(result, labels):
    scores = {}
    for class_id, score in result:
        if 0 <= class_id < len(labels):
            scores[str(labels[class_id]).lower()] = float(score)
    return scores


def map_to_four_classes(scores):
    """Keep H/S/N/A and report excluded seven-class mass as uncertainty."""
    selected = [
        max(0.0, scores.get("happy", 0.0)),
        max(0.0, scores.get("sad", 0.0)),
        max(0.0, scores.get("neutral", 0.0)),
        max(0.0, scores.get("angry", scores.get("anger", 0.0))),
    ]
    selected_mass = sum(selected)
    total_mass = sum(max(0.0, value) for value in scores.values())
    other_mass = max(0.0, total_mass - selected_mass)
    if selected_mass <= 1e-9:
        return [0.25, 0.25, 0.25, 0.25], 0.0, min(1.0, other_mass)
    probs = [value / selected_mass for value in selected]
    return probs, min(1.0, selected_mass), min(1.0, other_mass)


class CommandBuffer:
    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        if not data:
            return []
        self.buffer += bytes(data)
        lines = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if text:
                lines.append(text)
        if len(self.buffer) > 512:
            self.buffer = b""
        return lines


def initialize_uart():
    err.check_raise(pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION), "UART TX pin mapping failed")
    err.check_raise(pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNCTION), "UART RX pin mapping failed")
    return uart.UART(UART_DEVICE, UART_BAUD)


def main():
    serial = initialize_uart()
    detector = nn.YOLOv8(model=FACE_MODEL, dual_buff=False)
    landmarks = nn.FaceLandmarks(model="")
    classifier = nn.Classifier(model=EMOTION_MODEL, dual_buff=False)
    cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
    screen = display.Display() if SHOW_PREVIEW else None

    commands = CommandBuffer()
    sequence = 0
    last_send_ms = 0
    current_trial = "unassigned"

    while not app.need_exit():
        for command in commands.feed(serial.read()):
            parts = command.split()
            if parts[0] == "PING":
                serial.write_str("PONG {} {}\n".format(parts[1] if len(parts) > 1 else "0", monotonic_ms()))
            elif parts[0] == "GET_STATE":
                serial.write_str("STATE ready seq={} trial={}\n".format(sequence, current_trial))
            elif parts[0] == "SET_TRIAL" and len(parts) > 1:
                current_trial = parts[1][:31]
                serial.write_str("ACK_TRIAL {}\n".format(current_trial))

        total_start = monotonic_ms()
        capture_start = monotonic_ms()
        frame = cam.read()
        capture_ms = elapsed_ms(capture_start)

        detect_start = monotonic_ms()
        objects = detector.detect(frame, conf_th=DETECT_CONFIDENCE, iou_th=DETECT_IOU, sort=1)
        detect_ms = elapsed_ms(detect_start)

        face_detected = False
        bbox = [0, 0, 0, 0]
        probs = [0.25, 0.25, 0.25, 0.25]
        selected_mass = 0.0
        other_mass = 1.0
        align_ms = 0
        classify_ms = 0

        valid_objects = [obj for obj in objects if obj.score >= DETECT_CONFIDENCE]
        if valid_objects:
            obj = max(valid_objects, key=lambda item: item.w * item.h)
            align_start = monotonic_ms()
            aligned = landmarks.crop_image(
                frame,
                obj.x,
                obj.y,
                obj.w,
                obj.h,
                obj.points,
                classifier.input_width(),
                classifier.input_height(),
                CROP_SCALE,
            )
            align_ms = elapsed_ms(align_start)
            if aligned:
                gray = aligned.to_format(image.Format.FMT_GRAYSCALE)
                classify_start = monotonic_ms()
                result = classifier.classify(gray, softmax=True)
                classify_ms = elapsed_ms(classify_start)
                probs, selected_mass, other_mass = map_to_four_classes(dense_scores(result, classifier.labels))
                face_detected = True
                bbox = [int(obj.x), int(obj.y), int(obj.w), int(obj.h)]
                frame.draw_rect(obj.x, obj.y, obj.w, obj.h, image.COLOR_GREEN, 2)
                winner = max(range(4), key=lambda idx: probs[idx])
                frame.draw_string(obj.x, max(0, obj.y - 18), ["happy", "sad", "neutral", "anger"][winner], image.COLOR_GREEN)

        total_ms = elapsed_ms(total_start)
        now = monotonic_ms()
        if elapsed_ms(last_send_ms) >= PACKET_INTERVAL_MS:
            packet = {
                "v": PROTOCOL_VERSION,
                "type": "vision_emotion",
                "seq": sequence,
                "ts": now,
                "trial": current_trial,
                "face": face_detected,
                "bbox": bbox,
                "emo": [round(value, 6) for value in probs],
                "quality": round(selected_mass, 6) if face_detected else 0.05,
                "other_mass": round(other_mass, 6),
                "lat_ms": {
                    "capture": capture_ms,
                    "detect": detect_ms,
                    "align": align_ms,
                    "fer": classify_ms,
                    "total": total_ms,
                },
                "pitch": 0.0,
                "roll": 0.0,
                "trk": False,
            }
            serial.write_str(encode_packet(packet))
            sequence = (sequence + 1) & 0xFFFFFFFF
            last_send_ms = now

        if screen:
            screen.show(frame)


if __name__ == "__main__":
    main()
