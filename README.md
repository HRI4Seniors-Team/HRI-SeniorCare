# Resona Senior-Care HRI System

Resona is an ESP32-S3 + MaixCam Pro embodied interaction prototype for senior-care scenarios. The current branch focuses on multimodal emotion perception, voice dialogue, lightweight face tracking, and cloud-side emotion record display.

## Current implementation status

Updated on 2026-08-20.

- Main firmware: `Main/`
- Vision node for MaixCam Pro: `Main/MaixCAM/resona_visual_node.py`
- Remote dashboard: <https://sievox.cn/resona>
- Target ESP32 board profile: `bread-compact-wifi`
- Vision hardware: MaixCam Pro, not K210
- Local display: ST7789/LVGL with a Q-style cute face renderer
- Cloud chain: ESP32 posts fused emotion and warning records to `https://sievox.cn/resona/emotion/write`

The historical K210/MEMS7 description has been superseded by the MaixCam Pro + ESP32-S3 implementation. K210-related notes are retained only in legacy folders and should not be used for the current hardware setup.

## System architecture

```text
MaixCam Pro
  └─ face detection / visual emotion JSON
     └─ UART 115200
        └─ ESP32-S3 firmware
           ├─ voice dialogue and ASR/TTS client
           ├─ audio-visual emotion fusion
           ├─ LVGL cute face display
           ├─ SG90 face-tracking servo output
           └─ HTTPS upload
              └─ sievox.cn/resona dashboard
```

## Current wiring quick reference

This is the target ESP32-S3-side wiring for the new hardware revision.

| Module | Pin mapping | Notes |
| --- | --- | --- |
| OLED | SDA → GPIO41, SCL → GPIO42, VCC → 3.3 V, GND → GND | I2C OLED. |
| INMP441 | SCK → GPIO5, WS → GPIO4, SD → GPIO6, VDD → 3.3 V, GND → GND | Digital microphone input. |
| MAX98357A | DIN → GPIO7, BCLK → GPIO15, LRC/WS → GPIO16, VIN → 5 V, GND → GND | I2S speaker output. |
| SG90 pan servo | Signal → GPIO1, VCC → 5 V, GND → GND | Horizontal axis. |
| SG90 tilt servo | Signal → GPIO2, VCC → 5 V, GND → GND | Vertical axis. |
| Buttons | Volume+ → GPIO40, Volume- → GPIO39, BOOT → GPIO0, Custom → GPIO47 | BOOT is a strap pin. |

Important conflicts:

- GPIO0 is a strapping pin; keep it only for BOOT.
- GPIO39 is now occupied by Volume-; the old GPIO38/39 UART pair cannot stay as-is.
- GPIO41/42 are now consumed by the OLED.
- GPIO1/2 are now used by servo signals; verify boot stability on the target board before final wiring.
- Servo power should come from a separate 5 V rail with common ground to the ESP32.

## Repository layout

```text
HRI-SeniorCare/
├─ README.md                         # Project-level overview
├─ Main/
│  ├─ README.md                      # Firmware, wiring, build, and validation guide
│  ├─ MaixCAM/resona_visual_node.py  # MaixCam Pro vision UART script
│  ├─ main/
│  │  ├─ application.cc/.h           # Dialogue, fusion upload, warning upload
│  │  ├─ emotion/                    # Audio-visual fusion and LLM visual context helpers
│  │  ├─ servo/                      # SG90 pan/tilt face tracking
│  │  ├─ uart_k210/                  # Current UART bridge; name kept for compatibility
│  │  └─ display/lvgl_display/       # Cute face renderer
│  └─ tools/hardware_validation/     # Hardware validation utilities and notes
└─ xiaozhi-esp32-server/             # Upstream/reference server subtree
```

## Quick start

1. Read the current firmware guide: `Main/README.md`.
2. Upload `Main/MaixCAM/resona_visual_node.py` to the MaixCam Pro app/script directory.
3. Build and flash the ESP32-S3 firmware from `Main/`.
4. Open <https://sievox.cn/resona> and verify that visual emotion, dialogue status, and warning records update.

## Current validation summary

- MaixCam Pro to ESP32 UART communication has been validated with stable JSON packets.
- ESP32 voice dialogue path has been validated after reverting to the lightweight HTTP emotion-upload chain.
- Visual emotion context is injected into the dialogue prompt so the model can answer visual-state questions such as “你看见我什么情绪”.
- High audio-visual conflict no longer triggers a local audible alarm; warning records are uploaded to the cloud-side emotion warning log instead.
- SG90 face-tracking pan motion has been validated; pan/tilt support is present in firmware.

## Known engineering notes

- The firmware binary is close to the current app partition limit. The last checked build left only a few hundred bytes of free app partition space. Future feature additions should either reduce binary size or adjust the partition table.
- `Main/main/uart_k210/` still uses the historical folder name, but the active device is MaixCam Pro.
- Generated build folders, scratch analysis outputs, local logs, and Python cache files should not be committed.
