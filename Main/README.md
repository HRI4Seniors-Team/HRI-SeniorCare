# Resona ESP32-S3 + MaixCam Pro Firmware Guide

This document records the current runnable hardware version of Resona. The current implementation uses MaixCam Pro for visual perception and ESP32-S3 for voice interaction, display, emotion fusion, servo tracking, and cloud upload.

## 1. Hardware used in the current build

| Module | Role | Current status |
| --- | --- | --- |
| ESP32-S3 board | Main controller, voice dialogue, display, fusion, cloud upload | Running |
| MaixCam Pro | Face detection and visual emotion sender | Running |
| ST7789 display | Local state and cute face display | Running |
| SG90 servo | Face-tracking motion output | Pan verified; tilt support present |
| Remote server | Emotion dashboard and warning records | `https://sievox.cn/resona` |

K210 is no longer the visual hardware for this version. Any old K210 notes are legacy references and should not be used for current wiring.

## 2. Current wiring

This section is the migration target for the revised hardware with the previous screen restored.

### 2.1 Wiring table

| Module | Signal | ESP32-S3 pin | Notes |
| --- | --- | --- | --- |
| Vision UART | MaixCam TX | GPIO38 | Vision JSON input |
| Vision UART | MaixCam RX | GPIO39 | ESP32 command/debug output |
| Vision UART | GND | GND | Common ground |
| ST7789 screen | MOSI | GPIO11 | SPI data |
| ST7789 screen | SCLK | GPIO12 | SPI clock |
| ST7789 screen | DC | GPIO13 | Data/command |
| ST7789 screen | RST | GPIO14 | Reset |
| ST7789 screen | CS | GPIO21 | Chip select |
| ST7789 screen | BL | GPIO2 | Backlight |
| ST7789 screen | VCC | 3.3 V | Logic power |
| ST7789 screen | GND | GND | Common ground |
| INMP441 | SCK | GPIO5 | I2S clock |
| INMP441 | WS | GPIO4 | I2S word select |
| INMP441 | SD | GPIO6 | I2S data |
| INMP441 | VDD | 3.3 V | Logic power |
| INMP441 | GND | GND | Common ground |
| MAX98357A | DIN | GPIO7 | I2S data |
| MAX98357A | BCLK | GPIO15 | I2S bit clock |
| MAX98357A | LRC/WS | GPIO16 | I2S word select |
| MAX98357A | VIN | 5 V | Power input |
| MAX98357A | GND | GND | Common ground |
| SG90 pan servo | Signal | GPIO41 | Horizontal axis |
| SG90 pan servo | VCC | 5 V | External supply recommended |
| SG90 pan servo | GND | GND | Common ground |
| SG90 tilt servo | Signal | GPIO18 | Vertical axis |
| SG90 tilt servo | VCC | 5 V | External supply recommended |
| SG90 tilt servo | GND | GND | Common ground |
| Buttons | Volume+ | GPIO40 | Button input |
| Buttons | Volume- | GPIO17 | Button input |
| Buttons | BOOT | GPIO0 | Strapping pin |
| Buttons | Custom | GPIO47 | User-defined input |

### 2.2 Practical notes

- GPIO0 is a strapping pin. Keep it only for BOOT and avoid adding extra load.
- GPIO38/39 are reserved for the MaixCam vision UART pair.
- GPIO2 is consumed by the screen backlight.
- GPIO41/18 are used by the servo signals in this revision.
- Servos and MAX98357A must use a separate 5 V supply, with all grounds tied together at a single common ground.
- This wiring table is the migration target for the previous screen hardware. Keep the MaixCam vision node on GPIO38/39.

## 3. Runtime data flow

```text
MaixCam Pro visual node
  -> UART JSON frames
  -> ESP32 UART parser
  -> vision state cache
  -> audio-visual fusion
  -> LVGL cute face display / servo tracking / LLM context
  -> HTTPS emotion upload
  -> https://sievox.cn/resona dashboard
```

The ESP32 uploads regular fused emotion states and high-conflict warning records through:

```text
https://sievox.cn/resona/emotion/write
```

Dashboard-side checks commonly use:

```text
https://sievox.cn/resona/emotion/current
https://sievox.cn/resona/emotion/history
https://sievox.cn/resona/emotion/meta
```

## 4. MaixCam Pro vision script

Current script:

```text
Main/MaixCAM/resona_visual_node.py
```

The script configures:

```python
UART_DEVICE = "/dev/ttyS0"
UART_TX_PIN = "A16"
UART_RX_PIN = "A17"
UART_BAUD = 115200
```

Upload this file to the MaixCam Pro and run it as the active vision application. The ESP32 expects newline-delimited JSON packets with a CRC field. The parser also records summary counters for raw bytes, lines, packets, parse failures, CRC failures, drops, and header bytes.

Expected ESP32-side health signal:

```text
UART summary: ... lines > 0 parse_fail=0 crc_fail=0 drops=0
```

## 5. ESP32 firmware build and flash

The verified local ESP-IDF environment uses ESP-IDF v5.4 and MSYS2 Bash.

From PowerShell:

```powershell
& 'C:\msys64\usr\bin\bash.exe' -lc "source /c/Users/25453/esp-activate.sh && cd /c/Users/25453/Desktop/HRISenior/HRISenior/HRI-SeniorCare/Main && idf.py build"
```

Flash to COM5:

```powershell
& 'C:\msys64\usr\bin\bash.exe' -lc "source /c/Users/25453/esp-activate.sh && cd /c/Users/25453/Desktop/HRISenior/HRISenior/HRI-SeniorCare/Main && idf.py -p COM5 flash"
```

Use the explicit `C:\msys64\usr\bin\bash.exe` path. On this machine, plain `bash` may resolve to the Windows WSL launcher and fail to source the ESP-IDF environment.

## 6. Current implemented features

### 6.1 Visual emotion and dialogue context

MaixCam Pro sends the latest visual emotion state to ESP32. The ESP32 caches the latest visual result and injects a concise visual summary into the dialogue context. This allows the LLM to answer questions such as “你看见我什么情绪” using the latest vision state.

### 6.2 Audio-visual fusion

The firmware contains a Dempster-Shafer-style fusion module under:

```text
Main/main/emotion/
```

It fuses visual emotion evidence and audio-derived emotion evidence, then outputs dominant emotion, confidence score, conflict value, and high-conflict state.

### 6.3 Cloud upload and warning records

Regular fused emotion states are uploaded periodically. High-conflict events are not announced locally with an alarm because that interrupts speech output. Instead, the firmware uploads a warning record with fields such as:

```text
record_type = emotion_warning
warning_level = high_conflict
warning_message = Detected possible hidden distress...
```

The server can display these entries in the emotion warning record area.

### 6.4 Cute face display

The LVGL display no longer relies on plain emoji-only rendering. The current renderer uses a Q-style face with larger eyes, highlighted pupils, curved brow styling, and emotion-dependent expression changes.

Main files:

```text
Main/main/display/lvgl_display/cute_face_view.cc
Main/main/display/lvgl_display/cute_face_view.h
```

### 6.5 Face tracking servo

Servo tracking is implemented under:

```text
Main/main/servo/
```

The pan servo has been verified with MaixCam face input. Tilt support is implemented for the second SG90 servo, but mechanical installation and calibration should be checked before long-term operation.

## 7. Validation checklist

After flashing, check the following in order:

1. ESP32 boots without repeated reset.
2. Display shows the current dialogue state and Q-style face.
3. MaixCam Pro script starts and reports UART initialization on A16/A17.
4. ESP32 UART summary shows increasing line and packet counters with zero parse/CRC drops.
5. Moving a face in front of MaixCam changes visual state and servo output.
6. Speaking to the ESP32 triggers the voice dialogue path.
7. Asking “你看见我什么情绪” makes the model cite the visual emotion result.
8. `https://sievox.cn/resona` shows updated emotion data.
9. High-conflict events are uploaded as server-side warning records without local alarm playback.

## 8. Important limitations

- Firmware size is very close to the current app partition limit. The latest checked build left only a few hundred bytes of free space. Add new features only after checking binary size, or adjust the partition table.
- The UART component folder is still named `uart_k210` for compatibility with older code, but the current peer device is MaixCam Pro.
- The cloud `online` field may represent server-side long-connection state rather than HTTP upload status. For the current lightweight chain, successful `Cloud upload OK` logs and dashboard data updates are the primary health checks.
- Do not commit local build folders, scratch analysis data, Python cache files, hardware logs, or backup folders.

## 9. Git hygiene

Before pushing, inspect:

```powershell
git status --short
git diff --stat
```

Commit source, scripts, and documentation only. Exclude:

```text
Main/build*/
Main/.scratch*/
Main/hardware_logs/
Main/backup*/
__pycache__/
*.pyc
*.log
```
