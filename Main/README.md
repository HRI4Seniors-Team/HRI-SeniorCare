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

### 2.1 MaixCam Pro UART to ESP32-S3

| Signal | MaixCam Pro | ESP32-S3 | Notes |
| --- | --- | --- | --- |
| Vision TX | A16 / TX | GPIO38 / UART1 RX | Use a series resistor. The tested stable value is about 5 kΩ. |
| Vision RX | A17 / RX | GPIO39 / UART1 TX | Optional for one-way vision input, kept for bidirectional debugging. |
| Ground | GND | GND | Common ground is required. |
| Baud rate | 115200 | 115200 | 8N1, newline-delimited JSON. |

Earlier tests showed boot instability when using several strapping or conflicted pins. The current safe UART pair is GPIO38/GPIO39.

MaixCam-side script configuration:

```python
UART_DEVICE = "/dev/ttyS0"
UART_TX_PIN = "A16"
UART_RX_PIN = "A17"
UART_BAUD = 115200
```

ESP32-side UART configuration:

```text
UART port: UART1
TX: GPIO39
RX: GPIO38
Baud: 115200
Frame: 8N1
Protocol: newline-delimited JSON with CRC
```

### 2.2 Servo output and power

| Servo wire | Connect to | Notes |
| --- | --- | --- |
| Pan servo signal | ESP32-S3 GPIO41 | Horizontal XOY face tracking |
| Tilt servo signal | ESP32-S3 GPIO18 | Vertical XOZ face tracking |
| Servo VCC | External 5 V power positive | Recommended for SG90 stability |
| Servo GND | External power GND and ESP32 GND | Common ground is required |

Servo power should be supplied from an external 5 V source when possible. ESP32 GND and servo power GND must be connected together. Avoid powering multiple SG90 servos directly from the ESP32 3.3 V pin.

Current servo PWM parameters:

```text
Frequency: 50 Hz
Pulse width range: 500-2500 us
Angle clamp: 20-160 degrees
Initial angle: 90 degrees
```

### 2.3 ST7789 display wiring

| Display signal | ESP32-S3 pin | Notes |
| --- | --- | --- |
| SDA / DIN / MOSI | GPIO11 | SPI data |
| SCL / SCK | GPIO12 | SPI clock |
| DC | GPIO13 | Data/command |
| RST | GPIO14 | Reset |
| CS | GPIO21 | Chip select |
| BL | GPIO2 | Backlight |
| VCC | 3.3 V | Use the display module's required logic voltage |
| GND | GND | Common ground |

Display parameters:

```text
Resolution: 240 x 240
SPI mode: 0
Invert color: true
RGB order: RGB
```

### 2.4 Audio wiring

| Audio module signal | ESP32-S3 pin | Purpose |
| --- | --- | --- |
| Microphone WS | GPIO4 | I2S microphone word select |
| Microphone SCK | GPIO5 | I2S microphone clock |
| Microphone DIN | GPIO6 | I2S microphone data input |
| Speaker DOUT | GPIO7 | I2S speaker data output |
| Speaker BCLK | GPIO15 | I2S speaker bit clock |
| Speaker LRCK | GPIO16 | I2S speaker left/right clock |

### 2.5 Buttons, LED, and auxiliary output

| Function | Pin |
| --- | --- |
| Boot button | GPIO0 |
| Touch button | GPIO47 |
| Volume up button | GPIO40 |
| Volume down button | GPIO17 |
| Built-in LED | GPIO48 |
| Lamp output | GPIO10 |

GPIO40 is reserved for volume-up input in the current board profile and should not be reused for a servo signal.

### 2.6 Pins to avoid reusing

| Pin or group | Reason |
| --- | --- |
| GPIO0 | Boot mode strap/button |
| GPIO4 / GPIO5 / GPIO6 | Microphone I2S |
| GPIO7 / GPIO15 / GPIO16 | Speaker I2S |
| GPIO11 / GPIO12 / GPIO13 / GPIO14 / GPIO21 / GPIO2 | ST7789 display |
| GPIO17 | Volume down button |
| GPIO38 / GPIO39 | MaixCam Pro UART |
| GPIO40 | Volume up button |
| GPIO41 / GPIO18 | SG90 servo outputs |
| GPIO48 | Built-in LED |

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
