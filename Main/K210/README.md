# K210 音频跟踪模块

## 概述

K210 音频跟踪模块是一个基于麦克风阵列的智能声源定位和云台跟踪系统。该系统能够实时检测音频源的位置，并通过双轴云台进行精确跟踪。同时集成了与 ESP32S3 的 UART 串口通信功能，实现协同工作。本模块是 HRI4Seniors（老年人人机交互）项目的重要组成部分。

## 主要功能

### 音频目标检测
- **6+1 麦克风阵列**：6 个水平麦克风 + 1 个垂直麦克风
- **实时声源定位**：计算声源的水平和垂直方向误差
- **LED 方向指示**：通过 12 个 LED 环显示声源方向
- **噪声过滤**：可配置的误差忽略阈值，过滤微小干扰

### 云台控制系统
- **双轴控制**：俯仰轴（Pitch）和横滚轴（Roll）独立控制
- **PID 算法**：精确的比例-积分-微分控制算法
- **舵机驱动**：支持 50Hz PWM 舵机控制
- **范围限制**：可配置的运动范围和边界保护

### ESP32S3 串口通信
- **UART 双向通信**：与 ESP32S3 主控板实时数据交换
- **115200 波特率**：高速稳定的串口传输
- **非阻塞接收**：主循环中轮询接收，不影响音频跟踪性能
- **协同工作**：支持接收 ESP32 控制指令，反馈状态信息

### 实时显示
- **LCD 界面**：320x240 分辨率显示屏
- **状态监控**：实时显示云台角度和跟踪状态
- **可视化指示条**：直观显示俯仰和横滚角度

### 配置管理
- **JSON 配置文件**：支持从 `config.json` 加载自定义参数
- **默认配置回退**：配置文件缺失时自动使用内置默认值
- **热更新支持**：修改配置文件后重启即可生效

## 硬件配置

### 必需硬件
```
- K210 开发板
- 6 个水平麦克风（环形排列）
- 1 个垂直麦克风
- 双轴舵机云台
- LCD 显示屏（320x240）
- 12 个 SK9822 LED 灯
```

### 引脚连接

```python
# 麦克风阵列
i2s_d0=22      # 麦克风数据线0
i2s_d1=23      # 麦克风数据线1
i2s_d2=21      # 麦克风数据线2
i2s_d3=20      # 麦克风数据线3
i2s_ws=19      # 麦克风字时钟
i2s_sclk=18    # 麦克风串行时钟

# LED 控制
sk9822_dat=10  # LED 数据线
sk9822_clk=9   # LED 时钟线

# 舵机控制
pwm_pitch=7    # 俯仰轴舵机
pwm_roll=8     # 横滚轴舵机

# ESP32 UART 通信
uart1_rx=4     # K210 RX (接 ESP32 GPIO17 TX)
uart1_tx=5     # K210 TX (接 ESP32 GPIO18 RX)
# 波特率: 115200
```

## 核心算法

### 声源定位算法
系统通过分析 6+1 麦克风阵列的音频信号强度，计算声源的空间位置：

```python
# 垂直方向误差计算
height_err = (vertical_strength - horizontal_avg) / max(horizontal_avg, 1e-6) * out_range

# 水平方向误差计算  
for i in range(6):
    angle_rad = i * math.pi / 3 + angle_offset
    AngleX += direction[i] * math.sin(angle_rad)
    AngleY += direction[i] * math.cos(angle_rad)
horizontal_err = math.atan2(AngleX, AngleY) / math.pi * out_range
```

### PID 控制算法
采用经典的 PID 控制算法实现精确的云台跟踪：

- **比例项 (P)**：提供快速响应
- **积分项 (I)**：消除静态误差
- **微分项 (D)**：减少超调和振荡

```python
# PID 参数配置示例
config = {
    "pitch_pid": [0.5, 0.02, 0.03, 5],    # [P, I, D, I_max]
    "roll_pid": [0.5, 0.02, 0.03, 10],
    "pitch_reverse": False,
    "roll_reverse": True,
    "audio_range": 10,
    "ignore_threshold": 0.1,
    "roll_range": (10, 90)
}
```

## 核心类说明

### AudioTargetDetector

音频目标检测器，负责麦克风阵列的初始化和声源定位。

**主要方法：**

- `get_target_err()`: 返回声源的高度误差和水平误差
- `deinit()`: 释放麦克风资源

### Servo

舵机控制类，封装了舵机的 PWM 控制逻辑。

**主要参数：**

- `dir`: 初始位置 (0-100)
- `duty_min/max`: PWM 占空比范围
- `is_roll`: 是否为横滚轴控制

### PID

PID 控制器实现，提供精确的闭环控制。

**主要参数：**

- `p, i, d`: PID 参数
- `imax`: 积分限幅值
- `is_roll`: 是否为横滚轴（支持角度越界处理）

### Gimbal

云台控制器，整合音频检测和舵机控制。

**主要方法：**

- `run(height_err, horizontal_err)`: 执行一次跟踪控制

### UartComm

ESP32 串口通信类，负责与 ESP32S3 主控板的数据交互。

**主要方法：**

- `send(data)`: 发送数据到 ESP32（支持字符串和字节）
- `receive_line(timeout_ms)`: 接收一行数据（以 `\n` 结尾）
- `receive(timeout_ms)`: 非阻塞接收数据
- `start_receive_task(callback)`: 持续接收模式（阻塞式，需要独立任务）

**连接配置：**

- UART1: IO4(RX) ← ESP32 GPIO17(TX)
- UART1: IO5(TX) → ESP32 GPIO18(RX)
- 波特率: 115200
- 缓冲区: 4096 字节

## 使用方法

### 配置文件

系统支持通过 `config.json` 文件自定义配置参数。如果配置文件不存在或加载失败，系统会自动使用 `config.py` 中定义的默认配置。

**配置示例 (`config.json`):**

```json
{
  "init_pitch": 50,                     // 俯仰轴初始位置 (0-100)
  "init_roll": 50,                      // 横滚轴初始位置 (0-100)
  "pitch_pid": [0.5, 0.02, 0.03, 5],    // 俯仰轴 PID 参数 [P, I, D, I_max]
  "roll_pid": [0.5, 0.02, 0.03, 10],    // 横滚轴 PID 参数 [P, I, D, I_max]
  "pitch_reverse": false,               // 俯仰轴反向控制 (true=反向, false=正向)
  "roll_reverse": true,                 // 横滚轴反向控制 (true=反向, false=正向)
  "audio_range": 10,                    // 音频检测输出范围 (误差放大系数)
  "ignore_threshold": 0.1,              // 忽略阈值 (声音强度低于此值将被忽略)
  "roll_range": [10, 90],               // 横滚轴运动范围限制 [最小角度, 最大角度]
  "lcd_rotation": 0,                    // LCD 屏幕旋转角度 (0/90/180/270)
  "pitch_scale": 1.8,                   // 俯仰轴显示比例系数 (LCD 可视化缩放)
  "roll_scale": 1.8,                    // 横滚轴显示比例系数 (LCD 可视化缩放)
  "main_timeout": 120,                  // 主程序超时时间 (秒, 运行此时长后自动退出)
  "loop_delay": 0.01                    // 主循环延迟 (秒, 控制循环频率)
}
```

### 参数调试

通过修改 `config.json` 或直接修改 `config.py` 中的默认值来调整系统参数：

### 参数调整

#### 1. `ignore_threshold`（忽略阈值）

含义:

- 声音强度低于此阈值的声源会被忽略
- 数值越大 = 越不灵敏(过滤掉更多小声音)
- 数值越小 = 越灵敏(响应更多微弱声音)

效果：

```json
// 降低灵敏度(只响应大声音)
"ignore_threshold": 8    // 忽略强度 < 8 的声音,只响应强烈声源

// 提高灵敏度(响应小声音)
"ignore_threshold": 2    // 忽略强度 < 2 的声音,能检测到轻微声音

// 最灵敏(几乎响应所有声音)
"ignore_threshold": 0    // 不忽略任何声音
```

推荐值:

- 不灵敏(养老院环境,过滤背景噪音): 6-10
- 中等灵敏(正常室内): 3-5
- 高灵敏(安静环境,需要检测轻声): 1-2

#### 2. `audio_range` (音频检测范围)

含义:

- 定义有效声源的方向范围 [最小角度, 最大角度]
- 范围越窄 = 越不容易触发(只响应特定方向)
- 范围越宽 = 越容易触发(响应更大范围的声音)

效果：

```json
// 降低灵敏度(只响应正前方)
"audio_range": [-30, 30]   // 只检测 ±30° 范围内的声音

// 提高灵敏度(响应几乎所有方向)
"audio_range": [-160, 160] // 检测 ±160° 范围内的声音(接近360°)

// 中等灵敏度
"audio_range": [-90, 90]   // 检测 ±90° 范围内的声音(半圆)
```

推荐值:

- 不灵敏(只关注正前方): [-45, 45] 或 [-30, 30]
- 中等灵敏(前方半球): [-90, 90]
- 高灵敏(几乎全方位): [-150, 150]

### ESP32 通信协议

K210 通过 UART 与 ESP32S3 进行数据交换，支持以下通信模式：

#### 简单文本通信

```python
# 发送数据
uart_esp.send("Hello ESP32!\n")

# 接收数据
data = uart_esp.receive_line(timeout_ms=100)
if data:
    print("Received: {}".format(data))
```

#### JSON 协议通信（推荐）

建议使用 JSON 格式进行结构化数据交换：

```python
import json

# 发送 JSON 命令
command = {
    "cmd": "GIMBAL_STATUS",
    "pitch": pitch_servo.value,
    "roll": roll_servo.value
}
uart_esp.send(json.dumps(command) + "\n")

# 接收 JSON 命令
data = uart_esp.receive_line()
if data:
    try:
        msg = json.loads(data)
        if msg["cmd"] == "SET_POSITION":
            target_pitch = msg["pitch"]
            target_roll = msg["roll"]
    except:
        pass
```

#### 硬件连接

```text
ESP32S3          K210
GPIO17 (TX) ---> IO4 (RX)
GPIO18 (RX) <--- IO5 (TX)
GND ------------ GND
```

### 调参方案

#### 方案A：降低灵敏度

适用场景: 养老院、嘈杂环境、防止误触发

```json
{
  "ignore_threshold": 8,        // 忽略弱声音
  "audio_range": [-45, 45],     // 只检测正前方
  "pitch_pid": [0.3, 0.0, 0.1], // PID 可以稍微降低响应速度
  "roll_pid": [0.3, 0.0, 0.1]
}
```

效果:

- ✅ 只响应较大的声音(如正常说话、拍手)
- ✅ 忽略背景噪音、轻微声响
- ✅ 减少误触发

#### 方案B：提高灵敏度

适用场景: 安静房间、需要检测轻声、远距离声源

```json
{
  "ignore_threshold": 1,        // 检测微弱声音
  "audio_range": [-120, 120],   // 检测大范围方向
  "pitch_pid": [0.5, 0.0, 0.15], // PID 可以提高响应速度
  "roll_pid": [0.5, 0.0, 0.15]
}
```
效果:

- ✅ 能检测到轻声说话、轻微响动
- ✅ 响应更大范围的声源方向
- ⚠️ 可能会响应环境噪音

#### 方案C：中等灵敏度

适用场景: 一般室内环境

```json
{
  "ignore_threshold": 4,        // 中等阈值
  "audio_range": [-90, 90],     // 前方半圆
  "pitch_pid": [0.4, 0.0, 0.12],
  "roll_pid": [0.4, 0.0, 0.12]
}
```
效果:

- ✅ 响应正常音量的说话
- ✅ 过滤掉大部分背景噪音
- ✅ 平衡性能和准确性

### 其他参数

1. PID 参数

虽然主要控制云台响应速度,但也间接影响灵敏度:

```json
// 降低响应速度(减少抖动,看起来不那么"敏感")
"pitch_pid": [0.2, 0.0, 0.08]  // Kp 降低

// 提高响应速度(快速跟踪,看起来更"灵敏")
"pitch_pid": [0.6, 0.0, 0.18]  // Kp 提高
```

## 性能特点

- **低延迟**：音频处理到舵机响应延迟 < 50ms
- **高精度**：声源定位精度 ±5°
- **稳定性**：PID 控制确保平滑跟踪
- **可配置**：全面的参数调节接口
- **可视化**：实时状态显示和监控
- **协同工作**：与 ESP32S3 实时通信，支持分布式控制
- **容错机制**：配置文件缺失时自动回退到默认参数
- **超时保护**：主程序运行 120 秒后自动退出（可配置）

## 文件结构

```text
K210/
├── main.py              # 主程序入口
├── audio_detector.py    # 音频目标检测
├── servo.py             # 舵机控制
├── doa_calc.py          # DOA计算和PID控制
├── gimbal.py            # 云台控制器
├── led_ring.py          # LED环形灯和LCD显示
├── uart_comm.py         # ESP32 UART通信
├── config.py            # 配置管理
├── config.json          # 用户配置文件（可选）
├── boot.py              # K210启动脚本
├── requirements.txt     # Python依赖列表
└── README.md            # 本文档
```

## 应用场景

- **智能会议系统**：自动跟踪发言人
- **教育录播**：课堂教学自动摄像
- **安防监控**：声音触发的目标跟踪
- **机器人交互**：面向声源的自然交互
- **艺术装置**：声控互动展示
- **老年人关怀**：声音定位和响应（HRI4Seniors 项目）

## 扩展可能

- 集成视觉跟踪算法
- 多目标同时跟踪
- 无线控制接口
- 云端 AI 集成
- 语音识别联动
- 与 ESP32 深度集成（远程控制、状态同步）
- 多机协同（多个 K210 节点协作）
