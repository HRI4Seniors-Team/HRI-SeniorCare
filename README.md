# An MCP-based Chatbot

（中文 | [English](README_en.md) | [日本語](README_ja.md)）

## 项目说明

本项目为基于 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) (v2.0.4版本)项目的扩展，以下是一些本项目做的一些扩展功能说明：
- 新增 K210 音频跟踪模块
	- 音频目标检测：接入 MEMS7 麦克风阵列，并实现实时声源定位。
	- 云台控制系统：接入两个 sg90 云台进行双轴控制，自动追踪声源，为后续接入摄像头等其他设备提供准备。
	- ESP32S3 串口通信：实现 ESP32S3 与 K210 的 UART 双向通信。
	- 状态显示：在 K210 LCD 屏上实时显示云台角度和追踪状态等信息。
- MCP 服务：实现通过语音来控制舵机的旋转与状态。

项目使用的开发板为：ESP32S3(对应 `board->bread-compact-wifi`)、Sipeed Maixbit K210。

## 引脚对接与参数配置

### 引脚对接说明

#### **Sipeed Maixbit K210 与 MEMS7麦克风引脚对接**

```python
mic.init(
    i2s_d0=22,
    i2s_d1=23,
    i2s_d2=21,
    i2s_d3=20,
    i2s_ws=19,
    i2s_sclk=18, # MIC_CK
    sk9822_dat=10,
    sk9822_clk=9 # LED_CK
)
```

#### **K210 与 两个 sg90 的 PWM 引脚对接**

| K210 PWM | sg90  |
| -------- | ----- |
| IO7      | Pitch |
| IO8      | Roll  |

#### **ESP32S3 N16R8 与 Sipeed Maixbit K210 串口通信引脚对接**


| esp32s3<br>UART1 | K210<br>UARTHS  |
| ---------------- | --------------- |
| GPIO17 U1TXD     | IO4 ISP_RX (13) |
| GPIO18 U1RXD     | IO5 ISP_TX (12) |
| GND              | GND             |

### 舵机控制相关配置

#### 参数示例配置

```yaml
{
  "init_pitch": 50,                     # 俯仰轴初始位置 (0-100)
  "init_roll": 50,                      # 横滚轴初始位置 (0-100)
  "pitch_pid": [0.5, 0.02, 0.03, 5],    # 俯仰轴 PID 参数 [P, I, D, I_max]
  "roll_pid": [0.5, 0.02, 0.03, 10],    # 横滚轴 PID 参数 [P, I, D, I_max]
  "pitch_reverse": false,               # 俯仰轴反向控制 (true=反向, false=正向)
  "roll_reverse": true,                 # 横滚轴反向控制 (true=反向, false=正向)
  "audio_range": 10,                    # 音频检测输出范围 (误差放大系数)
  "ignore_threshold": 0.1,              # 忽略阈值 (声音强度低于此值将被忽略)
  "roll_range": [10, 90],               # 横滚轴运动范围限制 [最小角度, 最大角度]
  "lcd_rotation": 0,                    # LCD 屏幕旋转角度 (0/90/180/270)
  "pitch_scale": 1.8,                   # 俯仰轴显示比例系数 (LCD 可视化缩放)
  "roll_scale": 1.8,                    # 横滚轴显示比例系数 (LCD 可视化缩放)
  "main_timeout": 120,                  # 主程序超时时间 (秒, 运行此时长后自动退出)
  "loop_delay": 0.01                    # 主循环延迟 (秒, 控制循环频率)
}
```


#### 参数调整

通过修改 `config.json` 或直接修改 `config.py` 中的默认值来调整系统参数：

##### 1. `ignore_threshold`（忽略阈值）

含义:

- 声音强度低于此阈值的声源会被忽略
- 数值越大 = 越不灵敏(过滤掉更多小声音)
- 数值越小 = 越灵敏(响应更多微弱声音)

效果：

```yaml
# 降低灵敏度(只响应大声音)
"ignore_threshold": 8    # 忽略强度 < 8 的声音,只响应强烈声源

# 提高灵敏度(响应小声音)
"ignore_threshold": 2    # 忽略强度 < 2 的声音,能检测到轻微声音

# 最灵敏(几乎响应所有声音)
"ignore_threshold": 0    # 不忽略任何声音
```

推荐值:

- 不灵敏(养老院环境,过滤背景噪音): 6-10
- 中等灵敏(正常室内): 3-5
- 高灵敏(安静环境,需要检测轻声): 1-2

##### 2. `audio_range` (音频检测范围)

含义:

- 定义有效声源的方向范围 [最小角度, 最大角度]
- 范围越窄 = 越不容易触发(只响应特定方向)
- 范围越宽 = 越容易触发(响应更大范围的声音)

效果：

```yaml
# 降低灵敏度(只响应正前方)
"audio_range": [-30, 30]   # 只检测 ±30° 范围内的声音

# 提高灵敏度(响应几乎所有方向)
"audio_range": [-160, 160] # 检测 ±160° 范围内的声音(接近360°)

# 中等灵敏度
"audio_range": [-90, 90]   # 检测 ±90° 范围内的声音(半圆)
```

推荐值:

- 不灵敏(只关注正前方): [-45, 45] 或 [-30, 30]
- 中等灵敏(前方半球): [-90, 90]
- 高灵敏(几乎全方位): [-150, 150]

## 串口通信与 MCP 说明

### UART 通信

**K210 端**
```python
class UartComm:
    def __init__(self):
        # 初始化 UART1, 波特率 115200
        # K210: IO4=RX(接ESP32的TX/GPIO17), IO5=TX(接ESP32的RX/GPIO18)

        # try:
        #     fm.unregister(fm.fpioa.UART1_RX)
        # except ValueError:
        #     pass
        # try:
        #     fm.unregister(fm.fpioa.UART1_TX)
        # except ValueError:
        #     pass

        # 先释放原来的映射，避免和 REPL 冲突
        for func in (fm.fpioa.UARTHS_RX, fm.fpioa.UARTHS_TX):
            try:
                fm.unregister(func)
            except ValueError:
                pass

        try:
            # fm.register(13, fm.fpioa.UART1_RX, force=True)  # IO4 ← ESP32 TX
            # fm.register(12, fm.fpioa.UART1_TX, force=True)  # IO5 → ESP32 RX

            fm.register(board_info.PIN4, fm.fpioa.UARTHS_RX, force=True)  # IO4 ← ESP32 TX
            fm.register(board_info.PIN5, fm.fpioa.UARTHS_TX, force=True)  # IO5 → ESP32 RX

            print("(K210) UART pins registered: RX=IO4, TX=IO5")
        except:
            print("(K210) Failed to register UART1 pins")
            pass

        self.uart = UART(UART.UARTHS, 115200, read_buf_len=4096)
        print("(K210) UART initialized: 115200 baud")

    # 清空缓冲区
        if self.uart.any():
            self.uart.read()
            print("(K210) Cleared UART buffer")

    def send(self, data):
        """发送数据到 ESP32"""
        if isinstance(data, str):
            data = data.encode('utf-8')
        self.uart.write(data)
        # print(f"Sent to ESP32: {data}")
        print("Sent to ESP32: {}".format(data))
    
    def receive(self, timeout_ms=100):
        """接收 ESP32 发来的数据"""
        if self.uart.any():
            data = self.uart.read()
            if data:
                try:
                    return data.decode('utf-8')
                except:
                    return data  # 返回原始字节
        return None
    
    def receive_line(self, timeout_ms=1000):
        """接收一行数据(以 \n 结尾)"""
        start = time.ticks_ms()
        buffer = b''
        
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                char = self.uart.read(1)
                print('(K210) Received char: {}'.format(char))
                if char:
                    buffer += char
                    if char == b'\n':
                        try:
                            return buffer.decode('utf-8').strip()
                        except:
                            return buffer
            time.sleep_ms(10)
        
        # 超时检查
        if buffer:
            result = buffer.decode('utf-8').strip() if buffer else None
            print("(K210) Timeout with partial data: [{}]".format(result))
            return result
        print("(K210) Receive line timeout with no data")
        return None
    
    def start_receive_task(self, callback):
        """持续接收数据并调用回调函数处理
        
        Args:
            callback: 回调函数,接收一个参数(接收到的数据)
        """
        while True:
            data = self.receive_line()
            if data:
                # print(f"Received from ESP32: {data}")
                print("(K210) Received from ESP32: {}".format(data))
                callback(data)
            time.sleep_ms(10)
```

**ESP32S3端**

`uart_K210.cc`
```C
#include "uart_K210.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "UART_K210(ESP32)"

void UartK210::Init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_, TX_PIN, RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_, BUF_SIZE, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", 
             TX_PIN, RX_PIN, BAUD_RATE);
}

void UartK210::SendData(const char* data, size_t len) {
    uart_write_bytes(UART_NUM_, data, len);
    ESP_LOGI(TAG, "Sent to K210: %.*s (LOG)", len, data);
}

int UartK210::ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_NUM_, buffer, max_len, 
                          pdMS_TO_TICKS(timeout_ms));
}

void UartK210::StartReceiveTask() {
    xTaskCreate([](void* param) {
        UartK210* uart = static_cast<UartK210*>(param);
        uint8_t buffer[BUF_SIZE];
        size_t index = 0;
        
        while (1) {
            uint8_t byte;
            int len = uart->ReceiveData(&byte, 1, 100);  // 每次读 1 字节
            
            if (len > 0) {
                if (byte == '\n') {
                    // 收到完整一行
                    buffer[index] = '\0';
                    ESP_LOGI(TAG, "Received line: %s", buffer);
                    index = 0;  // 重置缓冲区
                } else if (index < BUF_SIZE - 1) {
                    buffer[index++] = byte;
                } else {
                    // 缓冲区满，丢弃
                    ESP_LOGW(TAG, "Buffer overflow, resetting");
                    index = 0;
                }
            }
        }
    }, "uart_rx_task", 4096, this, 5, NULL);
}
```

### MCP 的注册

`gimbal_controller.h`
```C
void RegisterMcpTools() {
	auto& mcp_server = McpServer::GetInstance();
	
	// 获取云台状态
	mcp_server.AddTool("gimbal.get_state", 
		"Get the current state of the gimbal (pitch and roll servo positions)", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("GET_STATE");
			ESP_LOGI(TAG, "Request gimbal state");
			return true;
		});

	// 横滚轴左转
	mcp_server.AddTool("gimbal.roll.turn_left", 
		"Turn the roll servo to the left", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ROLL_LEFT");
			ESP_LOGI(TAG, "Roll left");
			return true;
		});

	// 横滚轴右转
	mcp_server.AddTool("gimbal.roll.turn_right", 
		"Turn the roll servo to the right", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ROLL_RIGHT");
			ESP_LOGI(TAG, "Roll right");
			return true;
		});

	// 俯仰轴上转
	mcp_server.AddTool("gimbal.pitch.turn_up", 
		"Turn the pitch servo up", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("PITCH_UP");
			ESP_LOGI(TAG, "Pitch up");
			return true;
		});

	// 俯仰轴下转
	mcp_server.AddTool("gimbal.pitch.turn_down", 
		"Turn the pitch servo down", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("PITCH_DOWN");
			ESP_LOGI(TAG, "Pitch down");
			return true;
		});

	// 保持当前位置
	mcp_server.AddTool("gimbal.hold_position", 
		"Keep the servo in its current position (stop audio tracking)", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("HOLD_POSITION");
			ESP_LOGI(TAG, "Hold position");
			return true;
		});

	// 回归初始位置
	mcp_server.AddTool("gimbal.reset", 
		"Reset the gimbal to initial position", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("RESET");
			ESP_LOGI(TAG, "Reset to initial position");
			return true;
		});

	// 启用音频跟踪
	mcp_server.AddTool("gimbal.enable_tracking", 
		"Enable audio source tracking", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ENABLE_TRACKING");
			ESP_LOGI(TAG, "Enable audio tracking");
			return true;
		});

	// 禁用音频跟踪
	mcp_server.AddTool("gimbal.disable_tracking", 
		"Disable audio source tracking", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("DISABLE_TRACKING");
			ESP_LOGI(TAG, "Disable audio tracking");
			return true;
		});
}
```

## 介绍

👉 [人类：给 AI 装摄像头 vs AI：当场发现主人三天没洗头【bilibili】](https://www.bilibili.com/video/BV1bpjgzKEhd/)

👉 [手工打造你的 AI 女友，新手入门教程【bilibili】](https://www.bilibili.com/video/BV1XnmFYLEJN/)

小智 AI 聊天机器人作为一个语音交互入口，利用 Qwen / DeepSeek 等大模型的 AI 能力，通过 MCP 协议实现多端控制。

<img src="docs/mcp-based-graph.jpg" alt="通过MCP控制万物" width="320">

### 版本说明

当前 v2 版本与 v1 版本分区表不兼容，所以无法从 v1 版本通过 OTA 升级到 v2 版本。分区表说明参见 [partitions/v2/README.md](partitions/v2/README.md)。

使用 v1 版本的所有硬件，可以通过手动烧录固件来升级到 v2 版本。

v1 的稳定版本为 1.9.2，可以通过 `git checkout v1` 来切换到 v1 版本，该分支会持续维护到 2026 年 2 月。

### 已实现功能

- Wi-Fi / ML307 Cat.1 4G
- 离线语音唤醒 [ESP-SR](https://github.com/espressif/esp-sr)
- 支持两种通信协议（[Websocket](docs/websocket.md) 或 MQTT+UDP）
- 采用 OPUS 音频编解码
- 基于流式 ASR + LLM + TTS 架构的语音交互
- 声纹识别，识别当前说话人的身份 [3D Speaker](https://github.com/modelscope/3D-Speaker)
- OLED / LCD 显示屏，支持表情显示
- 电量显示与电源管理
- 支持多语言（中文、英文、日文）
- 支持 ESP32-C3、ESP32-S3、ESP32-P4 芯片平台
- 通过设备端 MCP 实现设备控制（音量、灯光、电机、GPIO 等）
- 通过云端 MCP 扩展大模型能力（智能家居控制、PC桌面操作、知识搜索、邮件收发等）
- K210 音频目标检测与云台跟踪系统，支持声源定位和自动跟随
- 自定义唤醒词、字体、表情与聊天背景，支持网页端在线修改 ([自定义Assets生成器](https://github.com/78/xiaozhi-assets-generator))

## 硬件

### K210 音频跟踪模块

K210 模块实现了基于麦克风阵列的音频目标检测和云台跟踪功能：

**主要功能：**
- 6 个麦克风环形阵列进行声源定位
- 实时声源高度和水平方向误差计算
- PID 控制算法驱动俯仰和横滚双轴云台
- LCD 显示云台状态和角度信息
- 支持声纹识别和目标跟随

**技术特点：**
- 使用 OPUS 音频编解码和 ESP-SR 离线语音唤醒
- 基于数学算法计算声源向量和角度偏移
- 支持参数可配置的 PID 控制器
- 低延迟音频处理和舵机响应
- 可视化界面显示跟踪状态

**硬件要求：**
- K210 开发板
- 6 麦克风环形阵列 + 1 个垂直麦克风
- 双轴舵机云台（俯仰 + 横滚）
- LCD 显示屏（320x240）
- LED 环用于方向指示

### 面包板手工制作实践

详见飞书文档教程：

👉 [《小智 AI 聊天机器人百科全书》](https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb?from=from_copylink)

面包板效果图如下：

![面包板效果图](docs/v1/wiring2.jpg)

### 支持 70 多个开源硬件（仅展示部分）

- <a href="https://oshwhub.com/li-chuang-kai-fa-ban/li-chuang-shi-zhan-pai-esp32-s3-kai-fa-ban" target="_blank" title="立创·实战派 ESP32-S3 开发板">立创·实战派 ESP32-S3 开发板</a>
- <a href="https://github.com/espressif/esp-box" target="_blank" title="乐鑫 ESP32-S3-BOX3">乐鑫 ESP32-S3-BOX3</a>
- <a href="https://docs.m5stack.com/zh_CN/core/CoreS3" target="_blank" title="M5Stack CoreS3">M5Stack CoreS3</a>
- <a href="https://docs.m5stack.com/en/atom/Atomic%20Echo%20Base" target="_blank" title="AtomS3R + Echo Base">M5Stack AtomS3R + Echo Base</a>
- <a href="https://gf.bilibili.com/item/detail/1108782064" target="_blank" title="神奇按钮 2.4">神奇按钮 2.4</a>
- <a href="https://www.waveshare.net/shop/ESP32-S3-Touch-AMOLED-1.8.htm" target="_blank" title="微雪电子 ESP32-S3-Touch-AMOLED-1.8">微雪电子 ESP32-S3-Touch-AMOLED-1.8</a>
- <a href="https://github.com/Xinyuan-LilyGO/T-Circle-S3" target="_blank" title="LILYGO T-Circle-S3">LILYGO T-Circle-S3</a>
- <a href="https://oshwhub.com/tenclass01/xmini_c3" target="_blank" title="虾哥 Mini C3">虾哥 Mini C3</a>
- <a href="https://oshwhub.com/movecall/cuican-ai-pendant-lights-up-y" target="_blank" title="Movecall CuiCan ESP32S3">璀璨·AI 吊坠</a>
- <a href="https://github.com/WMnologo/xingzhi-ai" target="_blank" title="无名科技Nologo-星智-1.54">无名科技 Nologo-星智-1.54TFT</a>
- <a href="https://www.seeedstudio.com/SenseCAP-Watcher-W1-A-p-5979.html" target="_blank" title="SenseCAP Watcher">SenseCAP Watcher</a>
- <a href="https://www.bilibili.com/video/BV1BHJtz6E2S/" target="_blank" title="ESP-HI 超低成本机器狗">ESP-HI 超低成本机器狗</a>

<div style="display: flex; justify-content: space-between;">
  <a href="docs/v1/lichuang-s3.jpg" target="_blank" title="立创·实战派 ESP32-S3 开发板">
    <img src="docs/v1/lichuang-s3.jpg" width="240" />
  </a>
  <a href="docs/v1/espbox3.jpg" target="_blank" title="乐鑫 ESP32-S3-BOX3">
    <img src="docs/v1/espbox3.jpg" width="240" />
  </a>
  <a href="docs/v1/m5cores3.jpg" target="_blank" title="M5Stack CoreS3">
    <img src="docs/v1/m5cores3.jpg" width="240" />
  </a>
  <a href="docs/v1/atoms3r.jpg" target="_blank" title="AtomS3R + Echo Base">
    <img src="docs/v1/atoms3r.jpg" width="240" />
  </a>
  <a href="docs/v1/magiclick.jpg" target="_blank" title="神奇按钮 2.4">
    <img src="docs/v1/magiclick.jpg" width="240" />
  </a>
  <a href="docs/v1/waveshare.jpg" target="_blank" title="微雪电子 ESP32-S3-Touch-AMOLED-1.8">
    <img src="docs/v1/waveshare.jpg" width="240" />
  </a>
  <a href="docs/v1/lilygo-t-circle-s3.jpg" target="_blank" title="LILYGO T-Circle-S3">
    <img src="docs/v1/lilygo-t-circle-s3.jpg" width="240" />
  </a>
  <a href="docs/v1/xmini-c3.jpg" target="_blank" title="虾哥 Mini C3">
    <img src="docs/v1/xmini-c3.jpg" width="240" />
  </a>
  <a href="docs/v1/movecall-cuican-esp32s3.jpg" target="_blank" title="CuiCan">
    <img src="docs/v1/movecall-cuican-esp32s3.jpg" width="240" />
  </a>
  <a href="docs/v1/wmnologo_xingzhi_1.54.jpg" target="_blank" title="无名科技Nologo-星智-1.54">
    <img src="docs/v1/wmnologo_xingzhi_1.54.jpg" width="240" />
  </a>
  <a href="docs/v1/sensecap_watcher.jpg" target="_blank" title="SenseCAP Watcher">
    <img src="docs/v1/sensecap_watcher.jpg" width="240" />
  </a>
  <a href="docs/v1/esp-hi.jpg" target="_blank" title="ESP-HI 超低成本机器狗">
    <img src="docs/v1/esp-hi.jpg" width="240" />
  </a>
</div>

## 软件

### 固件烧录

新手第一次操作建议先不要搭建开发环境，直接使用免开发环境烧录的固件。

固件默认接入 [xiaozhi.me](https://xiaozhi.me) 官方服务器，个人用户注册账号可以免费使用 Qwen 实时模型。

👉 [新手烧录固件教程](https://ccnphfhqs21z.feishu.cn/wiki/Zpz4wXBtdimBrLk25WdcXzxcnNS)

### 开发环境

- Cursor 或 VSCode
- 安装 ESP-IDF 插件，选择 SDK 版本 5.4 或以上
- Linux 比 Windows 更好，编译速度快，也免去驱动问题的困扰
- 本项目使用 Google C++ 代码风格，提交代码时请确保符合规范

### 开发者文档

- [自定义开发板指南](docs/custom-board.md) - 学习如何为小智 AI 创建自定义开发板
- [MCP 协议物联网控制用法说明](docs/mcp-usage.md) - 了解如何通过 MCP 协议控制物联网设备
- [MCP 协议交互流程](docs/mcp-protocol.md) - 设备端 MCP 协议的实现方式
- [K210 音频跟踪模块](K210/main.py) - 基于麦克风阵列的声源定位和云台跟踪系统
- [MQTT + UDP 混合通信协议文档](docs/mqtt-udp.md)
- [一份详细的 WebSocket 通信协议文档](docs/websocket.md)

## 大模型配置

如果你已经拥有一个小智 AI 聊天机器人设备，并且已接入官方服务器，可以登录 [xiaozhi.me](https://xiaozhi.me) 控制台进行配置。

👉 [后台操作视频教程（旧版界面）](https://www.bilibili.com/video/BV1jUCUY2EKM/)

## 相关开源项目

在个人电脑上部署服务器，可以参考以下第三方开源的项目：

- [xinnan-tech/xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) Python 服务器
- [joey-zhou/xiaozhi-esp32-server-java](https://github.com/joey-zhou/xiaozhi-esp32-server-java) Java 服务器
- [AnimeAIChat/xiaozhi-server-go](https://github.com/AnimeAIChat/xiaozhi-server-go) Golang 服务器

使用小智通信协议的第三方客户端项目：

- [huangjunsen0406/py-xiaozhi](https://github.com/huangjunsen0406/py-xiaozhi) Python 客户端
- [TOM88812/xiaozhi-android-client](https://github.com/TOM88812/xiaozhi-android-client) Android 客户端
- [100askTeam/xiaozhi-linux](http://github.com/100askTeam/xiaozhi-linux) 百问科技提供的 Linux 客户端
- [78/xiaozhi-sf32](https://github.com/78/xiaozhi-sf32) 思澈科技的蓝牙芯片固件
- [QuecPython/solution-xiaozhiAI](https://github.com/QuecPython/solution-xiaozhiAI) 移远提供的 QuecPython 固件

## 关于项目

这是一个由虾哥开源的 ESP32 项目，以 MIT 许可证发布，允许任何人免费使用，修改或用于商业用途。

我们希望通过这个项目，能够帮助大家了解 AI 硬件开发，将当下飞速发展的大语言模型应用到实际的硬件设备中。

如果你有任何想法或建议，请随时提出 Issues 或加入 QQ 群：1011329060

## Star History

<a href="https://star-history.com/#78/xiaozhi-esp32&Date">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date" />
 </picture>
</a>
