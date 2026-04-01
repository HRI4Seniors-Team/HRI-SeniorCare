## 引脚对接说明
### **sipeed maixbit k210 与 MEMS7麦克风引脚对接**

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

### **k210 与 两个 sg90 的 PWM 引脚对接**

| k210 PWM | sg90  |
| -------- | ----- |
| IO7      | Pitch |
| IO8      | Roll  |

### **ESP32S3 N16R8 与 sipeed maixbit k210 串口通信引脚对接**


| esp32s3<br>UART1 | k210<br>UARTHS|
| ---------------- | ------------- |
| GPIO17 U1TXD     | IO4 ISP_RX (13) |
| GPIO18 U1RXD     | IO5 ISP_TX (12) |
| GND              | GND           |

## 舵机控制相关配置

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

