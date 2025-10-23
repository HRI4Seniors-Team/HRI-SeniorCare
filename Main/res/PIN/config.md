### **sipeed maixbit k210 与 MEMS7麦克风引脚对接**

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


### **k210 与 两个 sg90 的 PWM 引脚对接**

| k210 PWM | sg90  |
| -------- | ----- |
| IO7      | Pitch |
| IO8      | Roll  |

### **esp32s3 N16R8 与 sipeed maixbit k210 串口通信引脚对接**

esp32s3 ---- k210
GPIO17 U1TXD ---- 

| esp32s3<br>UART1 | k210<br>UART0 |
| ---------------- | ------------- |
| GPIO17 U1TXD     | IO4 ISP_RX    |
| GPIO18 U1RXD     | IO5 ISP_TX    |
| GND              | GND           |
