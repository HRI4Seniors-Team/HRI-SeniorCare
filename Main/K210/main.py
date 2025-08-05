import math
import time
from Maix import MIC_ARRAY as mic
import lcd
from machine import UART

lcd.init()
mic.init()

uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

def get_mic_dir():
    """
    返回[x, y, r, angle_h, angle_v]：
    x,y 笛卡尔分量
    r: 归一化强度
    angle_h: 水平角度 0~359, 0为正前，顺时针
    angle_v: 俯仰角（没有3D麦克风可固定为90）
    """
    imga = mic.get_map()
    b = mic.get_dir(imga)
    mic.set_led(b, (0, 0, 255))
    AngleX = 0
    AngleY = 0
    for i in range(len(b)):
        if b[i] >= 2:
            AngleX += b[i] * math.sin(i * math.pi / 6)
            AngleY += b[i] * math.cos(i * math.pi / 6)
    AngleX = round(AngleX, 6)
    AngleY = round(AngleY, 6)
    Angle_h = None
    AngleAddPi = 0
    if AngleY < 0:
        AngleAddPi = 180
    if AngleX < 0 and AngleY > 0:
        AngleAddPi = 360
    if AngleX != 0 or AngleY != 0:
        if AngleY == 0:
            Angle_h = 90 if AngleX > 0 else 270
        else:
            Angle_h = AngleAddPi + round(math.degrees(math.atan(AngleX / AngleY)), 4)
        AngleR = round(math.sqrt(AngleY * AngleY + AngleX * AngleX), 4)
        # 俯仰角：二维麦克风无法获取，云台Y轴可固定90°
        Angle_v = 90
        return [AngleX, AngleY, AngleR, int(Angle_h) % 360, int(Angle_v)]
    else:
        return None

while True:
    mic_info = get_mic_dir()
    if mic_info:
        AngleX, AngleY, AngleR, Angle_h, Angle_v = mic_info
        out_str = "X:{},Y:{},STRENGTH:{}\r\n".format(Angle_h, Angle_v, int(AngleR))
        uart.write(out_str)
    time.sleep_ms(100)
