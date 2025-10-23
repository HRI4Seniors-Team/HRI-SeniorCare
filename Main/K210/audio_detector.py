from machine import UART
from math import pi
from Maix import MIC_ARRAY as mic
import math

# 音频目标检测类
# 输入参数：
#   out_range: 输出误差范围，默认为10
#   ignore_limit: 忽略微小误差的阈值比例，默认为0.02（即2%）
# 输出：
#   get_target_err()：返回声源高度误差和水平误差的元组（height_err, horizontal_err）
class AudioTargetDetector:
    def __init__(self, out_range=10, ignore_limit=0.02):
        self.out_range = out_range
        self.ignore_limit = ignore_limit
        self.__init_microphone()

        self.__turn_off_all_leds()

    def __init_microphone(self):
        mic.init()
        #mic.init(
            #i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20,
            #i2s_ws=19, i2s_sclk=18, sk9822_dat=10, sk9822_clk=9)
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

        self.uart =UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

    def __turn_off_all_leds(self):
        mic.set_led([1] * 12, (0,0,0))

    def get_target_err(self):
        """返回值为（pitch_err,roll_err, high_err）"""

        #1. 计算声源高度误差（垂直方向，控制Pitch轴）
        mic_data = mic.get_map()
        direction = mic.get_dir(mic_data)
        print("Direction data: {}".format(direction))
        mic.set_led(direction, (0, 255, 0))

        #计算声源向量
        vertical_strength = direction[6]
        horizontal_avg = sum(direction[0:6])/6

        height_err = (vertical_strength - horizontal_avg)/max(horizontal_avg, 1e-6) * self.out_range

        height_err = max(-self.out_range, min(self.out_range, height_err))

        # # 常见偏移值：
        # 0: 第0个麦克风已经指向正前方
        # math.pi/6: 需要顺时针旋转30°
        # -math.pi/6: 需要逆时针旋转30°
        # math.pi/3: 需要顺时针旋转60°
        angle_offset = 0

        #2. 计算水平平面误差（控制Roll轴360°旋转）
        AngleX, AngleY = 0, 0
        for i in range(6):
            angle_rad = i * math.pi / 3 + angle_offset
            AngleX += direction[i] * math.sin(angle_rad)
            AngleY += direction[i] * math.cos(angle_rad)

        horizontal_err = math.atan2(AngleX,AngleY) / (math.pi) * self.out_range


        #忽略微小误差
        if abs(height_err) < self.ignore_limit * self.out_range:
            height_err = 0
        if abs(horizontal_err) < self.ignore_limit * self.out_range:
            horizontal_err = 0

        print("高度误差:{},水平误差:{}".format(height_err, horizontal_err))
        return height_err, horizontal_err

    def deinit(self):
        mic.deinit()