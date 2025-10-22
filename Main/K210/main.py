import time, sys, math
# 引入定时器和PWM模块
from machine import Timer,PWM

from math import pi
from machine import UART
from Maix import MIC_ARRAY as mic

import lcd, image

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

# 舵机控制类
# 输入参数：
#   pwm: PWM对象
#   dir: 初始位置，默认50（范围0-100）
#   duty_min: 最小占空比，默认2.5%
#   duty_max: 最大占空比，默认12.5%
#   range_min: 最小控制范围，默认0
#   range_max: 最大控制范围，默认100
#   is_roll: 是否为横滚轴控制，默认False
# 输出：
#   drive(inc)：根据增量inc调整舵机位置
#   enable(en)：启用或禁用舵机
class Servo:
    def __init__(self, pwm, dir=50, duty_min=2.5, duty_max=12.5, range_min=0, range_max=100, is_roll=False):
        self.range_min = range_min
        self.range_max = range_max
        self.is_roll = is_roll
        self.value = self._clamp(dir)
        self.pwm = pwm
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_range = duty_max-duty_min
        self.enable(True)
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)

    def _clamp(self, value):
        if self.is_roll:
            if value > self.range_max:
                return self.range_max  # 触及180°边界
            elif value < self.range_min:
                return self.range_min
        return max(self.range_min, min(self.range_max, value))

    def enable(self,en):
        if en:
            self.pwm.enable()
        else:
            self.pwm.disable()


    def drive(self,inc):
        if self.is_roll:
            next_value = self.value + inc

            if next_value > self.range_max:
                inc = -(next_value - self.range_max)
            elif next_value < self.range_min:
                inc = -(next_value - self.range_min)

        self.value += inc
        self.value = self._clamp(self.value)

        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)


#PID算法
class PID:
    _kp = _ki = _kd = integrator = _imax = 0
    _last_error = _last_t = 0
    _RC = 1/(2 * pi *20)
    def __init__(self, p=0, i=0, d=0, imax=0, roll_range=(-180,180), is_roll=False):
        self._kp = float(p) # 比例系数
        self._ki = float(i) # 积分系数
        self._kd = float(d) # 微分系数
        self._imax = abs(imax)  #积分限幅值
        self._last_drivative = None # 上一次的微分值
        self.roll_min, self.roll_max = roll_range
        self.is_roll = is_roll

    def get_pid(self, error, scaler):   # error：当前误差，scaler：输出缩放因子（用于调整控制量范围）

        if self.is_roll:
            total_range = self.roll_max - self.roll_min

            if abs(error) > total_range / 2:
                error = error - total_range if error > 0 else error + total_range

        tnow = time.ticks_ms()
        dt = tnow - self._last_t
        output = 0

        #时间处理与异常检测
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()  # 超时或首次调用时重置积分
        self._last_t = tnow
        delta_time = float(dt) / float(1000)    # 转换为秒

        # 比例项（P）计算
        output += error * self._kp

        #微分项（D）计算
        if abs(self._kd) > 0 and dt > 0:
            if self._last_derivative == None:
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            # 低通滤波（抑制高频噪声）
            derivative = self._last_derivative +\
                                    ((delta_time / (self._RC + delta_time))*\
                                        (derivative - self._last_derivative))
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler

        # 积分项（I）计算
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
            output += self._integrator

        return output

        # 重置积分项与微分历史状态
    def reset_I(self):
        self._integrator = 0
        self._last_derivative = None

#云台控制
class Gimbal:
    def __init__(self, pitch, pid_pitch, roll=None, pid_roll=None):
        self._pitch = pitch # 俯仰轴执行器（如舵机）
        self._roll = roll   # 横滚轴执行器（可选）
        self._pid_pitch = pid_pitch # 俯仰轴PID控制器
        self._pid_roll = pid_roll   # 横滚轴PID控制器（可选）

    # 预留接口，用于直接控制舵机
    def set_out(self, pitch, roll, yaw=None):
        pass

    def run(self, height_err, horizontal_err=50, pitch_reverse=False, roll_reverse=False):
    # 俯仰轴控制
        out = self._pid_pitch.get_pid(height_err, 1)
        #print("err:{}, out:{}".format(pitch_err, out))
        out = max(-3, min(3, out))
        if pitch_reverse:
            out = - out
        self._pitch.drive(out)

    # 横滚轴控制（如果存在）
        if self._roll and self._pid_roll:
            roll_out = self._pid_roll.get_pid(horizontal_err, 1)

            roll_out = max(-5, min(5, out))
            if roll_reverse:
                roll_out = - roll_out
            self._roll.drive(roll_out)

_img_buf = None
def lcd_setup(rotation=0):
    try:
        lcd.init()
        lcd.rotation(rotation)
        lcd.clear(lcd.RED)
    except:
        pass

# 在 LCD 上显示相对角度信息
# pitch_val/roll_val：当前 Servo.value（0~100）
# init_pitch/init_roll：初始位置（可用 config 里的 init_*）
# *_scale：单位换算，默认 1.8°/单位（0~100 → 0~180°）
# *_invert：方向反转（机构/安装导致方向相反时设 True）
# 说明：本函数不修改任何舵机，只负责画图
def lcd_show_gimbal_delta(pitch_val, roll_val,
                          init_pitch=50, init_roll=50,
                          pitch_scale=1.8, roll_scale=1.8,
                          pitch_invert=False, roll_invert=False):
    global _img_buf
    W, H = 320, 240
    if _img_buf is None:
        _img_buf = image.Image(size=(W, H))

    # 计算“相对初始”的角度（度）
    d_pitch = (pitch_val - init_pitch) * pitch_scale
    d_roll  = (roll_val  - init_roll)  * roll_scale

    if pitch_invert: d_pitch = -d_pitch
    if roll_invert:  d_roll  = -d_roll

    # 方向文案
    def sign_text(v, up_txt, down_txt):
        if v > 0:  return up_txt,  abs(v)
        if v < 0:  return down_txt, abs(v)
        return "No change", 0.0

    pitch_dir, pitch_abs = sign_text(d_pitch, "down", "up")
    roll_dir,  roll_abs  = sign_text(d_roll,  "left", "right")

    # 画面
    _img_buf.clear()
    _img_buf.draw_string(8, 8,  "Gimbal status", color=(255,255,255), scale=2)

    # 俯仰
    _img_buf.draw_string(8, 50, "Pitch:", color=(0,255,255), scale=2)
    _img_buf.draw_string(8, 80, "%s  %5.1f°" % (pitch_dir, pitch_abs),
                         color=(0,255,0) if pitch_dir!="No change" else (200,200,200), scale=2)

    # 水平
    _img_buf.draw_string(8, 130,"Roll:", color=(0,255,255), scale=2)
    _img_buf.draw_string(8, 160,"%s  %5.1f°" % (roll_dir, roll_abs),
                         color=(0,200,255) if roll_dir!="No change" else (200,200,200), scale=2)

    # 简单指示条（±90°范围可视化）
    # pitch
    bar_x, bar_y, bar_w, bar_h = 8, 200, 300, 12
    _img_buf.draw_rectangle(bar_x-2, bar_y-2, bar_w+4, bar_h+4, color=(80,80,160), thickness=2)
    p = max(-90, min(90, d_pitch))
    p_w = int((p + 90) / 180.0 * bar_w)
    _img_buf.draw_rectangle(bar_x, bar_y, p_w, bar_h, color=(0,255,0), thickness=-1)
    _img_buf.draw_string(bar_x+bar_w+6, bar_y-4, "%+3.0f" % p, color=(0,255,0), scale=2)

    lcd.display(_img_buf)


def main():

    tim_pitch = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
    tim_roll = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
    pwm_pitch = PWM(tim_pitch, freq=50, duty=0, pin=7)
    pwm_roll = PWM(tim_roll, freq=50, duty=0, pin=8)

    
    config = {
        "init_pitch": 50,
        "init_roll": 50,

        "pitch_pid": [0.5, 0.02, 0.03, 5],
        "roll_pid": [0.5, 0.02, 0.03, 10],

        "pitch_reverse": False,
        "roll_reverse": True,

        # "audio_range": 15,
        "audio_range": 10,
        # "ignore_threshold": 0.005,
        "ignore_threshold": 0.1,

        "roll_range": (10, 90)
       }

    detector = AudioTargetDetector(
        out_range = config["audio_range"],
        ignore_limit=config["ignore_threshold"]
    )

    pid_pitch = PID(
        *config["pitch_pid"],
        is_roll=False
    )
    pid_roll = PID(
        *config["roll_pid"],
        is_roll=True
    )

    pitch_servo = Servo(
        pwm_pitch,
        dir=config["init_pitch"],
        range_min=0,
        range_max=100,
        is_roll=False
    )
    roll_servo = Servo(
        pwm_roll,
        dir=config["init_roll"],
        range_min=0,
        range_max=100,
        is_roll=True
    )

    gimbal = Gimbal(
        pitch=pitch_servo,
        pid_pitch=pid_pitch,
        roll=roll_servo,
        pid_roll=pid_roll
    )
    start_time = time.time()

    lcd_setup(rotation=0)
    # 记录舵机初始位置
    init_pitch = config["init_pitch"]
    init_roll  = config["init_roll"]
    
    pitch_disp_invert = False
    roll_disp_invert  = False
    
    try:
        while True:

            height_err, horizontal_err = detector.get_target_err()

            gimbal.run(
                height_err = height_err,
                horizontal_err = horizontal_err,
                pitch_reverse = config["pitch_reverse"],
                roll_reverse = config["roll_reverse"]
            )
            
            lcd_show_gimbal_delta(
                pitch_val=pitch_servo.value,
                roll_val=roll_servo.value,
                init_pitch=init_pitch,
                init_roll=init_roll,
                pitch_scale=1.8,     # 0~100 → 0~180° 的默认比例
                roll_scale=1.8,
                pitch_invert=pitch_disp_invert,
                roll_invert=roll_disp_invert
            )
            
            time.sleep_ms(10)

            if time.time() - start_time > 120:
                print("程序超时终止")
                break
    except Exception as e:
        print("程序异常: {}".format(e))
    finally:
        detector.deinit()
        pitch_servo.drive(config["init_pitch"])
        roll_servo.drive(config["init_roll"])
        pitch_servo.enable(False)
        roll_servo.enable(False)

#主程序入口
if __name__=="__main__":
    main()
