import time, sys, math
from machine import Timer,PWM

from math import pi
from machine import UART
from Maix import MIC_ARRAY as mic


class AudioTargetDetector:
    def __init__(self, out_range=10, ignore_limit=0.02):
        self.out_range = out_range
        self.ignore_limit = ignore_limit
        self.__init_microphone()

    def __init_microphone(self):
        mic.init()
        mic.init(
            i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20,
            i2s_ws=19, i2s_sclk=18, sk9822_dat=10, sk9822_clk=9)
        self.uart =UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

    def get_target_err(self):
        """返回值为（pitch_err,roll_err）"""
        mic_data = mic.get_map()
        direction = mic.get_dir(mic_data)
        print("Direction data: {}".format(direction))
        mic.set_led(direction, (57, 197, 187))

        #计算声源向量
        AngleX, AngleY = 0, 0
        for i, strength in enumerate(direction):
            AngleX += strength * math.sin(i * math.pi / 6)
            AngleY += strength * math.cos(i * math.pi / 6)

        #转换为误差信号
        pitch_err = AngleX / max(abs(AngleX), 1e-6) * self.out_range
        roll_err = AngleY / max(abs(AngleY), 1e-6) * self.out_range

        #忽略微小误差
        if abs(pitch_err) < self.ignore_limit * self.out_range:
            pitch_err = 0
        if abs(roll_err) < self.ignore_limit * self.out_range:
            roll_err = 0

        print("pitch_err:{},roll_err:{}".format(pitch_err, roll_err))
        return pitch_err, roll_err

    def deinit(self):
        mic.deinit()


class Servo:
    def __init__(self, pwm, dir=50, duty_min=2.5, duty_max=12.5, range_min=0, range_max=100):
        self.range_min = range_min
        self.range_max = range_max
        self.value = self._clamp(dir)
        self.pwm = pwm
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_range = duty_max-duty_min
        self.enable(True)
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)

    def _clamp(self, value):
        return max(self.range_min, min(self.range_max, value))

    def enable(self,en):
        if en:
            self.pwm.enable()
        else:
            self.pwm.disable()

    def dir(self , percentage):
        percentage = self._clamp(percentage)
        if percentage > 100:
            percentage = 100
        elif percentage < 0:
            percentage = 0
        self.pwm.duty(percentage/100*self.duty_range+self.duty_min)
        self.value = percentage

    def drive(self,inc):
        self.value += inc
        self.value = self._clamp(self.value)
        if self.value > 100:
            self.value =100
        elif self.value < 0:
            self.value = 0
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)


#PID算法
class PID:
    _kp = _ki = _kd = integrator = _imax = 0
    _last_error = _last_t = 0
    _RC = 1/(2 * pi *20)
    def __init__(self, p=0, i=0, d=0, imax=0):
        self._kp = float(p) # 比例系数
        self._ki = float(i) # 积分系数
        self._kd = float(d) # 微分系数
        self._imax = abs(imax)  #积分限幅值
        self._last_drivative = None # 上一次的微分值

    def get_pid(self, error, scaler):   # error：当前误差，scaler：输出缩放因子（用于调整控制量范围）
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
    def __init__(self, pitch, pid_pitch, roll=None, pid_roll=None, yaw=None, pid_yaw=None):
        self._pitch = pitch # 俯仰轴执行器（如舵机）
        self._roll = roll   # 横滚轴执行器（可选）
        self._yaw = yaw     # 偏航轴执行器（可选）
        self._pid_pitch = pid_pitch # 俯仰轴PID控制器
        self._pid_roll = pid_roll   # 横滚轴PID控制器（可选）
        self._pid_yaw = pid_yaw     # 偏航轴PID控制器（可选）

    # 预留接口，用于直接控制舵机
    def set_out(self, pitch, roll, yaw=None):
        pass

    def run(self, pitch_err, roll_err=50, yaw_err=50, pitch_reverse=False, roll_reverse=False, yaw_reverse=False):
    # 俯仰轴控制
        out = self._pid_pitch.get_pid(pitch_err, 1)
        #print("err:{}, out:{}".format(pitch_err, out))
        out = max(-3, min(3, out))
        if pitch_reverse:
            out = - out
        self._pitch.drive(out)

    # 横滚轴控制（如果存在）
        if self._roll:
            out = self._pid_roll.get_pid(roll_err, 1)
            out = max(-5, min(5, out))
            if roll_reverse:
                out = - out
            self._roll.drive(out)

    # 偏航轴控制（如果存在）
        if self._yaw:
            out = self._pid_yaw.get_pid(yaw_err, 1)
            if yaw_reverse:
                out = - out
            self._yaw.drive(out)


def main():

    tim_pitch = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
    tim_roll = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
    pwm_pitch = PWM(tim_pitch, freq=50, duty=0, pin=7)
    pwm_roll = PWM(tim_roll, freq=50, duty=0, pin=8)

    config = {
        "init_pitch": 50,
        "init_roll": 50,

        "pitch_pid": [0.3, 0, 0.02, 0],
        "roll_pid": [0.25, 0, 0.015, 0],

        "pitch_reverse": False,
        "roll_reverse": True,

        "audio_range": 10,
        "ignore_threshold": 0.02
       }

    detector = AudioTargetDetector(
        out_range = config["audio_range"],
        ignore_limit=config["ignore_threshold"]
    )

    pid_pitch = PID(*config["pitch_pid"])
    pid_roll = PID(*config["roll_pid"])

    pitch_servo = Servo(
        pwm_pitch,
        dir=config["init_pitch"],
        range_min=0,
        range_max=100
    )
    roll_servo = Servo(
        pwm_roll,
        dir=config["init_roll"],
        range_min=0,
        range_max=100
    )

    gimbal = Gimbal(
        pitch=pitch_servo,
        pid_pitch=pid_pitch,
        roll=roll_servo,
        pid_roll=pid_roll
    )
    start_time = time.time()
    try:
        while True:

            pitch_err, roll_err = detector.get_target_err()

            gimbal.run(
                pitch_err = pitch_err,
                roll_err = roll_err,
                pitch_reverse = config["pitch_reverse"],
                roll_reverse = config["roll_reverse"]
            )
            time.sleep_ms(10)

            if time.time() - start_time > 30:
                print("程序超时终止")
                break
    except Exception as e:
        print("程序异常: {}".format(e))
    finally:
        detector.deinit()
        pitch_servo.drive(config["init_pitch"])
        roll_servo.drive(config["init_roll"])

#主程序入口
if __name__=="__main__":
    main()



