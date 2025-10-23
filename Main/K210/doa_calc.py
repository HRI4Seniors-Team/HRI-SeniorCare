from math import pi
import time

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