from machine import PWM

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

    def set_position(self, position):
        """设置绝对位置"""
        self.value = self._clamp(position)
        duty = self.value/100*self.duty_range + self.duty_min
        self.pwm.duty(duty)
        return self.value

    def get_position(self):
        """获取当前位置"""
        return self.value

    def turn_left(self, amount=5):
        """向左转指定角度"""
        if amount < 0:
            amount = abs(amount)  # 确保amount为正数
        return self.drive(-amount)

    def turn_right(self, amount=5):
        """向右转指定角度"""
        if amount < 0:
            amount = abs(amount)  # 确保amount为正数
        return self.drive(amount)

    def get_info(self):
        """获取舵机状态信息"""
        return {
            "current_position": self.value,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "is_roll": self.is_roll,
            "duty_min": self.duty_min,
            "duty_max": self.duty_max
        }