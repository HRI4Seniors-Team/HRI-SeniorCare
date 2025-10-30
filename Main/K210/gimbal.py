# 云台控制模块
# 负责协调俯仰轴和横滚轴的控制

#云台控制
class Gimbal:
    def __init__(self, pitch, pid_pitch, roll=None, pid_roll=None):
        self._pitch = pitch
        self._roll = roll
        self._pid_pitch = pid_pitch # 俯仰轴PID控制器
        self._pid_roll = pid_roll   # 横滚轴PID控制器

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