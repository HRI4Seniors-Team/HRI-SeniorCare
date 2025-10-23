import time, sys, math
# 引入定时器和PWM模块
from machine import Timer, PWM

# 从模块导入所需的类和函数
from audio_detector import AudioTargetDetector
from servo import Servo
from doa_calc import PID
from gimbal import Gimbal
from led_ring import lcd_setup, lcd_show_gimbal_delta
from uart_comm import uart
from config import load_config_from_json

def main():
    """主程序函数"""
    
    # 初始化PWM定时器
    tim_pitch = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
    tim_roll = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
    pwm_pitch = PWM(tim_pitch, freq=50, duty=0, pin=7)
    pwm_roll = PWM(tim_roll, freq=50, duty=0, pin=8)

    # 加载配置 - 从 config.py 获取
    config = load_config_from_json()

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
            
            data = uart.read()
            if data:
                print("Received:", data)
                uart.write(b"ACK\n")
            time.sleep(0.1)
            
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
