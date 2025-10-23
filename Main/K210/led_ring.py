import lcd, image

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
    _img_buf.draw_string(8, 80, "%s  %5.1f degrees" % (pitch_dir, pitch_abs),
                         color=(0,255,0) if pitch_dir!="No change" else (200,200,200), scale=2)

    # 水平
    _img_buf.draw_string(8, 130,"Roll:", color=(0,255,255), scale=2)
    _img_buf.draw_string(8, 160,"%s  %5.1f degrees" % (roll_dir, roll_abs),
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
