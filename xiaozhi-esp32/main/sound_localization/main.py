# 导入数学计算库，用于三角函数和角度计算
import math
# 导入时间库，用于延时控制
import time
# 导入MIC_ARRAY麦克风阵列模块，用于声音方向检测
from Maix import MIC_ARRAY as mic
# 导入LCD显示模块
import lcd
# 导入UART串口通信模块
from machine import UART

# 初始化LCD显示屏
lcd.init()
# 初始化麦克风阵列
mic.init()

# 初始化UART1串口，波特率115200，8位数据位，0位奇偶校验位，1位停止位，超时1秒，读缓冲区4KB
uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

def get_mic_dir():
    """
    获取声源方向信息的函数
    返回值：包含声源坐标和角度信息的列表 [AngleX, AngleY, AngleR, Angle]
    """
    mic_list = []  # 初始化返回列表
    
    # 获取麦克风阵列的声音强度分布图
    imga = mic.get_map()
    # 根据声音强度分布计算各个方向的声音强度数组（12个方向，每30度一个）
    b = mic.get_dir(imga)
    # 设置LED灯显示声源方向，颜色为蓝色(0,0,255)
    mic.set_led(b, (0,0,255))
    
    # 初始化笛卡尔坐标系中的X、Y分量
    AngleX = 0  # X轴方向的加权和
    AngleY = 0  # Y轴方向的加权和
    AngleAddPi = 0  # 角度修正值，用于处理不同象限
    
    # 遍历12个方向的声音强度值
    for i in range(len(b)):
        if b[i] >= 2:  # 只处理强度大于等于2的信号，过滤噪声
            # 将极坐标转换为直角坐标，i*π/6表示每个方向间隔30度
            AngleX += b[i] * math.sin(i * math.pi / 6)  # X分量累加
            AngleY += b[i] * math.cos(i * math.pi / 6)  # Y分量累加
    
    # 对坐标值进行精度处理，保留6位小数
    AngleX = round(AngleX, 6)
    AngleY = round(AngleY, 6)
    
    # 根据象限确定角度修正值
    if AngleY < 0: AngleAddPi = 180      # 第三、四象限，加180度
    if AngleX < 0 and AngleY > 0: AngleAddPi = 360  # 第二象限，加360度
    
    # 如果检测到有效声源（不在原点）
    if AngleX != 0 or AngleY != 0:
        if AngleY == 0:  # 特殊情况：声源在X轴上
            Angle = 90 if AngleX > 0 else 270  # 正X轴为90度，负X轴为270度
        else:
            # 一般情况：使用反正切函数计算角度，并加上象限修正
            Angle = AngleAddPi + round(math.degrees(math.atan(AngleX / AngleY)), 4)
        
        # 计算声源强度（距离原点的距离）
        AngleR = round(math.sqrt(AngleY * AngleY + AngleX * AngleX), 4)
        # 封装返回数据：[X坐标, Y坐标, 强度值, 角度]
        mic_list = [AngleX, AngleY, AngleR, Angle]
    
    return mic_list  # 返回声源信息列表

# 主循环：持续检测声源并通过串口发送数据
while True:
    # 调用声源检测函数
    mic_info = get_mic_dir()
    
    if mic_info:  # 只在检测到有效声源时发送数据
        # 解包声源信息
        AngleX, AngleY, AngleR, Angle = mic_info
        
        # 格式化输出字符串，将角度和强度转换为整数并打包
        # 格式："ANGLE:角度值,STRENGTH:强度值\r\n"
        out_str = "ANGLE:{},STRENGTH:{}\r\n".format(int(Angle), int(AngleR))
        
        # 通过UART串口发送声源信息
        uart.write(out_str)
    
    # LCD显示部分代码省略
    # 延时100毫秒，控制检测频率为10Hz
    time.sleep_ms(100)