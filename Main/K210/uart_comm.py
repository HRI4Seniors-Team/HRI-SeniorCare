from machine import UART
import time
from board import board_info
from fpioa_manager import fm
from Maix import GPIO

class UartComm:
    def __init__(self):
        # 初始化 UART1, 波特率 115200
        # K210: IO4=RX(接ESP32的TX/GPIO17), IO5=TX(接ESP32的RX/GPIO18)

        # try:
        #     fm.unregister(fm.fpioa.UART1_RX)
        # except ValueError:
        #     pass
        # try:
        #     fm.unregister(fm.fpioa.UART1_TX)
        # except ValueError:
        #     pass

        # 先释放原来的映射，避免和 REPL 冲突
        for func in (fm.fpioa.UARTHS_RX, fm.fpioa.UARTHS_TX):
            try:
                fm.unregister(func)
            except ValueError:
                pass

        try:
            # fm.register(13, fm.fpioa.UART1_RX, force=True)  # IO4 ← ESP32 TX
            # fm.register(12, fm.fpioa.UART1_TX, force=True)  # IO5 → ESP32 RX

            fm.register(board_info.PIN4, fm.fpioa.UARTHS_RX, force=True)  # IO4 ← ESP32 TX
            fm.register(board_info.PIN5, fm.fpioa.UARTHS_TX, force=True)  # IO5 → ESP32 RX

            print("(k210) UART pins registered: RX=IO4, TX=IO5")
        except:
            print("(k210) Failed to register UART1 pins")
            pass

        self.uart = UART(UART.UARTHS, 115200, read_buf_len=4096)
        print("(k210) UART initialized: 115200 baud")

    # 清空缓冲区
        if self.uart.any():
            self.uart.read()
            print("(k210) Cleared UART buffer")

    def send(self, data):
        """发送数据到 ESP32"""
        if isinstance(data, str):
            data = data.encode('utf-8')
        self.uart.write(data)
        # print(f"Sent to ESP32: {data}")
        print("Sent to ESP32: {}".format(data))
    
    def receive(self, timeout_ms=100):
        """接收 ESP32 发来的数据"""
        if self.uart.any():
            data = self.uart.read()
            if data:
                try:
                    return data.decode('utf-8')
                except:
                    return data  # 返回原始字节
        return None
    
    def receive_line(self, timeout_ms=1000):
        """接收一行数据(以 \n 结尾)"""
        start = time.ticks_ms()
        buffer = b''
        
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                char = self.uart.read(1)
                print('(k210) Received char: {}'.format(char))
                if char:
                    buffer += char
                    if char == b'\n':
                        try:
                            return buffer.decode('utf-8').strip()
                        except:
                            return buffer
            time.sleep_ms(10)
        
        # 超时检查
        if buffer:
            result = buffer.decode('utf-8').strip() if buffer else None
            print("(k210) Timeout with partial data: [{}]".format(result))
            return result
        print("(k210) Receive line timeout with no data")
        return None
    
    def start_receive_task(self, callback):
        """持续接收数据并调用回调函数处理
        
        Args:
            callback: 回调函数,接收一个参数(接收到的数据)
        """
        while True:
            data = self.receive_line()
            if data:
                # print(f"Received from ESP32: {data}")
                print("(k210) Received from ESP32: {}".format(data))
                callback(data)
            time.sleep_ms(10)
