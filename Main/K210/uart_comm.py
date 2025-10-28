from machine import UART
import time

uart = UART(UART.UART1, 115200, read_buf_len=4096)  # IO4=RX, IO5=TX

class UartComm:
    def __init__(self):
        # 初始化 UART1, 波特率 115200
        # K210: IO4=RX(接ESP32的TX/GPIO17), IO5=TX(接ESP32的RX/GPIO18)
        self.uart = UART(UART.UART1, 115200, read_buf_len=4096)
        print("UART initialized: 115200 baud")
    
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
                if char:
                    buffer += char
                    if char == b'\n':
                        try:
                            return buffer.decode('utf-8').strip()
                        except:
                            return buffer
            time.sleep_ms(10)
        
        return buffer.decode('utf-8').strip() if buffer else None
    
    def start_receive_task(self, callback):
        """持续接收数据并调用回调函数处理
        
        Args:
            callback: 回调函数,接收一个参数(接收到的数据)
        """
        while True:
            data = self.receive_line()
            if data:
                # print(f"Received from ESP32: {data}")
                print("Received from ESP32: {}".format(data))
                callback(data)
            time.sleep_ms(10)
