from machine import UART

uart = UART(UART.UART1, 115200, read_buf_len=4096)  # IO4=RX, IO5=TX
