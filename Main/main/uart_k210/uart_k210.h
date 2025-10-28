#ifndef UART_K210_H
#define UART_K210_H

#include <driver/uart.h>

class UartK210 {
public:
    void Init();
    void SendData(const char* data, size_t len);
    int ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms);
    void StartReceiveTask();  // 持续接收数据的任务
    
private:
    static constexpr uart_port_t UART_NUM_ = UART_NUM_1;
    static constexpr int TX_PIN = 17;
    static constexpr int RX_PIN = 18;
    static constexpr int BAUD_RATE = 115200;
    static constexpr int BUF_SIZE = 1024;
};

#endif