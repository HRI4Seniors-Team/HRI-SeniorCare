#include "uart_k210.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "UART_K210(ESP32)"

void UartK210::Init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_, TX_PIN, RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_, BUF_SIZE, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", 
             TX_PIN, RX_PIN, BAUD_RATE);
}

void UartK210::SendData(const char* data, size_t len) {
    uart_write_bytes(UART_NUM_, data, len);
    ESP_LOGI(TAG, "Sent to K210: %.*s (LOG)", len, data);
}

int UartK210::ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_NUM_, buffer, max_len, 
                          pdMS_TO_TICKS(timeout_ms));
}

void UartK210::StartReceiveTask() {
    xTaskCreate([](void* param) {
        UartK210* uart = static_cast<UartK210*>(param);
        uint8_t buffer[BUF_SIZE];
        size_t index = 0;
        
        while (1) {
            uint8_t byte;
            int len = uart->ReceiveData(&byte, 1, 100);  // 每次读 1 字节
            
            if (len > 0) {
                if (byte == '\n') {
                    // 收到完整一行
                    buffer[index] = '\0';
                    ESP_LOGI(TAG, "Received line: %s", buffer);
                    index = 0;  // 重置缓冲区
                } else if (index < BUF_SIZE - 1) {
                    buffer[index++] = byte;
                } else {
                    // 缓冲区满，丢弃
                    ESP_LOGW(TAG, "Buffer overflow, resetting");
                    index = 0;
                }
            }
        }
    }, "uart_rx_task", 4096, this, 5, NULL);
}