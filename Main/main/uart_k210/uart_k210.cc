#include "uart_k210.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "UART_K210"

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
}

int UartK210::ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_NUM_, buffer, max_len, 
                          pdMS_TO_TICKS(timeout_ms));
}

void UartK210::StartReceiveTask() {
    xTaskCreate([](void* param) {
        UartK210* uart = static_cast<UartK210*>(param);
        uint8_t buffer[BUF_SIZE];
        
        while (1) {
            int len = uart->ReceiveData(buffer, BUF_SIZE - 1, 100);
            if (len > 0) {
                buffer[len] = '\0';
                ESP_LOGI(TAG, "Received from K210: %s", buffer);
                
                // 在这里处理接收到的数据
                // 例如:解析命令、触发事件等
            }
        }
    }, "uart_rx_task", 4096, this, 5, NULL);
}