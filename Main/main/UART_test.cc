// ESP-IDF 示例
#define TX_PIN 17
#define RX_PIN 18

void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);

    while (1) {
        uart_write_bytes(uart_num, "Hello K210!\n", 12);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

