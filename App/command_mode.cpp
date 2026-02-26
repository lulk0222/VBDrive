#include "app.h"

static constexpr uint16_t UART_RX_BUFFER_SIZE = 32;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE + 1];


void start_uart_recv_it() {
    std::memset(uart_rx_buffer, '\0', UART_RX_BUFFER_SIZE + 1);
    /*
    / If TX uses DMA, next line is not needed,
    / if TX is IT or direct, next line is required due to bug in HAL.
    / Kept here so I don't forget
    HAL_UART_Abort_IT(&huart2);
    */
    auto status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2,
        reinterpret_cast<uint8_t*>(uart_rx_buffer),
        UART_RX_BUFFER_SIZE
    );
    if (status != HAL_OK) {
        // TODO
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    auto& app_config = get_app_config();

    uart_rx_buffer[size] = '\0';
    auto command_string = std::string(uart_rx_buffer);

    app_config.process_command(command_string);

    start_uart_recv_it();
}
