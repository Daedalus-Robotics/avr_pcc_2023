#include "esp32_serial_transport.hpp"

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>

#define UART_TXD  (CONFIG_MICROROS_UART_TXD)
#define UART_RXD  (CONFIG_MICROROS_UART_RXD)
#define UART_RTS  (CONFIG_MICROROS_UART_RTS)
#define UART_CTS  (CONFIG_MICROROS_UART_CTS)

#define UART_BUFFER_SIZE (512)

bool esp32SerialOpen(uxrCustomTransport *transport)
{
    auto *uart_port = (uart_port_t *) transport->args;

    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL)
    {
        return false;
    }
    if (uart_set_pin(*uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) == ESP_FAIL)
    {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, nullptr, 0) == ESP_FAIL)
    {
        return false;
    }

    return true;
}

bool esp32SerialClose(uxrCustomTransport *transport)
{
    auto *uart_port = (uart_port_t *) transport->args;

    return uart_driver_delete(*uart_port) == ESP_OK;
}

size_t esp32SerialWrite(uxrCustomTransport *transport,
                        const uint8_t *buf, size_t len, __attribute((unused)) uint8_t *err)
{
    auto *uart_port = (uart_port_t *) transport->args;
    const int tx_bytes = uart_write_bytes(*uart_port, (const char *) buf, len);
    return tx_bytes;
}

size_t esp32SerialRead(uxrCustomTransport *transport,
                       uint8_t *buf, size_t len, int timeout, __attribute((unused)) uint8_t *err)
{
    auto *uart_port = (uart_port_t *) transport->args;
    const int rx_bytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rx_bytes;
}
