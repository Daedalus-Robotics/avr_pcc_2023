#include <uxr/client/transport.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#ifndef AVR_PCC_2023_ESP32_SERIAL_TRANSPORT_H
#define AVR_PCC_2023_ESP32_SERIAL_TRANSPORT_H

#ifdef __cplusplus
extern "C"
{
#endif

bool esp32_serial_open(uxrCustomTransport *transport);

bool esp32_serial_close(uxrCustomTransport *transport);

size_t esp32_serial_write(uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

size_t esp32_serial_read(uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
};
#endif

#endif //AVR_PCC_2023_ESP32_SERIAL_TRANSPORT_H