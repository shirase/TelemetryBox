#include "uart.h"

#define UART_BUF_SIZE (1024)

bool uartReadByte(uint8_t *port, uint8_t *byte)
{
    uartPort_t *uartPort = (uartPort_t*)port;
    return (bool)uart_read_bytes(uartPort->uart_num, byte, 1, 10 / portTICK_PERIOD_MS);
}

bool uartWriteByte(uint8_t *port, uint8_t byte)
{
    uartPort_t *uartPort = (uartPort_t*)port;
    return (bool)uart_write_bytes(uartPort->uart_num, (char*)&byte, 1);
}

uint16_t uartReadBuf(uint8_t *port, uint8_t *buf, uint16_t len)
{
    uartPort_t *uartPort = (uartPort_t*)port;
    return uart_read_bytes(uartPort->uart_num, buf, len, 10 / portTICK_PERIOD_MS);
}

uint16_t uartWriteBuf(uint8_t *port, uint8_t *buf, uint16_t len)
{
    uartPort_t *uartPort = (uartPort_t*)port;
    return uart_write_bytes(uartPort->uart_num, (char*)buf, len);
}

void uartInit(uartPort_t *uartPort)
{
    uart_config_t uart_config = {
        .baud_rate = uartPort->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(uartPort->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uartPort->uart_num, uartPort->txPin, uartPort->rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uartPort->uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));

    uartPort->serial_port->port = (uint8_t*)uartPort;
    uartPort->serial_port->readByte = uartReadByte;
    uartPort->serial_port->writeByte = uartWriteByte;
    uartPort->serial_port->readBuf = uartReadBuf;
    uartPort->serial_port->writeBuf = uartWriteBuf;
}
