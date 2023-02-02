#pragma once

#include <Arduino.h>
#include "driver/uart.h"
#include "drivers/serial.h"

typedef struct {
    uart_port_t uart_num;
    int rxPin;
    int txPin; 
    int baud_rate;
    serial_port_t *serial_port;
} uartPort_t;

void uartInit(uartPort_t *uartPort);
