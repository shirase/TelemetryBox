#pragma once

#include "driver/uart.h"

#define USE_BLUETOOTH

#define CRSF_UART UART_NUM_2
#define CRSF_UART_RX_PIN (GPIO_NUM_16)
#define CRSF_UART_TX_PIN (GPIO_NUM_17)
#define CRSF_UART_BAUDRATE 115200
