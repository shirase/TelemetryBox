#pragma once

#include <Arduino.h>
#include <stdbool.h>

typedef struct {
    uint8_t *port;
    bool(*readByte)(uint8_t *port, uint8_t *byte);
    bool(*writeByte)(uint8_t *port, uint8_t byte);
    uint16_t(*readBuf)(uint8_t *port, uint8_t *buf, uint16_t len);
    uint16_t(*writeBuf)(uint8_t *port, uint8_t *buf, uint16_t len);
} serial_port_t;

void serialInit(serial_port_t *serialPort, Stream* serial);
