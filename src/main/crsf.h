#pragma once

#include <Arduino.h>
#include "drivers/serial.h"
#include "main/msp.h"

extern serial_port_t crsfPort;

void crsfTask(void * parameter);
void sendMspPacket(mspPacket_t *mspPacket);
