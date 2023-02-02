#pragma once

#include <Arduino.h>
#include "drivers/serial.h"
#include "lib/mavlink/ASLUAV/mavlink.h"

extern serial_port_t mavlinkPort;
extern SemaphoreHandle_t mavlinkMutex;

void mavlinkTask(void * parameter);
void mavlinkWaypointsCount(uint8_t waypointsCount);
void mavlinkWP(uint8_t wp_no, uint8_t action, uint32_t lat, uint32_t lon, uint32_t alt);
