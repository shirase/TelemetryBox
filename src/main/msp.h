#pragma once

#include <Arduino.h>

#define MSP_PORT_INBUF_SIZE 65

#define MSP_WP              118
#define MSP_SET_WP          209
#define MSP_WP_GETINFO      20
#define NAV_MAX_WAYPOINTS   120

typedef enum {
    MSP_PACKET_UNKNOWN,
    MSP_PACKET_COMMAND,
    MSP_PACKET_RESPONSE
} mspPacketType_e;

typedef struct __attribute__((packed)) {
    uint8_t size;
    uint8_t cmd;
} mspHeaderV1_t;

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    uint16_t function;
    uint16_t payloadSize;
} mspHeaderV2_t;

typedef struct {
    mspPacketType_e type;
    uint8_t         flags;
    uint16_t        function;
    uint16_t        payloadSize;
    uint8_t         payload[MSP_PORT_INBUF_SIZE];
    uint16_t        payloadReadIterator;
    bool            readError;
    uint8_t         version;

    void reset()
    {
        type = MSP_PACKET_UNKNOWN;
        flags = 0;
        function = 0;
        payloadSize = 0;
        payloadReadIterator = 0;
        readError = false;
        version = 0;
    }

    void addByte(uint8_t b)
    {
        payload[payloadSize++] = b;
    }

    void makeResponse()
    {
        type = MSP_PACKET_RESPONSE;
    }

    void makeCommand()
    {
        type = MSP_PACKET_COMMAND;
    }

    uint8_t readByte()
    {
        if (payloadReadIterator >= payloadSize) {
            // We are trying to read beyond the length of the payload
            readError = true;
            return 0;
        }

        return payload[payloadReadIterator++];
    }
} mspPacket_t;

typedef struct __attribute__ ((packed)) {
    uint8_t action;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t p1, p2, p3;
    uint8_t flag;
} navWaypoint_t;

typedef enum {
    NAV_WP_ACTION_WAYPOINT  = 0x01,
    NAV_WP_ACTION_HOLD_TIME = 0x03,
    NAV_WP_ACTION_RTH       = 0x04,
    NAV_WP_ACTION_SET_POI   = 0x05,
    NAV_WP_ACTION_JUMP      = 0x06,
    NAV_WP_ACTION_SET_HEAD  = 0x07,
    NAV_WP_ACTION_LAND      = 0x08
} navWaypointActions_e;

typedef enum {
    NAV_WP_FLAG_HOME = 0x48,
    NAV_WP_FLAG_LAST = 0xA5
} navWaypointFlags_e;

bool setWaypoint(uint8_t wpNumber, const navWaypoint_t *wpData);
bool getWaypoint(uint8_t wpNumber);
bool getWaypointCount();
void mspParse(uint16_t cmd, uint8_t *payload, uint16_t payloadSize);
