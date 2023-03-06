#include "main/msp.h"
#include "main/crsf.h"
#include "main/mavlink.h"

bool setWaypoint(uint8_t wpNumber, const navWaypoint_t *wpData)
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_SET_WP;

    mspPacket.payloadSize = 1 + sizeof (navWaypoint_t);
    mspPacket.payload[0] = wpNumber; // waypoint number
    if (mspPacket.payloadSize > MSP_PORT_INBUF_SIZE) 
        return false;
    memcpy(&mspPacket.payload[1], wpData, sizeof (navWaypoint_t));

    sendMspPacket(&mspPacket);

    return true;
}

bool getWaypoint(uint8_t wpNumber)
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_WP;

    mspPacket.payloadSize = 1;
    mspPacket.payload[0] = wpNumber; // waypoint number

    sendMspPacket(&mspPacket);

    return true;
}

bool getWaypointCount()
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_WP_GETINFO;
    mspPacket.payloadSize = 0;

    sendMspPacket(&mspPacket);

    return true;
}

void mspParse(uint16_t cmd, uint8_t *payload, uint16_t payloadSize)
{
    if (cmd == MSP_WP_GETINFO) {
        uint8_t p = 0; // U8 MSP Size
        p++; // U8 Reserved for waypoint capabilities
        p++; // U8 Maximum number of waypoints supported
        p++; // U8 Is current mission valid
        uint8_t waypointsCount = payload[p++]; // U8 Number of waypoints in current mission

        mavlinkWaypointsCount(waypointsCount);
    } else
    if (cmd == MSP_WP) {
        uint8_t p = 0; // U8 MSP Size
        uint8_t wp_no = payload[p++]; // U8 wp_no
        uint8_t action = payload[p++]; // U8 action (WAYPOINT)
        uint32_t lat = 0; // U32 lat
        lat |= payload[p++] << 0;
        lat |= payload[p++] << 8;
        lat |= payload[p++] << 16;
        lat |= payload[p++] << 24;
        uint32_t lon = 0; // U32 lon
        lon |= payload[p++] << 0;
        lon |= payload[p++] << 8;
        lon |= payload[p++] << 16;
        lon |= payload[p++] << 24;
        uint32_t alt = 0; // U32 altitude (cm)
        alt |= payload[p++] << 0;
        alt |= payload[p++] << 8;
        alt |= payload[p++] << 16;
        alt |= payload[p++] << 24;
        // U16 P1
        // U16 P2
        // U16 P3
        // U8 flags

        mavlinkWP(wp_no, action, lat, lon, alt);
    }
}

void mspTask(void * parameter)
{
    for( ;; ) {
        //getWaypointCount();
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}