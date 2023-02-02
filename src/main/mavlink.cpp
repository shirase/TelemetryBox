#include "main/mavlink.h"
#include "lib/mavlink/ASLUAV/mavlink.h"
#include "main/msp.h"
#include "main/input.h"

serial_port_t mavlinkPort = {};

static mavlink_message_t mavSendMsg;
static mavlink_message_t mavRecvMsg;
static mavlink_status_t mavRecvStatus;

static int incomingMissionWpCount = 0;
static int incomingMissionWpSequence = 0;

SemaphoreHandle_t mavlinkMutex = xSemaphoreCreateMutex();

bool mavlinkSendMessage(void)
{
    xSemaphoreTake(mavlinkMutex, portMAX_DELAY);

    uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
    int msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavSendMsg);

    bool res = (bool)mavlinkPort.writeBuf(mavlinkPort.port, mavBuffer, msgLength);

    xSemaphoreGive(mavlinkMutex);

    return res;
}

bool handleIncoming_MISSION_CLEAR_ALL(void)
{
    mavlink_mission_clear_all_t msg;
    mavlink_msg_mission_clear_all_decode(&mavRecvMsg, &msg);

    if (msg.target_system == system_id) {
        // set zero waypoint for reset
        navWaypoint_t navWaypoint = {0};
        setWaypoint(1, &navWaypoint);

        mavlink_msg_mission_ack_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        mavlinkSendMessage();

        return true;
    }

    return false;
}

bool handleIncoming_MISSION_COUNT(void)
{
    mavlink_mission_count_t msg;
    mavlink_msg_mission_count_decode(&mavRecvMsg, &msg);

    if (msg.target_system == system_id) {
        incomingMissionWpCount = msg.count; // We need to know how many items to request
        incomingMissionWpSequence = 0;
        mavlink_msg_mission_request_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, incomingMissionWpSequence, MAV_MISSION_TYPE_MISSION);
        mavlinkSendMessage();
        return true;
    }

    return false;
}

bool handleIncoming_MISSION_ITEM(void)
{
    mavlink_mission_item_t msg;
    mavlink_msg_mission_item_decode(&mavRecvMsg, &msg);

    if (msg.target_system == system_id) {
        if (msg.seq == incomingMissionWpSequence) {
            incomingMissionWpSequence++;

            navWaypoint_t wp;
            wp.action = (msg.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) ? NAV_WP_ACTION_RTH : NAV_WP_ACTION_WAYPOINT;
            wp.lat = (int32_t)(msg.x * 1e7f);
            wp.lon = (int32_t)(msg.y * 1e7f);
            wp.alt = msg.z * 100.0f;
            wp.p1 = 0;
            wp.p2 = 0;
            wp.p3 = 0;
            wp.flag = (incomingMissionWpSequence >= incomingMissionWpCount) ? NAV_WP_FLAG_LAST : 0;

            setWaypoint(incomingMissionWpSequence, &wp);

            if (incomingMissionWpSequence >= incomingMissionWpCount) {
                mavlink_msg_mission_ack_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            }
            else {
                mavlink_msg_mission_request_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, incomingMissionWpSequence, MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            }
        }
        else {
            // Wrong sequence number received
            mavlink_msg_mission_ack_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
            mavlinkSendMessage();
        }

        return true;
    }

    return false;
}

bool handleIncoming_MISSION_REQUEST_LIST(void)
{
    mavlink_mission_request_list_t msg;
    mavlink_msg_mission_request_list_decode(&mavRecvMsg, &msg);

    if (msg.target_system == system_id) {
        getWaypointCount();
        return true;
    }

    return true;
}

bool handleIncoming_MISSION_REQUEST(void)
{
    mavlink_mission_request_t msg;
    mavlink_msg_mission_request_decode(&mavRecvMsg, &msg);

    if (msg.target_system == system_id) {
        getWaypoint(msg.seq + 1);
        return true;
    }

    return false;
}

void mavlinkReadByte(uint8_t byte)
{
    uint8_t result = mavlink_parse_char(0, byte, &mavRecvMsg, &mavRecvStatus);
    if (result == MAVLINK_FRAMING_OK) {
        switch (mavRecvMsg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                break;
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                handleIncoming_MISSION_CLEAR_ALL();
                break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                handleIncoming_MISSION_COUNT();
                break;
            case MAVLINK_MSG_ID_MISSION_ITEM:
                handleIncoming_MISSION_ITEM();
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                handleIncoming_MISSION_REQUEST_LIST();
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST:
                handleIncoming_MISSION_REQUEST();
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                break;
            default:
                break;
        }
    }
}

void mavlinkTask(void * parameter)
{
    uint8_t buf[128];

    for( ;; ) {
        uint8_t len = mavlinkPort.readBuf(mavlinkPort.port, buf, sizeof(buf));
        for (int i = 0; i < len; i++) {
            mavlinkReadByte(buf[i]);
        }

        vTaskDelay(1);
    }
}

void mavlinkWaypointsCount(uint8_t waypointsCount)
{
    mavlink_msg_mission_count_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, waypointsCount, MAV_MISSION_TYPE_MISSION);
    mavlinkSendMessage();
}

void mavlinkWP(uint8_t wp_no, uint8_t action, uint32_t lat, uint32_t lon, uint32_t alt)
{
    mavlink_msg_mission_item_pack(system_id, component_id, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid,
            wp_no - 1,
            action == NAV_WP_ACTION_RTH ? MAV_FRAME_MISSION : MAV_FRAME_GLOBAL_RELATIVE_ALT,
            action == NAV_WP_ACTION_RTH ? MAV_CMD_NAV_RETURN_TO_LAUNCH : MAV_CMD_NAV_WAYPOINT,
            0,
            1,
            0, 0, 0, 0,
            lat / 1e7f,
            lon / 1e7f,
            alt / 100.0f,
            MAV_MISSION_TYPE_MISSION);

    mavlinkSendMessage();
}
