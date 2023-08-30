#include "crsf.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include <stdbool.h>
#include "input.h"
#include "common/math.h"
#include "lib/CRC/crc.h"

#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC 4 // length of Extended Dest/Origin, TYPE and CRC fields combined
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define ENCAPSULATED_MSP_HEADER_CRC_LEN     4
#define ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE   4
#define ENCAPSULATED_MSP_MAX_FRAME_LEN      (ENCAPSULATED_MSP_HEADER_CRC_LEN + ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE)
#define TELEMETRY_MSP_VERSION    1
#define TELEMETRY_MSP_VERSION_2  2
#define TELEMETRY_MSP_VER_SHIFT  5
#define TELEMETRY_MSP_VER_MASK   (0x7 << TELEMETRY_MSP_VER_SHIFT)
#define TELEMETRY_MSP_SEQ_MASK   0x0F
#define TELEMETRY_MSP_START_FLAG (1 << 4)
#define TELEMETRY_MSP_ERROR_FLAG (1 << 5)

serial_port_t crsfPort = {};

enum { CRSF_FRAME_SIZE_MAX = 64 };
enum { CRSF_FRAME_LENGTH_NOT_COUNTED_BYTES = 2 }; // address, length
enum { CRSF_FRAME_LENGTH_PAYLOAD_NOT_COUNTED_BYTES = 2 }; // type, crc
enum { CRSF_FRAME_PAYLOAD_EXT_HEADER_SIZE = 2 }; // dest_addr, orig_addr
enum { CRSF_FRAME_LENGTH_MAX = CRSF_FRAME_SIZE_MAX - CRSF_FRAME_LENGTH_NOT_COUNTED_BYTES }; // frame - address - length
enum { CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_LENGTH_MAX - CRSF_FRAME_LENGTH_PAYLOAD_NOT_COUNTED_BYTES }; // type, crc

#define CRSF_SYNC_BYTE  0XC8

typedef enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
} crsfAddress_e;

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,

    CRSF_FRAMETYPE_DEBUG = 0x7E
} crsf_frame_type_e;

typedef struct crsfFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength; // type, payload, crc
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union crsfFrame_u {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;

crsfFrame_t crsfFrame;

typedef enum {
    CRSF_READ_STATE_IDLE = 0,
    CRSF_READ_STATE_HEADER,
    CRSF_READ_STATE_BODY,
} crsfReadState_e;

crsfReadState_e crsfReadState = CRSF_READ_STATE_IDLE;

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void crsfParse()
{
    switch (crsfFrame.frame.type)
    {
    case CRSF_FRAMETYPE_MSP_RESP:
        {
            //Serial.write("CRSF_FRAMETYPE_MSP_RESP");

            uint8_t *mspStart = crsfFrame.frame.payload + CRSF_FRAME_PAYLOAD_EXT_HEADER_SIZE;
            const uint8_t mspHeader = *mspStart;
            const uint8_t version = (mspHeader & TELEMETRY_MSP_VER_MASK) >> TELEMETRY_MSP_VER_SHIFT;

            mspStart++;

            static uint16_t mspPayloadSize;
            uint16_t cmd = 0;
            uint8_t mspFramePayloadSize = crsfFrame.frame.frameLength - 5; // type byte - crc byte - dest_addr byte - orig_addr byte - msp header byte

            if (mspHeader & TELEMETRY_MSP_START_FLAG) {
                if (version == TELEMETRY_MSP_VERSION_2) { // todo
                    mspPayloadSize = (mspStart[0] | (mspStart[1] << 8));
                    cmd = (mspStart[2] | (mspStart[3] << 8));
                    if (mspHeader & TELEMETRY_MSP_ERROR_FLAG) {
                        Serial.write("MSP_ERROR_V2");
                        Serial.write(&mspStart[2], mspPayloadSize);
                        break;
                    }
                } else {
                    if (mspHeader & TELEMETRY_MSP_ERROR_FLAG) {
                        Serial.write("MSP_ERROR");
                        Serial.write(&mspStart[1], mspStart[0]);
                        break;
                    }

                    mspPayloadSize = mspStart[0];
                    uint8_t mspSize = mspPayloadSize + 2; // payload size + size byte + crc byte

                    // restore cmd byte from CRC
                    for (uint8_t i = 0; i < mspSize; i++) {
                        cmd ^= mspStart[i];
                    }
                }
            }

            if (mspPayloadSize <= mspFramePayloadSize) {
                mspParse(cmd, mspStart + 1 /* first byte is msp payload size */, mspPayloadSize);
            } else {
                // frame sequence, todo
            }
        }

        break;

    case CRSF_FRAMETYPE_GPS:
        {
            uint8_t *byte = crsfFrame.frame.payload;
            int32_t gpsSol_llh_lat = *byte << 24 | *(byte + 1) << 16 | *(byte + 2) << 8 | *(byte + 3);
            byte += 4;
            int32_t gpsSol_llh_lon = *byte << 24 | *(byte + 1) << 16 | *(byte + 2) << 8 | *(byte + 3);
            byte += 4;
            int16_t gpsSol_groundSpeed = *(byte) << 8 | *(byte + 1);
            byte += 2;
            int16_t gpsSol_groundCourse = *(byte) << 8 | *(byte + 1);
            byte += 2;
            uint16_t gpsSol_llh_alt = *(byte) << 8 | *(byte + 1);
            byte += 2;
            uint8_t gpsSol_numSat = *(byte);
            byte += 1;

            lat = gpsSol_llh_lat / 1e+7;
            lon = gpsSol_llh_lon / 1e+7;
            groundspeed = gpsSol_groundSpeed / 100;
            heading = gpsSol_groundCourse / 100;
            gps_alt = gpsSol_llh_alt - 1000;
            gps_sats = gpsSol_numSat;

            fixType = 3;
        }

        break;

    case CRSF_FRAMETYPE_ATTITUDE:
        {
            uint8_t *byte = crsfFrame.frame.payload;
            int16_t attitude_pitch = *(byte) << 8 | *(byte + 1);
            byte += 2;
            int16_t attitude_roll = *(byte) << 8 | *(byte + 1);
            byte += 2;
            int16_t attitude_yaw = *(byte) << 8 | *(byte + 1);
            byte += 2;

            int32_t pitch100 = (attitude_pitch / (RAD * 100));
            int32_t roll100 = (attitude_roll / (RAD * 100));
            int32_t yaw100 = (attitude_yaw / (RAD * 100));

            pitch = -(pitch100 / 100);
            roll = roll100 / 100;
            yaw = yaw100 / 100;
        }
        break;

    case CRSF_FRAMETYPE_DEBUG:
        {
            Serial.write(CRSF_FRAMETYPE_DEBUG);
            uint8_t *payload = crsfFrame.frame.payload;
            Serial.write(payload, crsfFrame.frame.frameLength - 2);
        }
        break;
    
    default:
        break;
    }
}

void crsfReadByte(uint8_t byte)
{
    static uint8_t position = 0;

    //Serial.write(byte);

    switch (crsfReadState)
    {
    case CRSF_READ_STATE_IDLE:
        if (byte == CRSF_ADDRESS_RADIO_TRANSMITTER || byte == CRSF_SYNC_BYTE || byte == CRSF_ADDRESS_BROADCAST) {
            position = 0;
            crsfFrame.bytes[position++] = byte;

            crsfReadState = CRSF_READ_STATE_HEADER;
        } else {
            //Serial.write(byte);
        }
        break;

    case CRSF_READ_STATE_HEADER:
        crsfFrame.bytes[position++] = byte;

        if (position == 3) {
            if (crsfFrame.frame.frameLength <= CRSF_FRAME_LENGTH_MAX) {
                crsfReadState = CRSF_READ_STATE_BODY;
            } else {
                position = 0;
                crsfReadState = CRSF_READ_STATE_IDLE;
            }
        }

        break;

    case CRSF_READ_STATE_BODY:
        crsfFrame.bytes[position++] = byte;
        if (position == crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_NOT_COUNTED_BYTES) {
            uint8_t frameCrc = byte;

            uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
            for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_PAYLOAD_NOT_COUNTED_BYTES /* type, crc */; ii++) {
                crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
            }

            crsfReadState = CRSF_READ_STATE_IDLE;
            position = 0;

            if (crc == frameCrc) {
                //Serial.write(crsfFrame.bytes, crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_NOT_COUNTED_BYTES);
                crsfParse();
            }
        }

        break;
    
    default:
        crsfReadState = CRSF_READ_STATE_IDLE;
    }
}

void crsfTask(void * parameter)
{
    uint8_t buf[CRSF_FRAME_SIZE_MAX];

    for( ;; ) {
        uint8_t len = crsfPort.readBuf(crsfPort.port, buf, sizeof(buf));
        for (int i = 0; i < len; i++) {
            crsfReadByte(buf[i]);
        }

        vTaskDelay(1);
    }
}

#define CRSF_CRC_POLY 0xd5
GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

static inline uint8_t ICACHE_RAM_ATTR CalcCRCMsp(uint8_t *data, int length, uint8_t crc = 0)
{
    for (uint8_t i = 0; i < length; ++i) {
        crc = crc ^ *data++;
    }
    return crc;
}

void sendMspPacket(mspPacket_t *packet)
{
    if (packet->function >= 0xFF) {
        // MSP V2 not supported by CRSF telemetry
        Serial.write("MSP V2 not supported by CRSF telemetry");
        return;
    }

    uint8_t outBuffer[ENCAPSULATED_MSP_MAX_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES];

    if (packet->payloadSize > ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE)
    {
        // send MSP sequence
        uint8_t seqNumber = 0;
        uint8_t payloadOffset = 0;

        while (payloadOffset < packet->payloadSize + 1) { // 1 byte for MSP CRC
            uint8_t header = TELEMETRY_MSP_VERSION << TELEMETRY_MSP_VER_SHIFT | (seqNumber & TELEMETRY_MSP_SEQ_MASK);
            seqNumber++;

            uint8_t outBufferOffset = 0;
            uint8_t mspFrameRemaining = ENCAPSULATED_MSP_MAX_FRAME_LEN;
            outBuffer[outBufferOffset++] = CRSF_ADDRESS_BROADCAST;                                     // address
            outBuffer[outBufferOffset++] = ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // length
            outBuffer[outBufferOffset++] = CRSF_FRAMETYPE_MSP_WRITE;                                    // packet type
            outBuffer[outBufferOffset++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;                              // destination
            outBuffer[outBufferOffset++] = CRSF_ADDRESS_RADIO_TRANSMITTER;                              // origin

            // Encapsulated MSP payload
            if (seqNumber == 0) {
                outBuffer[outBufferOffset++] = header | TELEMETRY_MSP_START_FLAG; // header
                mspFrameRemaining--;
                outBuffer[outBufferOffset++] = packet->payloadSize; // mspPayloadSize
                mspFrameRemaining--;
                outBuffer[outBufferOffset++] = packet->function;    // packet->cmd
                mspFrameRemaining--;
            } else {
                outBuffer[outBufferOffset++] = header; // header
                mspFrameRemaining--;
            }

            while (mspFrameRemaining > 0) {
                mspFrameRemaining--;
                // copy packet payload into outBuffer
                if (payloadOffset < packet->payloadSize) {
                    outBuffer[outBufferOffset++] = packet->payload[payloadOffset++];
                }
                if (payloadOffset == packet->payloadSize && mspFrameRemaining > 0) {
                    // add MSP crc
                    uint8_t crc = 0;
                    crc = CalcCRCMsp((uint8_t *)&packet->payloadSize, 1, crc);
                    crc = CalcCRCMsp((uint8_t *)&packet->function, 1, crc);
                    outBuffer[outBufferOffset++] = CalcCRCMsp(packet->payload, packet->payloadSize, crc);
                    payloadOffset++;
                    break;
                }
            }

            outBuffer[outBufferOffset] = crsf_crc.calc(&outBuffer[2], outBufferOffset - 2);
            outBufferOffset++;

            crsfPort.writeBuf(crsfPort.port, outBuffer, outBufferOffset);
        }

        return;
    }

    const uint8_t totalBufferLen = packet->payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES;

    // CRSF extended frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;                                      // address
    outBuffer[1] = packet->payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // length
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;                                    // packet type
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER;                              // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;                              // origin

    // Encapsulated MSP payload
    outBuffer[5] = 0x30;                // header
    outBuffer[6] = packet->payloadSize; // mspPayloadSize
    outBuffer[7] = packet->function;    // packet->cmd
    for (uint8_t i = 0; i < packet->payloadSize; ++i)
    {
        // copy packet payload into outBuffer
        outBuffer[8 + i] = packet->payload[i];
    }
    // Encapsulated MSP crc
    outBuffer[totalBufferLen - 2] = CalcCRCMsp(&outBuffer[6], packet->payloadSize + 2);

    // CRSF frame crc
    outBuffer[totalBufferLen - 1] = crsf_crc.calc(&outBuffer[2], packet->payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC - 1);

    crsfPort.writeBuf(crsfPort.port, outBuffer, totalBufferLen);
}