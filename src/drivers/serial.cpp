#include "drivers/serial.h"

bool serialReadByte(uint8_t *port, uint8_t *byte)
{
    Stream* serial = (Stream*)port;
    *byte = serial->read();
    return (*byte > -1);
}

bool serialWriteByte(uint8_t *port, uint8_t byte)
{
    Stream* serial = (Stream*)port;
    return serial->write(byte);
}

uint16_t serialReadBuf(uint8_t *port, uint8_t *buf, uint16_t len)
{
    Stream* serial = (Stream*)port;
    return serial->readBytes(buf, len);
}

uint16_t serialWriteBuf(uint8_t *port, uint8_t *buf, uint16_t len)
{
    Stream* serial = (Stream*)port;
    return serial->write(buf, len);
}

void serialInit(serial_port_t *serialPort, Stream* serial)
{
    serialPort->port = (uint8_t*)serial;
    serialPort->readByte = serialReadByte;
    serialPort->writeByte = serialWriteByte;
    serialPort->readBuf = serialReadBuf;
    serialPort->writeBuf = serialWriteBuf;
}
