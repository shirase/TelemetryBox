
--[[local function crc8(t)
    local c = 0
    for _, b in ipairs(t) do
       for i = 0, 7 do
          local c0 = c % 2
          local b0 = b % 2
          c = (c - c0) / 2
          b = (b - b0) / 2
          if c0 + b0 == 1 then
             c = c + 0x80 + (c % 16 < 8 and 8 or -8) + (c % 8 < 4 and 4 or -4)
          end
       end
    end
    return c
 end
 ]]--

local xor_tab = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, },
    {1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, },
    {2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13, },
    {3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12, },
    {4, 5, 6, 7, 0, 1, 2, 3, 12, 13, 14, 15, 8, 9, 10, 11, },
    {5, 4, 7, 6, 1, 0, 3, 2, 13, 12, 15, 14, 9, 8, 11, 10, },
    {6, 7, 4, 5, 2, 3, 0, 1, 14, 15, 12, 13, 10, 11, 8, 9, },
    {7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, },
    {8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7, },
    {9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, },
    {10, 11, 8, 9, 14, 15, 12, 13, 2, 3, 0, 1, 6, 7, 4, 5, },
    {11, 10, 9, 8, 15, 14, 13, 12, 3, 2, 1, 0, 7, 6, 5, 4, },
    {12, 13, 14, 15, 8, 9, 10, 11, 4, 5, 6, 7, 0, 1, 2, 3, },
    {13, 12, 15, 14, 9, 8, 11, 10, 5, 4, 7, 6, 1, 0, 3, 2, },
    {14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1, },
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, },
}

function bxor( a, b )

    local res = 0
    local c = 1

    while a > 0 and b > 0 do
        local a2 = a % 16
        local b2 = b % 16
        res = res + xor_tab[ a2 + 1 ][ b2 + 1 ] * c
        a = ( a - a2 ) / 16
        b = ( b - b2 ) / 16
        c = c * 16
    end

    return( res + a * c + b * c )
end

local crc_tab = {
    0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54,
    0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
    0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06,
    0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
    0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0,
    0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
    0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2,
    0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
    0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9,
    0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
    0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b,
    0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
    0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d,
    0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
    0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f,
    0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
    0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb,
    0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
    0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9,
    0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
    0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f,
    0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
    0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d,
    0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
    0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26,
    0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
    0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74,
    0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
    0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82,
    0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
    0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0,
    0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
}

function crc8( t )

    local crc = 0

    for i = 1, #t do
        crc = crc_tab[ bxor( crc, t[ i ] ) + 1 ]
    end

    return crc
end

local function serialWriteCrossfire(command, data)
    serialWrite('\0')
    local l = #data + 2
    serialWrite(string.char(l))
    serialWrite(string.char(command))

    local crc = {command}

    if (#data > 0) then
        for i = 1, #data do
            serialWrite(string.char(data[i]))
        end

        for j = 1, #data do
            crc[j + 1] = data[j]
        end
    end

    serialWrite(string.char(crc8(crc)))
end

local function int2bytes( value, size )
    local t = {}
    for i = size - 1, 0, -1 do
        t[#t + 1] = (bit32.rshift(value, 8 * i) % 256)
    end
    return t
end

CrossfirePacket = {}
CrossfirePacketLength = 0
CrossfirePacketStage = 0

local function readCrossfire(byte)
    if CrossfirePacketStage == 0 then
        if byte == 0x00 or byte == 0xC8 or byte == 0xEA or byte == 0xEE then
            CrossfirePacketStage = 1
            CrossfirePacket = {}
            CrossfirePacket[#CrossfirePacket + 1] = byte
            return false
        end
    elseif CrossfirePacketStage == 1 then
        CrossfirePacketStage = 2
        CrossfirePacketLength = byte + 2
        if CrossfirePacketLength <= 64 then
            CrossfirePacket[#CrossfirePacket + 1] = byte
        else
            CrossfirePacketStage = 0;
        end
        return false
    elseif CrossfirePacketStage == 2 then
        CrossfirePacket[#CrossfirePacket + 1] = byte
        if #CrossfirePacket == CrossfirePacketLength then
            CrossfirePacketStage = 0
            return true
        end
    end
end

local RAD = 0.01745329251994329576923690768489

local function run()
    local rssi = getRSSI()

    if rssi ~= 0 then
        local command, data = crossfireTelemetryPop()
        if command then
            --local debug = int2bytes(command, 4)
            --serialWriteCrossfire(0x7E, debug)

            serialWriteCrossfire(command, data)
        else
            local gps = getValue('GPS')
            local altitude = getValue('Alt')
            local hdg = getValue('Hdg')
            local sats = getValue('Sats')

            local frame = {}
            local bytes

            bytes = int2bytes(gps.lat * 1e+7, 4)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            bytes = int2bytes(gps.lon * 1e+7, 4)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            -- speed
            frame[#frame + 1] = 0x00
            frame[#frame + 1] = 0x00

            -- heading
            bytes = int2bytes(hdg, 2)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            -- altitude
            bytes = int2bytes(altitude + 1000, 2)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            -- sats
            bytes = int2bytes(sats, 1)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            serialWriteCrossfire(0x02, frame)

            local pitch = getValue('Ptch')
            pitch = pitch * RAD * 10000

            local roll = getValue('Roll')
            roll = roll * RAD * 10000

            local yaw = getValue('Yaw')
            yaw = yaw * RAD * 10000

            frame = {}

            bytes = int2bytes(pitch, 2)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            bytes = int2bytes(roll, 2)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            bytes = int2bytes(yaw, 2)
            for i = 1, #bytes do
                frame[#frame + 1] = bytes[i]
            end

            serialWriteCrossfire(0x1E, frame)
        end

        local str = serialRead(64)
        if #str > 0 then
            --local debug = int2bytes(#str, 4)
            --serialWriteCrossfire(0x7E, debug)

            for i = 1, #str do
                if readCrossfire(string.byte(str, i)) then
                    local t = {}
                    for j = 4, #CrossfirePacket - 1 do
                        t[j - 3] = CrossfirePacket[j]
                    end
                    crossfireTelemetryPush(CrossfirePacket[3], t)
                end
            end
        end
    end
end

return {run=run}