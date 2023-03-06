
local function crc8(t)
    local c = 0
    for _, b in ipairs(t) do
       for i = 0, 7 do
          c = c >> 1 ~ ((c ~ b >> i) & 1) * 0x8C
       end
    end
    return c
end

local function serialWriteCrossfire(command, data)
    serialWrite(0x00)
    serialWrite(#data + 2)
    serialWrite(command)
    serialWrite(data, #data)

    local t = {command}
    for j = 1, #data do
        t[j + 1] = data[j]
    end

    serialWrite(crc8(t))
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
            table.insert(CrossfirePacket, byte)
            return false
        end
    elseif CrossfirePacketStage == 1 then
        CrossfirePacketStage = 2
        CrossfirePacketLength = byte + 2
        table.insert(CrossfirePacket, #CrossfirePacket + 1, byte)
        return false
    elseif CrossfirePacketStage == 2 then
        table.insert(CrossfirePacket, #CrossfirePacket + 1, byte)
        if #CrossfirePacket == CrossfirePacketLength then
            CrossfirePacketStage = 0
            return true
        end
    end
end

local function run()
    local command, data = crossfireTelemetryPop()
    if command then
        serialWriteCrossfire(command, data)
    else
        local gps = getValue('GPS')
        local altitude = getValue('Alt')

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

        frame[#frame + 1] = 0x00
        frame[#frame + 1] = 0x00

        int2bytes(altitude + 1000, 2)
        for i = 1, #bytes do
            frame[#frame + 1] = bytes[i]
        end

        frame[#frame + 1] = 0x00

        serialWriteCrossfire(0x02, frame)
    end

    local str = serialRead()
    if #str then
        local bytes = string.byte(str, 1, #str)
        for i = 1, #bytes do
            if readCrossfire(bytes[i]) then
                local t = {};
                for j = 4, #CrossfirePacket - 1 do
                    t[j - 3] = CrossfirePacket[j];
                end
                crossfireTelemetryPush(CrossfirePacket[3], t)
            end
        end
    end
end

return {run=run}