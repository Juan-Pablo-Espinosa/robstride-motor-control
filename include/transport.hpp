#pragma once
#include <cstdint>

struct CANFrame {
    uint32_t can_id;  // 29-bit ID, WITHOUT CAN_EFF_FLAG — transport adds it on TX
    uint8_t  data[8];
    uint8_t  len;
};

class Transport {
public:
    virtual ~Transport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool send(const CANFrame& frame) = 0;
    virtual bool recv(CANFrame& frame, int timeout_ms) = 0;
    virtual bool recvNonBlocking(CANFrame& frame) = 0;
    virtual bool isOpen() const = 0;
};
