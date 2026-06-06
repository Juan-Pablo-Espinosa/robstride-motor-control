#pragma once
#include "transport.hpp"
#include <string>

class SocketCANTransport : public Transport {
public:
    explicit SocketCANTransport(const std::string& interface);
    ~SocketCANTransport() override;

    bool open() override;
    void close() override;
    bool send(const CANFrame& frame) override;
    bool recv(CANFrame& frame, int timeout_ms) override;
    bool isOpen() const override;

private:
    std::string iface_;
    int sock_ = -1;
};
