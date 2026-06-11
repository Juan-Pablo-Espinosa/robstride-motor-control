#include "socketcan_transport.hpp"
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

SocketCANTransport::SocketCANTransport(const std::string& interface)
    : iface_(interface) {}

SocketCANTransport::~SocketCANTransport() {
    close();
}

bool SocketCANTransport::open() {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        perror("socket");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    return true;
}

void SocketCANTransport::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool SocketCANTransport::send(const CANFrame& frame) {
    struct can_frame tx = {};
    tx.can_id  = frame.can_id | CAN_EFF_FLAG;  // always 29-bit extended
    tx.can_dlc = frame.len;
    std::memcpy(tx.data, frame.data, frame.len);
    return ::write(sock_, &tx, sizeof(tx)) == sizeof(tx);
}

bool SocketCANTransport::recv(CANFrame& frame, int timeout_ms) {
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct can_frame rx = {};
    int n = ::read(sock_, &rx, sizeof(rx));
    if (n <= 0) return false;

    frame.can_id = rx.can_id & ~CAN_EFF_FLAG;  // strip the flag on RX
    frame.len    = rx.can_dlc;
    std::memcpy(frame.data, rx.data, rx.can_dlc);
    return true;
}

bool SocketCANTransport::recvNonBlocking(CANFrame& frame) {
    struct can_frame rx = {};
    int n = ::recv(sock_, &rx, sizeof(rx), MSG_DONTWAIT);
    if (n <= 0) return false;
    frame.can_id = rx.can_id & ~CAN_EFF_FLAG;
    frame.len    = rx.can_dlc;
    std::memcpy(frame.data, rx.data, rx.can_dlc);
    return true;
}

bool SocketCANTransport::isOpen() const {
    return sock_ >= 0;
}
