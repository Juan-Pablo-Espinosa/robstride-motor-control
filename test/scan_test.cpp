// ---------------------------------------------------------------------------
// scan_test.cpp
// Milestone 1: discover RobStride motors on can1
//
// Expected output with motor ID 42 connected:
//   Opening can1...
//   Scanning IDs 1-127...
//   Found motor ID: 42
//   Scan complete. Found 1 motor(s).
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "robstride_motor.hpp"
#include "motor_config.hpp"
#include <cstdio>

int main() {
    // Open transport on can1 — motor is always on can1 (spi0.0, J1/CAN-A)
    SocketCANTransport transport("can1");

    printf("Opening can1...\n");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1. Is it up? Run:\n");
        fprintf(stderr, "  sudo ip link set can1 type can bitrate 1000000\n");
        fprintf(stderr, "  sudo ip link set can1 up\n");
        return 1;
    }

    // Motor ID doesn't matter for scan — we probe all IDs
    // Using ID 1 as placeholder; scan() overrides it internally
    RobstrideMotor motor(transport, 1, MotorConfig{});

    printf("Scanning IDs 1-127...\n");
    auto found = motor.scan(1, 127);

    printf("Scan complete. Found %zu motor(s).\n", found.size());
    for (uint8_t id : found) {
        printf("  Motor ID: %d\n", id);
    }

    transport.close();
    return 0;
}
