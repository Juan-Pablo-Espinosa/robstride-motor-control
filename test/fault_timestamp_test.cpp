// fault_timestamp_test.cpp
// Tests:
//   1. Both motors show up, enable cleanly
//   2. last_update timestamp is populated and age stays fresh (<10ms)
//   3. isFaulted() returns false under normal operation
//   4. Press 'c' to clearFault() on both motors (manual test)
//   5. Power-cycle a motor mid-run to observe auto-disable + faulted flag

#include "motor_bus.hpp"
#include "socketcan_transport.hpp"
#include <cstdio>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>

// Non-blocking keypress
static int kbhit() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN]  = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main() {
    SocketCANTransport transport("can1");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1\n");
        return 1;
    }

    MotorBus bus(transport, 200);

    // RS-03 ID 42
    MotorConfig cfg03;
    cfg03.model            = MotorModel::RS03;
    cfg03.joint_name       = "test_rs03";
    cfg03.max_acceleration = 5.0f;

    // RS-04 ID 127
    MotorConfig cfg04;
    cfg04.model            = MotorModel::RS04;
    cfg04.joint_name       = "test_rs04";
    cfg04.max_acceleration = 5.0f;

    bus.addMotor(42,  cfg03);
    bus.addMotor(127, cfg04);

    printf("Enabling motors...\n");
    bool ok42  = bus.enable(42);
    bool ok127 = bus.enable(127);
    printf("Motor 42  enable: %s\n", ok42  ? "OK" : "FAILED");
    printf("Motor 127 enable: %s\n", ok127 ? "OK" : "FAILED");

    if (!ok42 && !ok127) {
        fprintf(stderr, "No motors enabled — check CAN and power\n");
        return 1;
    }

    bus.start();
    printf("\nRunning at 200Hz. Press 'c' to clearFault both, 'q' to quit.\n");
    printf("Power-cycle a motor to trigger fault detection.\n\n");

    auto start = std::chrono::steady_clock::now();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();

        // Print state for both motors
        for (uint8_t id : {42, 127}) {
            MotorState s;
            bool got = bus.getState(id, s);
            if (!got) {
                printf("[%.1fs] Motor %3d: not registered\n", elapsed, id);
                continue;
            }

            double age_ms = std::chrono::duration<double, std::milli>(
                now - s.last_update).count();

            printf("[%.1fs] Motor %3d | angle=%+6.3f rad | vel=%+6.3f | "
                   "temp=%.1fC | fault=0x%02X | faulted=%s | age=%.1fms\n",
                elapsed, id,
                s.angle, s.velocity, s.temperature, s.fault,
                bus.isFaulted(id) ? "YES" : "no",
                age_ms);
        }
        printf("\n");

        // Check for stale timestamps (>50ms = something is wrong)
        for (uint8_t id : {42, 127}) {
            MotorState s;
            if (!bus.getState(id, s)) continue;
            double age_ms = std::chrono::duration<double, std::milli>(
                now - s.last_update).count();
            if (age_ms > 50.0) {
                printf("  WARNING: Motor %d state is stale (%.1fms) — loop may be stuck\n",
                       id, age_ms);
            }
        }

        int ch = kbhit();
        if (ch == 'q' || ch == 'Q') break;
        if (ch == 'c' || ch == 'C') {
            printf(">>> clearFault() on both motors\n");
            bus.clearFault(42);
            bus.clearFault(127);
            // Re-enable after clearing
            bus.enable(42);
            bus.enable(127);
        }
    }

    printf("Stopping...\n");
    bus.stop();
    transport.close();
    return 0;
}
