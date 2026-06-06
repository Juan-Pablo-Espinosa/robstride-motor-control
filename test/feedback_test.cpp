// ---------------------------------------------------------------------------
// feedback_test.cpp
// Milestone 2: read position, velocity, torque, temperature from motor ID 42
//
// Expected output:
//   Opening can1...
//   Enabling motor...
//   Reading feedback (press Ctrl+C to stop)...
//   angle=0.000 rad  vel=0.000 rad/s  torque=0.000 Nm  temp=35C  mode=Run
//   ...
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "robstride_motor.hpp"
#include <cstdio>
#include <csignal>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Clean shutdown on Ctrl+C — disable motor before exit
// ---------------------------------------------------------------------------
static volatile bool g_running = true;
void handle_sigint(int) { g_running = false; }

const char* modeString(MotorMode m) {
    switch (m) {
        case MotorMode::Reset:       return "Reset";
        case MotorMode::Calibration: return "Calibration";
        case MotorMode::Run:         return "Run";
        default:                     return "Unknown";
    }
}

int main() {
    signal(SIGINT, handle_sigint);

    SocketCANTransport transport("can1");

    printf("Opening can1...\n");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1.\n");
        return 1;
    }

    // RS-03 confirmed on ID 42
    RobstrideMotor motor(transport, 42);

    printf("Enabling motor...\n");
    if (!motor.enable()) {
        fprintf(stderr, "Enable failed — no feedback received.\n");
        fprintf(stderr, "Check motor power and CAN connection.\n");
        transport.close();
        return 1;
    }
    printf("Motor enabled.\n\n");

    printf("Reading feedback (press Ctrl+C to stop)...\n");

    while (g_running) {
        MotorState state;
        if (motor.requestFeedback(state)) {
            printf("angle=%7.3f rad  vel=%7.3f rad/s  torque=%7.3f Nm  temp=%5.1fC  fault=0x%02X  mode=%s\n",
                state.angle,
                state.velocity,
                state.torque,
                state.temperature,
                state.fault,
                modeString(state.mode));
        } else {
            fprintf(stderr, "Feedback timeout.\n");
        }

        // Poll at ~10Hz — enough to verify feedback before we go to 200Hz control
        usleep(100000);
    }

    printf("\nShutting down — disabling motor...\n");
    motor.disable();
    transport.close();
    printf("Done.\n");
    return 0;
}
