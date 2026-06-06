// ---------------------------------------------------------------------------
// control_test.cpp
// Milestone 3: enable motor, command a small position move, return, disable.
//
// Safety parameters:
//   kp=10.0  (low stiffness — won't snap to position aggressively)
//   kd=0.5   (light damping)
//   torque=0 (no feedforward)
//   move=0.5 rad (~28 degrees)
//
// Hold the motor shaft firmly before running.
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "robstride_motor.hpp"
#include <cstdio>
#include <csignal>
#include <unistd.h>

static volatile bool g_running = true;
void handle_sigint(int) { g_running = false; }

int main() {
    signal(SIGINT, handle_sigint);

    SocketCANTransport transport("can1");

    printf("Opening can1...\n");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1.\n");
        return 1;
    }

    RobstrideMotor motor(transport, 42);

    // --- Enable ---
    printf("Enabling motor...\n");
    if (!motor.enable()) {
        fprintf(stderr, "Enable failed.\n");
        transport.close();
        return 1;
    }
    printf("Motor enabled.\n");

    // --- Read starting position ---
    MotorState state;
    if (!motor.requestFeedback(state)) {
        fprintf(stderr, "Could not read initial position.\n");
        motor.disable();
        transport.close();
        return 1;
    }

    float start_angle  = state.angle;
    float target_angle = start_angle + 0.5f;  // 0.5 rad forward
    printf("Start angle:  %.3f rad\n", start_angle);
    printf("Target angle: %.3f rad\n\n", target_angle);

    // --- Move to target_angle ---
    printf("Moving to target (hold shaft)...\n");
    for (int i = 0; i < 200 && g_running; i++) {
        motor.sendMIT(target_angle, 0.0f, 10.0f, 0.5f, 0.0f);

        if (motor.requestFeedback(state)) {
            printf("angle=%7.3f rad  vel=%6.3f rad/s  torque=%6.3f Nm  temp=%.1fC\n",
                state.angle, state.velocity, state.torque, state.temperature);
        }

        usleep(10000);  // 100Hz
    }

    if (!g_running) goto shutdown;

    // --- Hold for 2 seconds ---
    printf("\nHolding at target...\n");
    for (int i = 0; i < 200 && g_running; i++) {
        motor.sendMIT(target_angle, 0.0f, 10.0f, 0.5f, 0.0f);
        motor.requestFeedback(state);
        usleep(10000);
    }

    if (!g_running) goto shutdown;

    // --- Return to start ---
    printf("\nReturning to start angle...\n");
    for (int i = 0; i < 200 && g_running; i++) {
        motor.sendMIT(start_angle, 0.0f, 10.0f, 0.5f, 0.0f);

        if (motor.requestFeedback(state)) {
            printf("angle=%7.3f rad  vel=%6.3f rad/s  torque=%6.3f Nm\n",
                state.angle, state.velocity, state.torque);
        }

        usleep(10000);
    }

shutdown:
    printf("\nDisabling motor...\n");
    motor.disable();
    transport.close();
    printf("Done.\n");
    return 0;
}
