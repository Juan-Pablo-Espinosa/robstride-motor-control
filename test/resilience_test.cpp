// resilience_test.cpp
// Test: unplug a motor mid-run, confirm loop stays at 200Hz and surviving motor keeps moving
//
// Instructions:
//   1. Run this — both motors will sine sweep
//   2. While running, unplug motor 127's CAN connector
//   3. Confirm: loop rate stays 200Hz, motor 42 keeps moving, 127 goes stale
//   4. Replug 127 — it should NOT recover (expected, needs re-enable)

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>

static MotorBus* g_bus = nullptr;
void handle_signal(int) { if (g_bus) g_bus->emergencyStop(); }

int main() {
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    SocketCANTransport transport("can1");
    if (!transport.open()) { fprintf(stderr, "Failed to open can1\n"); return 1; }

    MotorConfig cfg03;
    cfg03.model            = MotorModel::RS03;
    cfg03.max_acceleration = 5.0f;
    cfg03.resolve();

    MotorConfig cfg04;
    cfg04.model            = MotorModel::RS04;
    cfg04.max_acceleration = 5.0f;
    cfg04.resolve();

    MotorBus bus(transport);
    g_bus = &bus;

    bus.addMotor(42,  cfg03);
    bus.addMotor(127, cfg04);

    printf("Enabling motors...\n");
    bus.enableAll();
    bus.start();

    printf("Running. Unplug motor 127 CAN while this is running.\n");
    printf("%-6s  %-8s %-8s %-8s  %-8s %-8s %-8s  %-6s  %s\n",
           "t(s)", "42_ang", "42_vel", "42_trq", "127_ang", "127_vel", "127_trq", "Hz", "staleness_127(ms)");

    auto start = std::chrono::steady_clock::now();
    int cycle = 0;

    while (bus.isRunning()) {
        float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();

        // Sine sweep both motors around their current position
        MotorTarget t42, t127;
        t42.angle  = 0.0f + 1.0f * sinf(2.0f * M_PI * 0.5f * t);
        t42.kp = 80.0f; t42.kd = 3.0f;
        t127.angle = 0.0f + 1.0f * sinf(2.0f * M_PI * 0.5f * t);
        t127.kp = 200.0f; t127.kd = 10.0f;

        bus.setTarget(42,  t42);
        bus.setTarget(127, t127);

        if (cycle % 20 == 0) {
            MotorState s42, s127;
            bus.getState(42,  s42);
            bus.getState(127, s127);

            // Staleness = how long since last update from 127
            auto now = std::chrono::steady_clock::now();
            float stale_ms = std::chrono::duration<float, std::milli>(now - s127.last_update).count();

            printf("%-6.1f  %-8.3f %-8.3f %-8.2f  %-8.3f %-8.3f %-8.2f  %-6.1f  %.0f ms%s\n",
                   t,
                   s42.angle,  s42.velocity,  s42.torque,
                   s127.angle, s127.velocity, s127.torque,
                   bus.measuredHz(),
                   stale_ms,
                   stale_ms > 100.0f ? "  <-- STALE" : "");
        }

        usleep(5000);
        cycle++;
    }

    printf("\nDone. Loop exited cleanly.\n");
    if (bus.isRunning()) bus.stop();
    transport.close();
    return 0;
}
