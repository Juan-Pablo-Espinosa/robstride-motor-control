// ---------------------------------------------------------------------------
// showcase_all.cpp
//
// Discovers all motors on can1 and runs them simultaneously through the same
// scripted sequence as showcase_single, but via MotorBus's control thread.
//
// This validates:
//   - scanAndAdd correctness
//   - per-motor state isolation (each motor tracks its own origin)
//   - control loop throughput under N motors (watch measured Hz)
//   - emergencyStop with multiple motors
//
// Usage: ./showcase_all
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <map>

static MotorBus* g_bus = nullptr;

void handle_signal(int sig) {
    (void)sig;
    if (g_bus) g_bus->emergencyStop();
}

void printAllStates(MotorBus& bus, const std::map<uint8_t, float>& origins,
                    const char* phase, int cycle) {
    auto states = bus.getAllStates();
    for (auto& [id, s] : states) {
        float origin = origins.count(id) ? origins.at(id) : 0.0f;
        float err = 0.0f; // tracking error vs origin for display
        (void)origin;
        const char* fault_str = s.fault != 0 ? " [FAULT]" : "";
        const char* hot_str   = s.temperature > 60.0f ? " [HOT]" : "";
        printf("  [%3d] %-14s cy=%4d  ang=%6.3f  vel=%6.3f  trq=%5.2f  T=%.1fC%s%s\n",
            id, phase, cycle,
            s.angle, s.velocity, s.torque,
            s.temperature, fault_str, hot_str);
        (void)err;
    }
}

int main() {
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    SocketCANTransport transport("can1");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1.\n");
        return 1;
    }

    MotorConfig cfg;
    cfg.model = MotorModel::RS03;
    cfg.resolve();

    MotorBus bus(transport);
    g_bus = &bus;

    // --- Discover all motors ---
    printf("Scanning for motors (1-127)...\n");
    auto found = bus.scanAndAdd(1, 127, cfg);

    if (found.empty()) {
        fprintf(stderr, "No motors found. Check power and CAN.\n");
        return 1;
    }

    printf("Found %zu motor(s): ", found.size());
    for (uint8_t id : found) printf("%d ", id);
    printf("\n\n");

    // --- Enable all and record starting positions ---
    printf("Enabling all motors...\n");
    bus.enableAll();
    usleep(50000); // let feedback settle

    std::map<uint8_t, float> origins;
    {
        auto states = bus.getAllStates();
        for (auto& [id, s] : states) {
            origins[id] = s.angle;
            printf("  Motor %d start angle: %.3f rad\n", id, s.angle);
        }
    }
    printf("\n");

    bus.start();
    printf("Control loop running at %dHz. Ctrl+C = emergency stop.\n\n", bus.getHz());

    // Helper: run all motors through a phase for N cycles
    auto runPhase = [&](const char* name, int cycles,
                        std::function<MotorTarget(uint8_t, int)> target_fn) -> bool {
        printf("=== %s ===\n", name);
        for (int i = 0; i < cycles; ++i) {
            if (!bus.isRunning()) return false;
            for (uint8_t id : found) {
                bus.setTarget(id, target_fn(id, i));
            }
            usleep(10000); // 100Hz poll, bus loop runs at 200Hz internally
            if (i % 20 == 0) { // print every 0.2s to avoid flooding
                printAllStates(bus, origins, name, i);
            }
        }
        return bus.isRunning();
    };

    bool ok = true;

    // Phase 1 — Sine sweep ±0.5 rad around each motor's own origin
    ok = runPhase("sine_sweep", 300, [&](uint8_t id, int i) {
        float org = origins.count(id) ? origins.at(id) : 0.0f;
        MotorTarget t;
        t.angle    = org + 0.5f * sinf(2.0f * M_PI * 1.0f * (i / 100.0f));
        t.velocity = 0.0f;
        t.kp       = 50.0f;
        t.kd       = 2.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // Phase 2 — Step +0.8 rad from each origin, hold 2s
    ok = runPhase("step_hold", 200, [&](uint8_t id, int) {
        float org = origins.count(id) ? origins.at(id) : 0.0f;
        MotorTarget t;
        t.angle    = org + 0.8f;
        t.velocity = 0.0f;
        t.kp       = 80.0f;
        t.kd       = 3.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // Phase 3 — Velocity hold +2 rad/s on all motors simultaneously
    // KEY TEST: measured Hz should stay near 200 with N motors
    ok = runPhase("vel_hold", 200, [&](uint8_t, int) {
        MotorTarget t;
        t.angle    = 0.0f;
        t.velocity = 2.0f;
        t.kp       = 0.0f;
        t.kd       = 5.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // Phase 4 — Return all to origin
    ok = runPhase("return", 200, [&](uint8_t id, int) {
        float org = origins.count(id) ? origins.at(id) : 0.0f;
        MotorTarget t;
        t.angle    = org;
        t.velocity = 0.0f;
        t.kp       = 80.0f;
        t.kd       = 3.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // Phase 5 — Emergency stop
    printf("\n=== emergency_stop ===\n");
    printf("Triggering emergencyStop() on all %zu motors...\n", found.size());
    bus.emergencyStop();
    usleep(10000);

done:
    printf("\n--- Run complete ---\n");
    printf("Measured loop rate: %.1f Hz\n", bus.measuredHz());
    {
        auto states = bus.getAllStates();
        for (auto& [id, s] : states) {
            printf("Motor %d final: angle=%.3f  temp=%.1fC  fault=0x%02X\n",
                id, s.angle, s.temperature, s.fault);
        }
    }

    if (bus.isRunning()) bus.stop();
    transport.close();
    return 0;
}

