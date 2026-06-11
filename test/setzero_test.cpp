// setzero_test.cpp
// Validates bus.setZero(id) — commits current position as zero in motor flash.
//
// WARNING: this permanently changes the motor's zero position.
// Run with motor in a known good position.
//
// Usage: ./setzero_test <motor_id>
// Example: ./setzero_test 42

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <cmath>
#include <cstdlib>

static MotorBus* g_bus = nullptr;
void handle_signal(int) { if (g_bus) g_bus->emergencyStop(); }

static int passed = 0;
static int failed = 0;

void check(const char* name, bool result) {
    if (result) { printf("  [PASS] %s\n", name); passed++; }
    else        { printf("  [FAIL] %s\n", name); failed++; }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: ./setzero_test <motor_id>\n");
        fprintf(stderr, "Example: ./setzero_test 42\n");
        return 1;
    }

    uint8_t motor_id = static_cast<uint8_t>(atoi(argv[1]));
    printf("\nTarget motor ID: %d\n", motor_id);

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    SocketCANTransport transport("can1");
    if (!transport.open()) { fprintf(stderr, "Failed to open can1\n"); return 1; }

    // Pick config based on motor ID — extend this as you add more motors
    MotorConfig cfg;
    if (motor_id == 127) {
        cfg.model = MotorModel::RS04;
    } else {
        cfg.model = MotorModel::RS03;
    }
    cfg.max_acceleration = 5.0f;
    cfg.resolve();

    printf("\n=== Test 4: setZero (motor %d) ===\n\n", motor_id);

    MotorBus bus(transport);
    g_bus = &bus;
    bus.addMotor(motor_id, cfg);
    bus.enable(motor_id);
    usleep(50000);

    MotorState before;
    bus.getState(motor_id, before);
    printf("  Motor %d position before setZero: %.4f rad\n", motor_id, before.angle);

    // Confirm before committing
    printf("\n  WARNING: this will permanently set %.4f rad as the new zero.\n", before.angle);
    printf("  Press Enter to continue, Ctrl+C to abort...\n");
    getchar();

    // --- Call setZero ---
    printf("  Calling setZero on motor %d...\n", motor_id);
    bool ok = bus.setZero(motor_id);
    check("setZero() returns true", ok);
    usleep(100000); // 100ms — motor needs time to commit to flash

    // --- Read back position ---
    MotorState after;
    bus.getState(motor_id, after);
    printf("  Motor %d position after setZero:  %.4f rad\n", motor_id, after.angle);
    check("motor reports ~0.0 rad after setZero", fabsf(after.angle) < 0.1f);

    // --- Start loop and verify motor still responds ---
    bus.start();
    usleep(50000);

    MotorTarget t;
    t.angle  = 0.0f;
    t.kp     = 80.0f;
    t.kd     = 3.0f;
    bus.setTarget(motor_id, t);
    usleep(200000);

    MotorState running;
    bus.getState(motor_id, running);
    printf("  Motor %d holding zero: %.4f rad  temp=%.1fC  fault=0x%02X\n",
           motor_id, running.angle, running.temperature, running.fault);

    check("motor responds after setZero — temp valid", running.temperature > 0.0f);
    check("no fault after setZero", running.fault == 0);

    bus.stop();
    g_bus = nullptr;

    printf("\n=== Results: %d passed, %d failed ===\n\n", passed, failed);
    transport.close();
    return failed > 0 ? 1 : 0;
}
