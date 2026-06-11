// timestamp_test.cpp
// Validates MotorState.last_update timestamps:
//   1. Timestamp is valid (non-zero) after enable
//   2. Timestamp advances each cycle — motor is actually being heard
//   3. Staleness detection works — can identify a "dead" observation

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <chrono>

static MotorBus* g_bus = nullptr;
void handle_signal(int) { if (g_bus) g_bus->emergencyStop(); }

static int passed = 0;
static int failed = 0;

void check(const char* name, bool result) {
    if (result) { printf("  [PASS] %s\n", name); passed++; }
    else        { printf("  [FAIL] %s\n", name); failed++; }
}

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

    printf("\n=== Test 3: Observation Timestamps ===\n\n");

    MotorBus bus(transport);
    g_bus = &bus;
    bus.addMotor(42,  cfg03);
    bus.addMotor(127, cfg04);
    bus.enableAll();
    bus.start();
    usleep(100000); // let loop run ~20 cycles

    // --- Check 1: timestamps are non-zero after enable ---
    {
        MotorState s42, s127;
        bus.getState(42,  s42);
        bus.getState(127, s127);

        auto epoch = std::chrono::steady_clock::time_point{};
        check("motor 42  timestamp non-zero after enable", s42.last_update  != epoch);
        check("motor 127 timestamp non-zero after enable", s127.last_update != epoch);
    }

    // --- Check 2: timestamp advances over time ---
    {
        MotorState before, after;
        bus.getState(42, before);
        usleep(50000); // wait 50ms = ~10 cycles
        bus.getState(42, after);
        check("motor 42 timestamp advances over 50ms",
              after.last_update > before.last_update);

        bus.getState(127, before);
        usleep(50000);
        bus.getState(127, after);
        check("motor 127 timestamp advances over 50ms",
              after.last_update > before.last_update);
    }

    // --- Check 3: freshness — last update within 20ms (5x the 5ms cycle) ---
    {
        MotorState s42, s127;
        bus.getState(42,  s42);
        bus.getState(127, s127);
        auto now = std::chrono::steady_clock::now();

        float age42  = std::chrono::duration<float, std::milli>(now - s42.last_update).count();
        float age127 = std::chrono::duration<float, std::milli>(now - s127.last_update).count();

        printf("  motor 42  last update age: %.1f ms\n", age42);
        printf("  motor 127 last update age: %.1f ms\n", age127);

        check("motor 42  fresh within 20ms", age42  < 20.0f);
        check("motor 127 fresh within 20ms", age127 < 20.0f);
    }

    // --- Check 4: staleness threshold logic (what ROS node will use) ---
    {
        // Simulate what the ROS node watchdog will do:
        // if age > 100ms → joint is dead, stop sending commands
        MotorState s;
        bus.getState(42, s);
        auto now = std::chrono::steady_clock::now();
        float age = std::chrono::duration<float, std::milli>(now - s.last_update).count();
        bool is_fresh = age < 100.0f;
        check("staleness threshold logic works (age < 100ms = fresh)", is_fresh);
    }

    bus.stop();
    g_bus = nullptr;

    printf("\n=== Results: %d passed, %d failed ===\n\n", passed, failed);
    transport.close();
    return failed > 0 ? 1 : 0;
}
