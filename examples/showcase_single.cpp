// ---------------------------------------------------------------------------
// showcase_single.cpp
//
// Exercises one motor through a scripted sequence to validate the library:
//
//   1. Scan and auto-select motor
//   2. Sine sweep — smooth oscillation, validates feedback tracking
//   3. Step response — hard position step, validates kp/kd clamping
//   4. Velocity hold — spin at constant velocity, validates vel normalization
//   5. Torque pulse — brief feedforward torque, validates torque path
//   6. Return to zero — back to start angle
//   7. Emergency stop — validates estop path
//
// Prints a live data line every cycle so you can watch for:
//   - angle tracking error (should close within a few cycles)
//   - velocity reading sign (should match direction of movement)
//   - temperature climb (should stay under 60C in a short run)
//   - fault byte (must stay 0x00)
//
// Usage: ./showcase_single [motor_id]   (default: auto-scan)
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <cstdlib>

static MotorBus* g_bus = nullptr;

void handle_signal(int sig) {
    (void)sig;
    // Async-signal-safe — only touches atomics inside
    if (g_bus) g_bus->emergencyStop();
}

// Print one data line and flag anything suspicious
void printLine(const char* phase, float target, const MotorState& s) {
    float err = target - s.angle;
    const char* warn = "";
    if (s.fault != 0)          warn = " [FAULT]";
    else if (s.temperature > 60.0f) warn = " [HOT]";
    else if (fabsf(err) > 2.0f)    warn = " [TRACKING ERR]";

    printf("%-16s  tgt=%6.3f  ang=%6.3f  err=%6.3f  vel=%6.3f  trq=%5.2f  T=%.1fC%s\n",
        phase, target, s.angle, err, s.velocity, s.torque, s.temperature, warn);
}

int main(int argc, char** argv) {
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

    // --- Discover motor ---
    uint8_t motor_id = 0;
    if (argc >= 2) {
        motor_id = static_cast<uint8_t>(atoi(argv[1]));
        bus.addMotor(motor_id, cfg);
        printf("Using motor ID %d from argument.\n", motor_id);
    } else {
        printf("Scanning for motors...\n");
        auto found = bus.scanAndAdd(1, 127, cfg);
        if (found.empty()) {
            fprintf(stderr, "No motors found.\n");
            return 1;
        }
        motor_id = found[0];
        printf("Found motor ID %d — using it.\n\n", motor_id);
    }

    // --- Enable and get starting position ---
    printf("Enabling motor %d...\n", motor_id);
    if (!bus.enable(motor_id)) {
        fprintf(stderr, "Enable failed.\n");
        return 1;
    }

    MotorState state;
    bus.getState(motor_id, state);
    float origin = state.angle;
    printf("Start angle: %.3f rad\n\n", origin);

    bus.start();
    printf("Control loop running. Ctrl+C triggers emergency stop.\n\n");

    // Helper: run a phase for N cycles at 100Hz, printing every cycle
    // Returns false if estop fired
    auto runPhase = [&](const char* name, int cycles,
                        std::function<MotorTarget(int)> target_fn) -> bool {

        for (int i = 0; i < cycles; ++i) {
            if (!bus.isRunning()) return false;
            MotorTarget t = target_fn(i);
            bus.setTarget(motor_id, t);
            usleep(10000); // 100Hz — slower than the 200Hz loop, that's fine
            MotorState s;
            bus.getState(motor_id, s);
            printLine(name, t.angle, s);
        }
        return bus.isRunning();
    };

    // -----------------------------------------------------------------------
    // Phase 1 — Sine sweep ±0.8 rad around origin, 2 Hz, 3 seconds
    // Tests: smooth angle tracking, feedback sign, velocity reading
    // -----------------------------------------------------------------------
    printf("=== Phase 1: Sine sweep +-0.8 rad @ 2Hz ===\n");
    bool ok = runPhase("sine_sweep", 300, [&](int i) {
        MotorTarget t;
        t.angle    = origin + 0.8f * sinf(2.0f * M_PI * 2.0f * (i / 100.0f));
        t.velocity = 0.0f;
        t.kp       = 50.0f;
        t.kd       = 2.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // -----------------------------------------------------------------------
    // Phase 2 — Step response: jump +1.0 rad, hold 2s
    // Tests: kp/kd clamping, step settling, tracking error convergence
    // NOTE: MIT mode is PD only — expect ~0.1-0.4 rad steady-state error due to
    // static friction with no integrator. This is normal. Use position mode
    // (PARAM_LOC_REF, item 7) for zero steady-state error.
    // -----------------------------------------------------------------------
    printf("\n=== Phase 2: Step +1.0 rad, hold 2s ===\n");
    ok = runPhase("step_hold", 200, [&](int) {
        MotorTarget t;
        t.angle    = origin + 1.0f;
        t.velocity = 0.0f;
        t.kp       = 80.0f;
        t.kd       = 3.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // Phase 3 — Angle ramp at +2 rad/s for 2s
    // MIT velocity field is feedforward only — true velocity setpoint needs speed mode.
    // We simulate velocity control by advancing the angle target each cycle.
    // poll interval = 10ms, so delta_angle = vel * 0.010
    // Tests: continuous angle tracking, velocity feedback reading during motion
    printf("\n=== Phase 3: Angle ramp at +2 rad/s (2s) ===\n");
    {
        MotorState ramp_state;
        bus.getState(motor_id, ramp_state);
        float ramp_angle = ramp_state.angle;   // start from actual current position
        const float ramp_vel   = 2.0f;         // rad/s
        const float ramp_dt    = 0.010f;       // 10ms poll interval
        const float ramp_limit = ramp_angle + ramp_vel * 2.0f;  // 2 seconds worth

        ok = runPhase("angle_ramp", 200, [&](int) {
            ramp_angle += ramp_vel * ramp_dt;
            if (ramp_angle > ramp_limit) ramp_angle = ramp_limit;
            MotorTarget t;
            t.angle    = ramp_angle;
            t.velocity = ramp_vel;   // feedforward helps tracking
            t.kp       = 40.0f;
            t.kd       = 2.0f;
            t.torque   = 0.0f;
            return t;
        });
    }
    if (!ok) goto done;

    // -----------------------------------------------------------------------
    // Phase 4 — Torque pulse: 2Nm feedforward for 0.5s, position held at origin
    // Tests: torque normalization, torque feedback reading
    // -----------------------------------------------------------------------
    printf("\n=== Phase 4: Torque pulse 2Nm @ origin ===\n");
    ok = runPhase("torque_pulse", 50, [&](int) {
        MotorTarget t;
        t.angle    = origin;
        t.velocity = 0.0f;
        t.kp       = 30.0f;
        t.kd       = 2.0f;
        t.torque   = 2.0f;
        return t;
    });
    if (!ok) goto done;

    // -----------------------------------------------------------------------
    // Phase 5 — Return to origin
    // -----------------------------------------------------------------------
    printf("\n=== Phase 5: Return to origin ===\n");
    ok = runPhase("return", 200, [&](int) {
        MotorTarget t;
        t.angle    = origin;
        t.velocity = 0.0f;
        t.kp       = 80.0f;
        t.kd       = 3.0f;
        t.torque   = 0.0f;
        return t;
    });
    if (!ok) goto done;

    // -----------------------------------------------------------------------
    // Phase 6 — Emergency stop test
    // -----------------------------------------------------------------------
    printf("\n=== Phase 6: Emergency stop ===\n");
    printf("Triggering emergencyStop() now...\n");
    bus.emergencyStop();
    // Give the loop one cycle to process it
    usleep(10000);

done:
    printf("\n--- Run complete ---\n");
    {
        MotorState s;
        bus.getState(motor_id, s);
        printf("Final state: angle=%.3f  vel=%.3f  torque=%.3f  temp=%.1fC  fault=0x%02X\n",
            s.angle, s.velocity, s.torque, s.temperature, s.fault);
    }

    // Join the thread (emergencyStop already set running_=false)
    // ~MotorBus() handles this, but be explicit for clarity
    if (bus.isRunning()) bus.stop();

    transport.close();
    return 0;
}
