// ---------------------------------------------------------------------------
// precision_demo.cpp
//
// Realistic knee joint motion test.
// Simulates a humanoid knee: small precise movements, hold periods,
// weight-bearing stiffness, and a return to zero.
//
// Configurable at the top:
//   HOLD_MS       — time to hold each position (ms)
//   JOINT_MIN     — minimum angle (rad), e.g. 0 = fully extended
//   JOINT_MAX     — maximum angle (rad), e.g. 2.09 = 120° flexion
//
// Usage: ./precision_demo [motor_id]   (default: auto-scan)
// ---------------------------------------------------------------------------
#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <cstdlib>

// ---------------------------------------------------------------------------
// CONFIGURE HERE
// ---------------------------------------------------------------------------
static constexpr int   HOLD_MS    = 4000;          // ms to hold each position
static constexpr float JOINT_MIN  = 0.0f;          // rad — fully extended knee
static constexpr float JOINT_MAX  = 2.094f;        // rad — 120° max flexion
static constexpr float KP         = 200.0f;         // position stiffness
static constexpr float KD         = 17.0f;          // damping
static constexpr float MAX_ACCEL  = 5.0f;          // rad/s² — smooth ramp
// ---------------------------------------------------------------------------

static constexpr float PI = 3.14159265358979f;

static MotorBus* g_bus = nullptr;
void handle_signal(int) {
    if (g_bus) g_bus->emergencyStop();
}

void printLine(const char* phase, float target, const MotorState& s) {
    float err = target - s.angle;
    const char* warn = "";
    if (s.fault != 0)             warn = " [FAULT]";
    else if (s.temperature > 60.0f) warn = " [HOT]";
    else if (fabsf(err) > 0.15f)  warn = " [ERR]";
    printf("%-22s  tgt=%6.3f  ang=%6.3f  err=%+6.3f  vel=%6.2f  trq=%5.2f  T=%.1fC%s\n",
        phase, target, s.angle, err, s.velocity, s.torque, s.temperature, warn);
}

bool moveTo(MotorBus& bus, uint8_t id, const char* label,
            float target, int hold_ms)
{
    // Clamp to joint range
    if (target < JOINT_MIN) target = JOINT_MIN;
    if (target > JOINT_MAX) target = JOINT_MAX;

    const int poll_us = 10000;   // 10ms
    const int cycles  = hold_ms / 10;

    MotorTarget t;
    t.angle    = target;
    t.velocity = 0.0f;
    t.kp       = KP;
    t.kd       = KD;
    t.torque   = 0.0f;

    for (int i = 0; i < cycles; i++) {
        if (!bus.isRunning()) return false;
        bus.setTarget(id, t);
        usleep(poll_us);

        if (i % 10 == 0) {
            MotorState s;
            bus.getState(id, s);
            printLine(label, target, s);
            if (s.fault != 0) {
                fprintf(stderr, "FAULT 0x%02X — aborting.\n", s.fault);
                return false;
            }
        }
    }
    return true;
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
    cfg.model            = MotorModel::RS03;
    cfg.max_acceleration = MAX_ACCEL;
    cfg.min_angle        = JOINT_MIN;
    cfg.max_angle        = JOINT_MAX;
    cfg.resolve();

    MotorBus bus(transport, 200);
    g_bus = &bus;

    // --- Find motor ---
    uint8_t motor_id = 0;
    if (argc >= 2) {
        motor_id = static_cast<uint8_t>(atoi(argv[1]));
        bus.addMotor(motor_id, cfg);
        printf("Using motor ID %d.\n", motor_id);
    } else {
        printf("Scanning for motors on can1...\n");
        auto found = bus.scanAndAdd(1, 127, cfg);
        if (found.empty()) {
            fprintf(stderr, "No motors found.\n");
            transport.close();
            return 1;
        }
        motor_id = found[0];
        printf("Found motor ID %d.\n", motor_id);
    }

    if (!bus.enable(motor_id)) {
        fprintf(stderr, "Enable failed.\n");
        transport.close();
        return 1;
    }

    bus.start();

    printf("\nJoint range: %.1f° to %.1f°\n",
        JOINT_MIN * 180.0f / PI, JOINT_MAX * 180.0f / PI);
    printf("Hold time:   %dms per position\n", HOLD_MS);
    printf("kp=%.0f  kd=%.1f  max_accel=%.1f rad/s²\n\n", KP, KD, MAX_ACCEL);

    // -----------------------------------------------------------------------
    // Knee joint sequence — realistic gait-like movements
    //
    // All angles relative to JOINT_MIN (fully extended = 0)
    // Typical walking knee ROM: 0-70° swing, 0-20° stance
    // -----------------------------------------------------------------------

    // deg to rad helper
    auto deg = [](float d) { return d * PI / 180.0f; };

    // Phase 0 — go to zero (fully extended, weight bearing)
    printf("=== Phase 0: Full extension (0°) — standing ===\n");
    if (!moveTo(bus, motor_id, "standing", JOINT_MIN, HOLD_MS)) goto done;

    // Phase 1 — slight flex for balance (10°)
    printf("\n=== Phase 1: Slight flex (10°) — balance ===\n");
    if (!moveTo(bus, motor_id, "balance_flex", deg(10), HOLD_MS)) goto done;

    // Phase 2 — mid stance flex (20°)
    printf("\n=== Phase 2: Stance flex (20°) ===\n");
    if (!moveTo(bus, motor_id, "stance_flex", deg(20), HOLD_MS)) goto done;

    // Phase 3 — return to extension
    printf("\n=== Phase 3: Return to extension (0°) ===\n");
    if (!moveTo(bus, motor_id, "extend", JOINT_MIN, HOLD_MS)) goto done;

    // Phase 4 — swing phase flex (65°)
    printf("\n=== Phase 4: Swing phase (65°) ===\n");
    if (!moveTo(bus, motor_id, "swing_65", deg(65), HOLD_MS)) goto done;

    // Phase 5 — peak swing (70°)
    printf("\n=== Phase 5: Peak swing (70°) ===\n");
    if (!moveTo(bus, motor_id, "swing_peak", deg(70), HOLD_MS)) goto done;

    // Phase 6 — swing to extension (landing prep, 10°)
    printf("\n=== Phase 6: Landing prep (10°) ===\n");
    if (!moveTo(bus, motor_id, "landing", deg(10), HOLD_MS)) goto done;

    // Phase 7 — full extension (heel strike)
    printf("\n=== Phase 7: Heel strike — full extension (0°) ===\n");
    if (!moveTo(bus, motor_id, "heel_strike", JOINT_MIN, HOLD_MS)) goto done;

    // Phase 8 — deep flex test (90°)
    printf("\n=== Phase 8: Deep flex (90°) ===\n");
    if (!moveTo(bus, motor_id, "deep_flex", deg(90), HOLD_MS)) goto done;

    // Phase 9 — back to zero
    printf("\n=== Phase 9: Return to zero ===\n");
    if (!moveTo(bus, motor_id, "return_zero", JOINT_MIN, HOLD_MS)) goto done;

done:
    printf("\n=== Done ===\n");
    {
        MotorState s;
        bus.getState(motor_id, s);
        printf("Final: angle=%.4f rad (%.2f°)  error from zero=%.4f rad (%.3f°)\n",
            s.angle, s.angle * 180.0f / PI,
            s.angle - JOINT_MIN,
            (s.angle - JOINT_MIN) * 180.0f / PI);
    }
    bus.stop();
    transport.close();
    return 0;
}
