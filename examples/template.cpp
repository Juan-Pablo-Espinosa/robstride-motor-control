// =============================================================================
// template.cpp — GR-0X Motor Control Starter Template
//
// Use this file as the starting point for any motor control program.
// It demonstrates every major feature of the robstride-motor-control library:
//
//   - Defining multiple motors with individual configs (kp, kd, accel, limits)
//   - Adding motors manually by ID or auto-scanning
//   - Enabling/disabling individual motors or all at once
//   - Setting targets individually or in a synchronized group
//   - Reading back state (angle, velocity, torque, temp, fault)
//   - Running a custom control loop alongside the 200Hz bus thread
//   - Emergency stop from signal handler
//   - Clean shutdown
//
// HOW TO USE:
//   1. Define your motors in the MOTOR DEFINITIONS section
//   2. Write your control logic in the CONTROL LOOP section
//   3. Build: cd ~/gr0x-motor/build && make -j4
//   4. Run:   ./template
//
// For a ROS node, replace the control loop with your ROS callbacks.
// setTarget() and getAllStates() are thread-safe — call them from any thread.
// =============================================================================

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>

// =============================================================================
// GLOBAL BUS HANDLE — needed for signal handler
// =============================================================================
static MotorBus* g_bus = nullptr;

void handle_signal(int) {
    if (g_bus) g_bus->emergencyStop();
}

// =============================================================================
// MOTOR DEFINITIONS
//
// Define one MotorConfig per joint. Set:
//   model            — RS00 through RS06
//   joint_name       — human-readable label for logs
//   min_angle        — soft lower limit (rad)
//   max_angle        — soft upper limit (rad)
//   max_velocity     — soft velocity limit (rad/s), -1 = use hardware max
//   max_torque       — soft torque limit (Nm),      -1 = use hardware max
//   max_acceleration — ramp limit (rad/s²),         -1 = unlimited
//   invert_direction — true for mirrored joints (e.g. left vs right)
//
// After filling in the fields, call cfg.resolve() — this collapses software
// limits against hardware limits and must be called before use.
// =============================================================================

// --- Helper: degrees to radians ---
static constexpr float DEG = 3.14159265f / 180.0f;

MotorConfig makeKneeRight() {
    MotorConfig cfg;
    cfg.model            = MotorModel::RS03;
    cfg.joint_name       = "knee_right";
    cfg.min_angle        = 0.0f;          // 0° = fully extended
    cfg.max_angle        = 120.0f * DEG;  // 120° max flexion
    cfg.max_velocity     = 10.0f;         // rad/s
    cfg.max_torque       = 40.0f;         // Nm
    cfg.max_acceleration = 5.0f;          // rad/s² — smooth ramp
    cfg.invert_direction = false;
    cfg.resolve();
    return cfg;
}

MotorConfig makeKneeLeft() {
    MotorConfig cfg;
    cfg.model            = MotorModel::RS03;
    cfg.joint_name       = "knee_left";
    cfg.min_angle        = 0.0f;
    cfg.max_angle        = 120.0f * DEG;
    cfg.max_velocity     = 10.0f;
    cfg.max_torque       = 40.0f;
    cfg.max_acceleration = 5.0f;
    cfg.invert_direction = true;          // mirrored mounting
    cfg.resolve();
    return cfg;
}

MotorConfig makeHipRight() {
    MotorConfig cfg;
    cfg.model            = MotorModel::RS04;
    cfg.joint_name       = "hip_right";
    cfg.min_angle        = -30.0f * DEG;
    cfg.max_angle        =  90.0f * DEG;
    cfg.max_velocity     = 8.0f;
    cfg.max_torque       = 80.0f;
    cfg.max_acceleration = 4.0f;
    cfg.invert_direction = false;
    cfg.resolve();
    return cfg;
}

// Add more joints here following the same pattern...

// =============================================================================
// PD GAINS
//
// Define kp/kd per joint type. These go into MotorTarget, not MotorConfig.
// Tune these for your robot's mass and geometry.
//
// Rule of thumb for RobStride RS-03:
//   Standing/weight-bearing:  kp=150-200, kd=10-20
//   Swing/free motion:        kp=60-80,   kd=5-8
//   Compliant/backdrivable:   kp=20-40,   kd=3-5
// =============================================================================
static constexpr float KP_STANCE = 180.0f;
static constexpr float KD_STANCE =  15.0f;
static constexpr float KP_SWING  =  70.0f;
static constexpr float KD_SWING  =   6.0f;

// =============================================================================
// MOTOR ID MAP
//
// Map joint names to CAN IDs. Scan once to find IDs:
//   cd ~/gr0x-motor/build && ./scan_test
// Then fill these in. Factory default ID is 127.
// =============================================================================
static constexpr uint8_t ID_KNEE_RIGHT =  42;
static constexpr uint8_t ID_KNEE_LEFT  =  43;  // example — change to real ID
static constexpr uint8_t ID_HIP_RIGHT  =  44;  // example — change to real ID

// =============================================================================
// HELPER: build a MotorTarget
// =============================================================================
MotorTarget makeTarget(float angle, float kp, float kd,
                       float vel = 0.0f, float torque = 0.0f) {
    MotorTarget t;
    t.angle    = angle;
    t.velocity = vel;
    t.kp       = kp;
    t.kd       = kd;
    t.torque   = torque;
    return t;
}

// =============================================================================
// HELPER: print state for one motor
// =============================================================================
void printState(const char* name, uint8_t id, MotorBus& bus) {
    MotorState s;
    if (!bus.getState(id, s)) {
        printf("%-14s [%3d]  NOT FOUND\n", name, id);
        return;
    }
    printf("%-14s [%3d]  ang=%7.3f  vel=%6.2f  trq=%5.2f  T=%.1fC  fault=0x%02X%s\n",
        name, id,
        s.angle, s.velocity, s.torque, s.temperature, s.fault,
        s.fault ? " [FAULT]" : "");
}

// =============================================================================
// HELPER: blocking move — set target and wait until error < threshold or timeout
//
// Use this for sequential scripted moves.
// For a ROS node, use setTarget() directly from your callback instead.
// =============================================================================
bool waitForPosition(MotorBus& bus, uint8_t id, float target,
                     float kp, float kd,
                     float threshold_rad = 0.05f, int timeout_ms = 5000) {
    auto start = std::chrono::steady_clock::now();
    MotorTarget t = makeTarget(target, kp, kd);

    while (bus.isRunning()) {
        bus.setTarget(id, t);

        MotorState s;
        bus.getState(id, s);

        if (s.fault != 0) {
            fprintf(stderr, "Motor %d faulted during move: 0x%02X\n", id, s.fault);
            return false;
        }

        float err = fabsf(s.angle - target);
        if (err < threshold_rad) return true;

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms) {
            fprintf(stderr, "Motor %d move timeout — err=%.3f rad\n", id, err);
            return false;
        }

        usleep(5000);  // 5ms poll
    }
    return false;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    // -------------------------------------------------------------------------
    // 1. Open transport
    //    Use "can1" for J1/CAN-A (motors).
    //    Use "can0" for J2/CAN-B (second bus, nothing connected yet).
    // -------------------------------------------------------------------------
    SocketCANTransport transport("can1");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1. Run: sudo ~/gr0x-motor/setup.sh\n");
        return 1;
    }

    // -------------------------------------------------------------------------
    // 2. Create bus at 200Hz
    // -------------------------------------------------------------------------
    MotorBus bus(transport, 200);
    g_bus = &bus;

    // -------------------------------------------------------------------------
    // 3. Add motors
    //
    // Option A — add by known ID (faster, use in production):
    //   bus.addMotor(ID_KNEE_RIGHT, makeKneeRight());
    //   bus.addMotor(ID_KNEE_LEFT,  makeKneeLeft());
    //   bus.addMotor(ID_HIP_RIGHT,  makeHipRight());
    //
    // Option B — auto-scan (slower ~2s, use for first bringup):
    //   auto found = bus.scanAndAdd(1, 127, makeKneeRight());
    //
    // Use Option A once you know your IDs.
    // -------------------------------------------------------------------------
    bus.addMotor(ID_KNEE_RIGHT, makeKneeRight());
    // bus.addMotor(ID_KNEE_LEFT,  makeKneeLeft());   // uncomment when wired
    // bus.addMotor(ID_HIP_RIGHT,  makeHipRight());   // uncomment when wired

    // -------------------------------------------------------------------------
    // 4. Enable motors
    //
    // enableAll() arms every motor in the bus.
    // enable(id) arms a single motor.
    // After enable, getState() is immediately valid (seeded from real position).
    // -------------------------------------------------------------------------
    printf("Enabling motors...\n");
    bus.enable(ID_KNEE_RIGHT);
    // bus.enableAll();

    // Print initial states
    printState("knee_right", ID_KNEE_RIGHT, bus);

    // -------------------------------------------------------------------------
    // 5. Start the 200Hz control thread
    //
    // After start(), the bus sends MIT frames to all enabled motors every 5ms.
    // Use setTarget() from any thread to update commands.
    // Use getAllStates() to read back all joint states atomically.
    // -------------------------------------------------------------------------
    bus.start();
    printf("Bus running at %d Hz\n\n", bus.getHz());

    // =========================================================================
    // CONTROL LOOP
    //
    // Everything below here is your application logic.
    // Replace this section with your own code.
    //
    // For a ROS node:
    //   - Joint commands arrive in a ROS callback → call bus.setTarget()
    //   - Joint states are published in a timer → call bus.getAllStates()
    //   - On shutdown → call bus.emergencyStop()
    //
    // For a scripted sequence:
    //   - Use waitForPosition() for blocking moves
    //   - Use setTarget() + sleep for non-blocking moves
    //   - Use getAllStates() to log or check all joints at once
    // =========================================================================

    printf("=== Starting control sequence ===\n\n");

    // --- Example 1: move to zero and hold ---
    printf("Moving to zero (fully extended)...\n");
    waitForPosition(bus, ID_KNEE_RIGHT, 0.0f, KP_STANCE, KD_STANCE);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printState("knee_right", ID_KNEE_RIGHT, bus);

    // --- Example 2: move to 45° ---
    printf("\nMoving to 45 degrees...\n");
    waitForPosition(bus, ID_KNEE_RIGHT, 45.0f * DEG, KP_SWING, KD_SWING);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printState("knee_right", ID_KNEE_RIGHT, bus);

    // --- Example 3: move to 90° ---
    printf("\nMoving to 90 degrees...\n");
    waitForPosition(bus, ID_KNEE_RIGHT, 90.0f * DEG, KP_SWING, KD_SWING);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printState("knee_right", ID_KNEE_RIGHT, bus);

    // --- Example 4: synchronized multi-motor command ---
    // When you have multiple motors, set all targets before the bus
    // reads them — getAllStates() and setTarget() are both thread-safe.
    printf("\nSynchronized command to all joints...\n");
    bus.setTarget(ID_KNEE_RIGHT, makeTarget(45.0f * DEG, KP_SWING, KD_SWING));
    // bus.setTarget(ID_KNEE_LEFT,  makeTarget(45.0f * DEG, KP_SWING, KD_SWING));
    // bus.setTarget(ID_HIP_RIGHT,  makeTarget(20.0f * DEG, KP_STANCE, KD_STANCE));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- Example 5: read all states at once ---
    printf("\nAll joint states:\n");
    auto states = bus.getAllStates();
    for (auto& [id, s] : states) {
        printf("  [%3d]  ang=%7.3f rad (%6.2f deg)  vel=%6.2f  trq=%5.2f  fault=0x%02X\n",
            id, s.angle, s.angle / DEG, s.velocity, s.torque, s.fault);
    }

    // --- Example 6: return to zero ---
    printf("\nReturning to zero...\n");
    waitForPosition(bus, ID_KNEE_RIGHT, 0.0f, KP_STANCE, KD_STANCE);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // =========================================================================
    // END OF CONTROL LOOP
    // =========================================================================

    printf("\n=== Sequence complete ===\n");
    printState("knee_right", ID_KNEE_RIGHT, bus);

    // -------------------------------------------------------------------------
    // 6. Shutdown
    //
    // stop()         — graceful: joins thread, then disables all motors
    // emergencyStop()— async-safe: sets atomics only, loop handles disable
    //                  call this from signal handlers or on critical fault
    // -------------------------------------------------------------------------
    bus.stop();
    transport.close();
    printf("Done.\n");
    return 0;
}
