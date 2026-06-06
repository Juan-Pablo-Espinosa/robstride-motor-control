#pragma once
#include "transport.hpp"
#include "robstride_motor.hpp"
#include <unordered_map>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>

// ---------------------------------------------------------------------------
// MotorTarget — what the RL policy writes each cycle
// ---------------------------------------------------------------------------
struct MotorTarget {
    float angle    = 0.0f;
    float velocity = 0.0f;
    float kp       = 10.0f;
    float kd       = 0.5f;
    float torque   = 0.0f;
};

// ---------------------------------------------------------------------------
// MotorEntry — internal per-motor state
// ---------------------------------------------------------------------------
struct MotorEntry {
    RobstrideMotor motor;
    MotorTarget    target;
    MotorState     state;
    bool           enabled = false;

    MotorEntry(Transport& transport, uint8_t id, const MotorConfig& config)
        : motor(transport, id, config) {}
};

// ---------------------------------------------------------------------------
// MotorBus
//
// Owns a transport and a set of motors. Runs a 200Hz control thread that
// sends MIT commands to all enabled motors and reads back their state.
//
// Thread safety:
//   setTarget()  — safe to call from any thread (RL policy / ROS node)
//   getState()   — safe to call from any thread
//   enable/disable — safe to call from UI thread
//   control loop — runs internally, do not call manually
// ---------------------------------------------------------------------------
class MotorBus {
public:
    // hz: control loop frequency (default 200Hz)
    explicit MotorBus(Transport& transport, int hz = 200);
    ~MotorBus();

    // --- Setup ---

    // Scan for motors in [start, end], add all found to the bus
    // Scan and add motors — all discovered motors use default_config
    // You can reconfigure individual motors after with addMotor()
    std::vector<uint8_t> scanAndAdd(uint8_t start = 1, uint8_t end = 127,
                                    const MotorConfig& default_config = MotorConfig{});

    // Manually add a known motor ID without scanning
    // Add a motor with full config (model + joint limits)
    void addMotor(uint8_t id, const MotorConfig& config = MotorConfig{});

    // --- Control (thread-safe, called by RL policy or ROS node) ---

    // Write a full MIT target for one motor
    void setTarget(uint8_t id, const MotorTarget& target);

    // Convenience: set target angle only, keep existing kp/kd/vel/torque
    void setAngle(uint8_t id, float angle);

    // Read latest state for one motor
    bool getState(uint8_t id, MotorState& state_out);

    // Read all motor states at once — for RL policy observation vector
    // Returns map of id → MotorState
    std::unordered_map<uint8_t, MotorState> getAllStates();

    // --- Arming ---
    bool enable(uint8_t id);
    bool disable(uint8_t id, bool clear_fault = false);
    void enableAll();
    void disableAll();

    // --- Control thread ---
    void start();   // start 200Hz control loop
    void stop();    // stop cleanly (disables all motors first)
    bool isRunning() const { return running_; }

    // --- Introspection (for UI) ---
    std::vector<uint8_t> motorIds() const;
    int getHz() const { return hz_; }

    // Actual measured loop rate (Hz) — for monitoring
    float measuredHz() const { return measured_hz_; }

private:
    Transport&  transport_;
    int         hz_;

    // Motor map — keyed by CAN ID
    // Access always under mutex_
    std::unordered_map<uint8_t, MotorEntry> motors_;
    mutable std::mutex mutex_;

    // Control thread
    std::thread        control_thread_;
    std::atomic<bool>  running_  { false };
    std::atomic<float> measured_hz_ { 0.0f };

    // Main control loop — runs in control_thread_
    void controlLoop();
};

