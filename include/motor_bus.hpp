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
    bool           enabled     = false;
    bool           faulted     = false;

    // Acceleration ramp state
    float interpolated_angle      = 0.0f;  // current ramp position (rad)
    float prev_interpolated_angle = 0.0f;  // for feedforward velocity calculation
    bool  ramp_initialized        = false; // seeded from real motor position on first enable

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
//   setTarget()      — safe to call from any thread (RL policy / ROS node)
//   getState()       — safe to call from any thread
//   enable/disable   — safe to call from UI thread
//   emergencyStop()  — async-signal-safe, callable from SIGINT/SIGTERM handler
//   control loop     — runs internally, do not call manually
// ---------------------------------------------------------------------------
class MotorBus {
public:
    explicit MotorBus(Transport& transport, int hz = 200);
    ~MotorBus();

    // --- Setup ---
    std::vector<uint8_t> scanAndAdd(uint8_t start = 1, uint8_t end = 127,
                                    const MotorConfig& default_config = MotorConfig{});
    void addMotor(uint8_t id, const MotorConfig& config = MotorConfig{});

    // --- Control (thread-safe, called by RL policy or ROS node) ---
    void setTarget(uint8_t id, const MotorTarget& target);
    void setAngle(uint8_t id, float angle);
    bool getState(uint8_t id, MotorState& state_out);
    std::unordered_map<uint8_t, MotorState> getAllStates();

    // --- Arming ---
    bool enable(uint8_t id);
    bool disable(uint8_t id, bool clear_fault = false);
    void enableAll();
    void disableAll();

    // --- Calibration ---
    bool setZero(uint8_t id);  // permanently sets current position as zero — use carefully

    // --- Fault ---
    bool isFaulted(uint8_t id);
    bool clearFault(uint8_t id);  // disable(clear_fault=true) + resets faulted flag

    // --- Control thread ---
    void start();
    void stop();    // graceful: joins thread, then disables all motors
    bool isRunning() const { return running_; }

    // --- Emergency stop ---
    // Async-signal-safe. Sets estop_ and running_ atomically.
    // The control loop detects estop_ at the top of the next cycle (<=5ms),
    // sends COMM_STOP to all enabled motors, and exits cleanly.
    // Safe to call from SIGINT/SIGTERM handler or any thread.
    // Idempotent — safe to call multiple times.
    void emergencyStop();

    // --- Introspection ---
    std::vector<uint8_t> motorIds() const;
    int   getHz()       const { return hz_; }
    float measuredHz()  const { return measured_hz_; }

private:
    Transport&  transport_;
    int         hz_;

    std::unordered_map<uint8_t, MotorEntry> motors_;
    mutable std::mutex mutex_;

    std::thread        control_thread_;
    std::atomic<bool>  running_  { false };
    std::atomic<bool>  estop_    { false };   // set by emergencyStop()
    std::atomic<float> measured_hz_ { 0.0f };

    void controlLoop();
};
