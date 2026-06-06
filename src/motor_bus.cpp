#include "motor_bus.hpp"
#include <cstdio>
#include <algorithm>

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

MotorBus::MotorBus(Transport& transport, int hz)
    : transport_(transport), hz_(hz) {}

MotorBus::~MotorBus() {
    // emergencyStop sets running_=false without joining.
    // We then join here safely — destructor is not a signal handler.
    emergencyStop();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

std::vector<uint8_t> MotorBus::scanAndAdd(uint8_t start, uint8_t end,
                                           const MotorConfig& default_config) {
    MotorConfig scanner_config;
    RobstrideMotor scanner(transport_, 1, scanner_config);
    auto found = scanner.scan(start, end);

    std::lock_guard<std::mutex> lock(mutex_);
    for (uint8_t id : found) {
        if (motors_.find(id) == motors_.end()) {
            motors_.emplace(id, MotorEntry(transport_, id, default_config));
        }
    }
    return found;
}

void MotorBus::addMotor(uint8_t id, const MotorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motors_.find(id) == motors_.end()) {
        motors_.emplace(id, MotorEntry(transport_, id, config));
    }
}

// ---------------------------------------------------------------------------
// Control (thread-safe)
// ---------------------------------------------------------------------------

void MotorBus::setTarget(uint8_t id, const MotorTarget& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it != motors_.end()) {
        it->second.target = target;
    }
}

void MotorBus::setAngle(uint8_t id, float angle) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it != motors_.end()) {
        it->second.target.angle = angle;
    }
}

bool MotorBus::getState(uint8_t id, MotorState& state_out) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    state_out = it->second.state;
    return true;
}

std::unordered_map<uint8_t, MotorState> MotorBus::getAllStates() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<uint8_t, MotorState> out;
    for (auto& [id, entry] : motors_) {
        out[id] = entry.state;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Arming
// ---------------------------------------------------------------------------

bool MotorBus::enable(uint8_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    bool ok = it->second.motor.enable();
        if (ok) {
            it->second.enabled = true;
            // Immediately populate state so getState() after enable() is never stale zeros.
            // enable() already got a feedback frame — request one more to fill entry.state.
            it->second.motor.requestFeedback(it->second.state);
            // Seed the acceleration ramp from the motor's actual position.
            // This prevents the ramp from driving the motor from 0 to its real position
            // on the first control cycle after arming.
            it->second.interpolated_angle      = it->second.state.angle;
            it->second.prev_interpolated_angle = it->second.state.angle;
            it->second.ramp_initialized        = true;
        }
    return ok;
}

bool MotorBus::disable(uint8_t id, bool clear_fault) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    bool ok = it->second.motor.disable(clear_fault);
    it->second.enabled = false;
    return ok;
}

void MotorBus::enableAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [id, entry] : motors_) {
        if (entry.motor.enable()) {
            entry.enabled = true;
            entry.motor.requestFeedback(entry.state);
            entry.interpolated_angle      = entry.state.angle;
            entry.prev_interpolated_angle = entry.state.angle;
            entry.ramp_initialized        = true;
        }
    }
}

void MotorBus::disableAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [id, entry] : motors_) {
        entry.motor.disable(false);
        entry.enabled = false;
    }
}

// ---------------------------------------------------------------------------
// Emergency stop
//
// Async-signal-safe: only writes to atomics, no malloc, no mutex, no I/O.
// The control loop polls estop_ at the top of every cycle and handles the
// actual COMM_STOP sends from the loop thread where the mutex is safe to use.
// ---------------------------------------------------------------------------
void MotorBus::emergencyStop() {
    estop_   = true;
    running_ = false;
}

// ---------------------------------------------------------------------------
// Control thread
// ---------------------------------------------------------------------------

void MotorBus::start() {
    if (running_) return;
    estop_   = false;   // clear any previous estop before starting
    running_ = true;
    control_thread_ = std::thread(&MotorBus::controlLoop, this);
}

void MotorBus::stop() {
    if (!running_) return;
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    // Graceful shutdown — disable all motors after thread exits cleanly
    disableAll();
}

void MotorBus::controlLoop() {
    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    const auto period = std::chrono::microseconds(1000000 / hz_);
    auto next_wake    = clock::now();

    auto rate_window_start = clock::now();
    int  rate_cycles       = 0;

    while (running_) {

        // --- Emergency stop check ---
        // Checked first, before acquiring the mutex or touching CAN.
        // estop_ is set by emergencyStop() which may be called from a signal
        // handler. We handle it here in the loop thread where mutex is safe.
        if (estop_) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [id, entry] : motors_) {
                if (entry.enabled) {
                    entry.motor.disable(false);
                    entry.enabled = false;
                }
            }
            // running_ was already set false by emergencyStop()
            break;
        }

        // --- Send MIT commands and read feedback for all enabled motors ---
        {
            const float dt = 1.0f / static_cast<float>(hz_);

            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [id, entry] : motors_) {
                if (!entry.enabled) continue;

                const MotorTarget& t = entry.target;
                const float max_accel = entry.motor.config().max_acceleration;

                float cmd_angle;
                float cmd_vel;

                if (max_accel < 0.0f || !entry.ramp_initialized) {
                    // Unlimited / passthrough — use target directly
                    cmd_angle = t.angle;
                    cmd_vel   = t.velocity;
                } else {
                    // Acceleration-limited ramp
                    const float max_step = max_accel * dt;  // rad per cycle
                    const float error    = t.angle - entry.interpolated_angle;

                    entry.prev_interpolated_angle = entry.interpolated_angle;

                    if (error > max_step) {
                        entry.interpolated_angle += max_step;
                    } else if (error < -max_step) {
                        entry.interpolated_angle -= max_step;
                    } else {
                        entry.interpolated_angle = t.angle;
                    }

                    // Feedforward velocity = how fast we're moving the interpolated target
                    cmd_vel   = (entry.interpolated_angle - entry.prev_interpolated_angle) / dt;
                    cmd_angle = entry.interpolated_angle;
                }

                entry.motor.sendMIT(cmd_angle, cmd_vel, t.kp, t.kd, t.torque);
                entry.motor.requestFeedback(entry.state);
            }
        }

        // --- Measure actual loop rate every second ---
        rate_cycles++;
        auto now = clock::now();
        double elapsed = duration(now - rate_window_start).count();
        if (elapsed >= 1.0) {
            measured_hz_ = static_cast<float>(rate_cycles / elapsed);
            rate_cycles       = 0;
            rate_window_start = now;
        }

        // --- Sleep until next cycle ---
        next_wake += period;
        std::this_thread::sleep_until(next_wake);
    }
}

// ---------------------------------------------------------------------------
// Introspection
// ---------------------------------------------------------------------------

std::vector<uint8_t> MotorBus::motorIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> ids;
    ids.reserve(motors_.size());
    for (auto& [id, _] : motors_) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}
