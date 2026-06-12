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
            // Always force MIT mode — motors remember run_mode across power cycles
            it->second.motor.setRunMode(RunMode::MIT);
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
            entry.motor.setRunMode(RunMode::MIT);
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

bool MotorBus::setZero(uint8_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    bool ok = it->second.motor.setZero();
    if (ok) {
        // Reseed ramp from new zero position
        it->second.motor.requestFeedback(it->second.state);
        it->second.interpolated_angle      = it->second.state.angle;
        it->second.prev_interpolated_angle = it->second.state.angle;
    }
    return ok;
}

bool MotorBus::isFaulted(uint8_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    return it->second.faulted;
}

bool MotorBus::isStale(uint8_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    return it->second.stale;
}

bool MotorBus::clearFault(uint8_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = motors_.find(id);
    if (it == motors_.end()) return false;
    it->second.motor.disable(true);
    it->second.enabled = false;
    it->second.faulted = false;
    return true;
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

bool MotorBus::recoverBusOff() {
    fprintf(stderr, "[BUS-OFF] Detected — reinitializing can1...\n");
    std::lock_guard<std::mutex> lock(mutex_);

    // Bound each command to 2 seconds — never let a hung `ip` call
    // freeze the control loop indefinitely.
    int r = 0;
    r |= system("timeout 2 sudo ip link set can1 down");
    r |= system("timeout 2 sudo ip link set can1 type can bitrate 1000000");
    r |= system("timeout 2 sudo ip link set can1 up");

    if (r != 0) {
        fprintf(stderr, "[BUS-OFF] Recovery failed or timed out — check sudoers/interface\n");
        return false;
    }

    // Reopen the socket — the old fd is dead after interface bounce
    transport_.close();
    if (!transport_.open()) {
        fprintf(stderr, "[BUS-OFF] Failed to reopen transport\n");
        return false;
    }

    // Re-enable all motors that were enabled before BUS-OFF
    for (auto& [id, entry] : motors_) {
        if (entry.enabled) {
            entry.motor.enable();
            entry.motor.setRunMode(RunMode::MIT);
            entry.motor.requestFeedback(entry.state);
            entry.interpolated_angle      = entry.state.angle;
            entry.prev_interpolated_angle = entry.state.angle;
            entry.ramp_initialized        = true;
        }
    }

    busoff_count_   = 0;
    busoff_recover_ = busoff_recover_ + 1;
    fprintf(stderr, "[BUS-OFF] Recovery complete (total recoveries: %d)\n",
            busoff_recover_.load());
    return true;
}

void MotorBus::controlLoop() {
    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    bool need_recovery = false;

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

        // --- Phase 1: send MIT to all enabled motors (no recv) ---
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
                    cmd_angle = t.angle;
                    cmd_vel   = t.velocity;
                } else {
                    const float max_step = max_accel * dt;
                    const float error    = t.angle - entry.interpolated_angle;

                    entry.prev_interpolated_angle = entry.interpolated_angle;

                    if (error > max_step) {
                        entry.interpolated_angle += max_step;
                    } else if (error < -max_step) {
                        entry.interpolated_angle -= max_step;
                    } else {
                        entry.interpolated_angle = t.angle;
                    }

                    cmd_vel   = (entry.interpolated_angle - entry.prev_interpolated_angle) / dt;
                    cmd_angle = entry.interpolated_angle;
                }

                entry.motor.sendMIT(cmd_angle, cmd_vel, t.kp, t.kd, t.torque);
            }
        }

        // --- Phase 2: wait 2ms for motors to respond, then drain non-blocking ---
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            CANFrame rx = {};
            int frames_received = 0;
            auto now = std::chrono::steady_clock::now();

            // Drain all available frames — stops immediately when socket is empty
            while (transport_.recvNonBlocking(rx)) {
                frames_received++;
                uint8_t comm     = (rx.can_id >> 24) & 0x1F;
                uint8_t reply_id = (rx.can_id >> 8) & 0xFF;
                if (comm != COMM_FEEDBACK) continue;

                auto it = motors_.find(reply_id);
                if (it == motors_.end()) continue;

                MotorEntry& entry = it->second;
                if (!entry.enabled) continue;

                if (entry.motor.parseFeedback(rx, entry.state)) {
                    entry.state.last_update = std::chrono::steady_clock::now();
                    if (entry.state.fault != 0) {
                        entry.faulted = true;
                        entry.enabled = false;
                        fprintf(stderr, "[FAULT] Motor %d fault code: 0x%02X — auto-disabled\n",
                                reply_id, entry.state.fault);
                    }
                }
            }

            // --- BUS-OFF detection ---
            // Count enabled motors — if there are any and we got zero frames,
            // the bus may be dead. After 50 consecutive silent cycles (~250ms),
            // attempt recovery.
            int enabled_count = 0;
            for (auto& [id, entry] : motors_) {
                if (entry.enabled) enabled_count++;
            }

            if (enabled_count > 0 && frames_received == 0) {
                busoff_count_++;
                if (busoff_count_ >= 50) {
                    need_recovery = true;
                }
            } else {
                busoff_count_ = 0;  // reset on any successful receive
            }

            // --- Per-motor staleness watchdog ---
            // Tracks consecutive cycles since each motor's last feedback.
            // 20 cycles (~100ms) -> mark stale, keep sending last-known commands.
            // 100 cycles (~500ms) -> auto-disable, this joint is gone.
            for (auto& [id, entry] : motors_) {
                if (!entry.enabled) continue;

                auto age = std::chrono::duration<float, std::milli>(
                    now - entry.state.last_update).count();

                if (age > 5.0f) {
                    entry.stale_cycles++;
                } else {
                    entry.stale_cycles = 0;
                    entry.stale        = false;
                }

                if (entry.stale_cycles >= 20 && !entry.stale) {
                    entry.stale = true;
                    fprintf(stderr, "[STALE] Motor %d — no feedback for >100ms\n", id);
                }

                if (entry.stale_cycles >= 100) {
                    entry.enabled = false;
                    entry.faulted = true;
                    fprintf(stderr, "[STALE] Motor %d — no feedback for >500ms, auto-disabled\n", id);
                }
            }
        }
        // --- BUS-OFF recovery (outside the lock — recoverBusOff() locks internally) ---
        if (need_recovery) {
            recoverBusOff();
            need_recovery = false;
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
