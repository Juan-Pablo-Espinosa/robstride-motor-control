#include "motor_bus.hpp"
#include <cstdio>
#include <algorithm>

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

MotorBus::MotorBus(Transport& transport, int hz)
    : transport_(transport), hz_(hz) {}

MotorBus::~MotorBus() {
    stop();
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
    if (ok) it->second.enabled = true;
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
// Control thread
// ---------------------------------------------------------------------------

void MotorBus::start() {
    if (running_) return;
    running_ = true;
    control_thread_ = std::thread(&MotorBus::controlLoop, this);
}

void MotorBus::stop() {
    if (!running_) return;
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    // Safe shutdown — disable all motors after thread exits
    disableAll();
}

void MotorBus::controlLoop() {
    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    const auto period = std::chrono::microseconds(1000000 / hz_);
    auto next_wake    = clock::now();

    // For measuring actual loop rate
    auto   rate_window_start = clock::now();
    int    rate_cycles        = 0;

    while (running_) {
        // cycle_start reserved for future jitter monitoring
        (void)clock::now();

        // --- Send MIT commands and read feedback for all enabled motors ---
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [id, entry] : motors_) {
                if (!entry.enabled) continue;

                const MotorTarget& t = entry.target;

                // Send MIT command
                entry.motor.sendMIT(
                    t.angle, t.velocity, t.kp, t.kd, t.torque);

                // Read feedback — non-blocking with short timeout
                // If motor misses a cycle that's ok, state stays stale
                entry.motor.requestFeedback(entry.state);
            }
        }

        // --- Measure actual loop rate every second ---
        rate_cycles++;
        auto now = clock::now();
        double elapsed = duration(now - rate_window_start).count();
        if (elapsed >= 1.0) {
            measured_hz_ = static_cast<float>(rate_cycles / elapsed);
            rate_cycles        = 0;
            rate_window_start  = now;
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

