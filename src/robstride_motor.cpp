#include "robstride_motor.hpp"
#include <cstring>
#include <cstdio>

uint16_t RobstrideMotor::normalize(float value, float min, float max) {
    if (value < min) value = min;
    if (value > max) value = max;
    return static_cast<uint16_t>((value - min) / (max - min) * 65535.0f);
}

float RobstrideMotor::denormalize(uint16_t raw, float min, float max) {
    return min + (static_cast<float>(raw) / 65535.0f) * (max - min);
}

uint32_t RobstrideMotor::makeCANId(uint8_t comm_type, uint16_t data_2, uint8_t motor_id) {
    return (static_cast<uint32_t>(comm_type) << 24)
         | (static_cast<uint32_t>(data_2)    <<  8)
         | static_cast<uint32_t>(motor_id);
}

RobstrideMotor::RobstrideMotor(Transport& transport, uint8_t motor_id,
                               const MotorConfig& config)
    : transport_(transport), motor_id_(motor_id), config_(config) {
    // Resolve limits on construction so clamping is always valid
    config_.resolve();
}

std::vector<uint8_t> RobstrideMotor::scan(uint8_t start, uint8_t end) {
    std::vector<uint8_t> found;
    for (uint8_t id = start; id <= end; ++id) {
        CANFrame tx = {};
        tx.can_id = makeCANId(COMM_OBTAIN_ID, ROBSTRIDE_HOST_ID, id);
        tx.len    = 8;
        std::memset(tx.data, 0, 8);
        if (!transport_.send(tx)) continue;
        CANFrame rx = {};
        if (transport_.recv(rx, 50)) {
            uint8_t comm         = (rx.can_id >> 24) & 0x1F;
            uint8_t discovered_id = (rx.can_id >> 8) & 0xFF;
            if (comm == COMM_OBTAIN_ID) {
                printf("Found motor ID: %d\n", discovered_id);
                found.push_back(discovered_id);
            }
        }
    }
    return found;
}

bool RobstrideMotor::enable() {
    CANFrame tx = {};
    tx.can_id = makeCANId(COMM_ENABLE, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len    = 8;
    std::memset(tx.data, 0, 8);
    if (!transport_.send(tx)) return false;
    MotorState state;
    return waitFeedback(state);
}

bool RobstrideMotor::disable(bool clear_fault) {
    CANFrame tx = {};
    tx.can_id  = makeCANId(COMM_STOP, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len     = 8;
    std::memset(tx.data, 0, 8);
    if (clear_fault) tx.data[0] = 1;
    if (!transport_.send(tx)) return false;
    MotorState state;
    return waitFeedback(state);
}

bool RobstrideMotor::writeParam(uint16_t index, float value) {
    CANFrame tx = {};
    tx.can_id  = makeCANId(COMM_WRITE_PARAM, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len     = 8;
    tx.data[0] = index & 0xFF;
    tx.data[1] = (index >> 8) & 0xFF;
    tx.data[2] = 0x00;
    tx.data[3] = 0x00;
    std::memcpy(&tx.data[4], &value, sizeof(float));
    if (!transport_.send(tx)) return false;
    MotorState state;
    return waitFeedback(state);
}

bool RobstrideMotor::readParam(uint16_t index, float& value_out) {
    CANFrame tx = {};
    tx.can_id  = makeCANId(COMM_READ_PARAM, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len     = 8;
    tx.data[0] = index & 0xFF;
    tx.data[1] = (index >> 8) & 0xFF;
    std::memset(&tx.data[2], 0, 6);
    if (!transport_.send(tx)) return false;
    CANFrame rx = {};
    if (!transport_.recv(rx, 50)) return false;
    std::memcpy(&value_out, &rx.data[4], sizeof(float));
    return true;
}

bool RobstrideMotor::setRunMode(RunMode mode) {
    return writeParam(PARAM_RUN_MODE, static_cast<float>(mode));
}

bool RobstrideMotor::sendMIT(float angle, float vel, float kp, float kd, float torque) {
    // Apply joint limits and direction inversion
    angle  = config_.clampAngle(angle);
    vel    = config_.clampVelocity(vel);
    torque = config_.clampTorque(torque);

    // Clamp kp/kd to model hardware limits
    if (kp < 0.0f) kp = 0.0f;
    if (kp > config_.resolved.max_kp) kp = config_.resolved.max_kp;
    if (kd < 0.0f) kd = 0.0f;
    if (kd > config_.resolved.max_kd) kd = config_.resolved.max_kd;

    // Normalize torque into CAN ID data_2 field
    uint16_t torque_raw = normalize(torque,
        -config_.resolved.max_torque, config_.resolved.max_torque);

    CANFrame tx = {};
    tx.can_id = makeCANId(COMM_CONTROL, torque_raw, motor_id_);
    tx.len    = 8;

    uint16_t a = normalize(angle, -config_.resolved.hw_max_angle,
                                   config_.resolved.hw_max_angle);
    uint16_t v = normalize(vel,   -config_.resolved.max_velocity,
                                   config_.resolved.max_velocity);
    uint16_t p = normalize(kp,     0.0f, config_.resolved.max_kp);
    uint16_t d = normalize(kd,     0.0f, config_.resolved.max_kd);

    tx.data[0] = (a >> 8) & 0xFF;  tx.data[1] = a & 0xFF;
    tx.data[2] = (v >> 8) & 0xFF;  tx.data[3] = v & 0xFF;
    tx.data[4] = (p >> 8) & 0xFF;  tx.data[5] = p & 0xFF;
    tx.data[6] = (d >> 8) & 0xFF;  tx.data[7] = d & 0xFF;

    return transport_.send(tx);
}

bool RobstrideMotor::parseFeedback(const CANFrame& frame, MotorState& state_out) const {
    uint8_t comm = (frame.can_id >> 24) & 0x1F;
    if (comm != COMM_FEEDBACK) return false;

    state_out.mode = static_cast<MotorMode>((frame.can_id >> 22) & 0x03);

    uint16_t a = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    uint16_t v = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    uint16_t t = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

    float angle = denormalize(a, -config_.resolved.hw_max_angle,
                                  config_.resolved.hw_max_angle);

    // Correct feedback for inverted motors
    state_out.angle    = config_.correctFeedbackAngle(angle);
    state_out.velocity = denormalize(v, -config_.resolved.max_velocity,
                                        config_.resolved.max_velocity);
    state_out.torque   = denormalize(t, -config_.resolved.max_torque,
                                        config_.resolved.max_torque);

    uint16_t temp_raw  = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
    state_out.temperature = static_cast<float>(temp_raw) / 10.0f;
    state_out.fault    = 0;

    return true;
}

bool RobstrideMotor::requestFeedback(MotorState& state_out) {
    return sendMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f) && waitFeedback(state_out);
}

bool RobstrideMotor::waitFeedback(MotorState& state_out, int timeout_ms) {
    CANFrame rx = {};
    if (!transport_.recv(rx, timeout_ms)) return false;
    return parseFeedback(rx, state_out);
}

bool RobstrideMotor::setZero() {
    CANFrame tx = {};
    tx.can_id  = makeCANId(COMM_SET_ZERO, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len     = 8;
    std::memset(tx.data, 0, 8);
    tx.data[0] = 1;
    if (!transport_.send(tx)) return false;
    MotorState state;
    return waitFeedback(state);
}
