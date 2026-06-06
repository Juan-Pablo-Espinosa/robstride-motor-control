#include "robstride_motor.hpp"
#include <cstring>
#include <cstdio>

// ---------------------------------------------------------------------------
// Helpers — normalization math
// ---------------------------------------------------------------------------

// Map a float in [min, max] to a uint16 in [0, 65535]
// Values outside range are clamped
uint16_t RobstrideMotor::normalize(float value, float min, float max) {
    if (value < min) value = min;
    if (value > max) value = max;
    return static_cast<uint16_t>((value - min) / (max - min) * 65535.0f);
}

// Map a uint16 in [0, 65535] back to float in [min, max]
float RobstrideMotor::denormalize(uint16_t raw, float min, float max) {
    return min + (static_cast<float>(raw) / 65535.0f) * (max - min);
}

// Build the 29-bit CAN ID from its three fields
uint32_t RobstrideMotor::makeCANId(uint8_t comm_type, uint16_t data_2, uint8_t motor_id) {
    return (static_cast<uint32_t>(comm_type) << 24)
         | (static_cast<uint32_t>(data_2)    <<  8)
         | static_cast<uint32_t>(motor_id);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RobstrideMotor::RobstrideMotor(Transport& transport, uint8_t motor_id)
    : transport_(transport), motor_id_(motor_id) {}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

std::vector<uint8_t> RobstrideMotor::scan(uint8_t start, uint8_t end) {
    std::vector<uint8_t> found;

    for (uint8_t id = start; id <= end; ++id) {
        // ObtainID probe: comm_type=0, data_2=host_id, motor_id=target
        CANFrame tx = {};
        tx.can_id = makeCANId(COMM_OBTAIN_ID, ROBSTRIDE_HOST_ID, id);
        tx.len    = 8;
        std::memset(tx.data, 0, 8);

        if (!transport_.send(tx)) {
            fprintf(stderr, "scan: send failed at id %d\n", id);
            continue;
        }

        // Motor replies with comm_type=0, data_2=its own CAN ID
        CANFrame rx = {};
        if (transport_.recv(rx, 50)) {
            uint8_t comm = (rx.can_id >> 24) & 0x1F;
            uint8_t discovered_id = (rx.can_id >> 8) & 0xFF;
            if (comm == COMM_OBTAIN_ID) {
                printf("Found motor ID: %d\n", discovered_id);
                found.push_back(discovered_id);
            }
        }
    }

    return found;
}

// ---------------------------------------------------------------------------
// Arming
// ---------------------------------------------------------------------------

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

    // data[0] = 1 clears fault on stop, per RobStride protocol docs
    if (clear_fault) tx.data[0] = 1;

    if (!transport_.send(tx)) return false;

    MotorState state;
    return waitFeedback(state);
}

// ---------------------------------------------------------------------------
// Parameter read / write
// ---------------------------------------------------------------------------

bool RobstrideMotor::writeParam(uint16_t index, float value) {
    CANFrame tx = {};
    tx.can_id = makeCANId(COMM_WRITE_PARAM, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len    = 8;

    // data[0-1]: parameter index, little-endian uint16
    tx.data[0] = index & 0xFF;
    tx.data[1] = (index >> 8) & 0xFF;
    tx.data[2] = 0x00;
    tx.data[3] = 0x00;

    // data[4-7]: value as little-endian float32
    std::memcpy(&tx.data[4], &value, sizeof(float));

    if (!transport_.send(tx)) return false;

    MotorState state;
    return waitFeedback(state);
}

bool RobstrideMotor::readParam(uint16_t index, float& value_out) {
    CANFrame tx = {};
    tx.can_id = makeCANId(COMM_READ_PARAM, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len    = 8;

    // data[0-1]: parameter index, little-endian uint16
    tx.data[0] = index & 0xFF;
    tx.data[1] = (index >> 8) & 0xFF;
    tx.data[2] = 0x00;
    tx.data[3] = 0x00;
    tx.data[4] = 0x00;
    tx.data[5] = 0x00;
    tx.data[6] = 0x00;
    tx.data[7] = 0x00;

    if (!transport_.send(tx)) return false;

    // Motor replies with comm_type=2 (Feedback), value in data[4-7]
    CANFrame rx = {};
    if (!transport_.recv(rx, 50)) return false;

    std::memcpy(&value_out, &rx.data[4], sizeof(float));
    return true;
}

bool RobstrideMotor::setRunMode(RunMode mode) {
    return writeParam(PARAM_RUN_MODE, static_cast<float>(mode));
}

// ---------------------------------------------------------------------------
// MIT control
// ---------------------------------------------------------------------------

bool RobstrideMotor::sendMIT(float angle, float vel, float kp, float kd, float torque) {
    // Normalize torque into data_2 field of CAN ID (16 bits)
    // Range: [-120, +120] Nm
    uint16_t torque_raw = normalize(torque, -120.0f, 120.0f);

    CANFrame tx = {};
    tx.can_id = makeCANId(COMM_CONTROL, torque_raw, motor_id_);
    tx.len    = 8;

    // Pack payload big-endian, each field normalized to uint16
    // bytes 0-1: angle    [-4pi, +4pi] rad
    // bytes 2-3: velocity [-30, +30] rad/s
    // bytes 4-5: kp       [0, 500]
    // bytes 6-7: kd       [0, 5]
    uint16_t a = normalize(angle, -12.566f, 12.566f);  // -4pi to +4pi
    uint16_t v = normalize(vel,   -30.0f,   30.0f);
    uint16_t p = normalize(kp,      0.0f,  500.0f);
    uint16_t d = normalize(kd,      0.0f,    5.0f);

    tx.data[0] = (a >> 8) & 0xFF;
    tx.data[1] =  a       & 0xFF;
    tx.data[2] = (v >> 8) & 0xFF;
    tx.data[3] =  v       & 0xFF;
    tx.data[4] = (p >> 8) & 0xFF;
    tx.data[5] =  p       & 0xFF;
    tx.data[6] = (d >> 8) & 0xFF;
    tx.data[7] =  d       & 0xFF;

    return transport_.send(tx);
}

// ---------------------------------------------------------------------------
// Feedback
// ---------------------------------------------------------------------------

// Static: parse a raw feedback CANFrame into a MotorState
bool RobstrideMotor::parseFeedback(const CANFrame& frame, MotorState& state_out) {
    uint8_t comm = (frame.can_id >> 24) & 0x1F;
    if (comm != COMM_FEEDBACK) return false;

    // Extract MotorMode from bits 22-23 of the full CAN ID
    state_out.mode = static_cast<MotorMode>((frame.can_id >> 22) & 0x03);

    // Denormalize payload fields
    uint16_t a = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    uint16_t v = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    uint16_t t = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

    state_out.angle       = denormalize(a, -12.566f, 12.566f);
    state_out.velocity    = denormalize(v, -30.0f,   30.0f);
    state_out.torque      = denormalize(t, -120.0f,  120.0f);
    // Temperature is a uint16 across bytes 6-7, scaled by 10.0 (units: 0.1°C)
    // e.g. raw=0x0118=280 → 280/10.0 = 28.0°C
    uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
    state_out.temperature = static_cast<float>(temp_raw) / 10.0f;
    state_out.fault       = 0;  // no separate fault byte — it was part of temperature

    return true;
}

// Request a feedback frame from the motor and parse it
bool RobstrideMotor::requestFeedback(MotorState& state_out) {
    // Send a zero MIT control frame — this is the correct way to request
    // feedback. The motor replies to every comm_type=1 frame with a
    // comm_type=2 feedback frame. Using ObtainID does NOT trigger feedback.
    // Zero torque, zero velocity, zero kp/kd = safe torque-free nudge.
    return sendMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f) && waitFeedback(state_out);
}

// Internal: block until a feedback frame arrives or timeout
bool RobstrideMotor::waitFeedback(MotorState& state_out, int timeout_ms) {
    CANFrame rx = {};
    if (!transport_.recv(rx, timeout_ms)) return false;
    return parseFeedback(rx, state_out);
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

bool RobstrideMotor::setZero() {
    CANFrame tx = {};
    tx.can_id  = makeCANId(COMM_SET_ZERO, ROBSTRIDE_HOST_ID, motor_id_);
    tx.len     = 8;
    std::memset(tx.data, 0, 8);
    tx.data[0] = 1;  // required by protocol to confirm intent

    if (!transport_.send(tx)) return false;

    MotorState state;
    return waitFeedback(state);
}
