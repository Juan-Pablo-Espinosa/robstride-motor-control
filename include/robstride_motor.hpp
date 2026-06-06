#pragma once
#include "transport.hpp"
#include <cstdint>
#include <vector>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// Host ID used in data_2 field for most commands (per RobStride protocol)
static constexpr uint8_t  ROBSTRIDE_HOST_ID  = 0xFD;

// CAN communication type codes (bits 28-24 of the 29-bit CAN ID)
static constexpr uint8_t  COMM_OBTAIN_ID     = 0;
static constexpr uint8_t  COMM_CONTROL       = 1;   // MIT mode
static constexpr uint8_t  COMM_FEEDBACK      = 2;   // motor state reply
static constexpr uint8_t  COMM_ENABLE        = 3;
static constexpr uint8_t  COMM_STOP          = 4;
static constexpr uint8_t  COMM_SET_ZERO      = 6;
static constexpr uint8_t  COMM_SET_ID        = 7;
static constexpr uint8_t  COMM_READ_PARAM    = 17;
static constexpr uint8_t  COMM_WRITE_PARAM   = 18;

// Parameter register indices (little-endian uint16 in data[0-1])
static constexpr uint16_t PARAM_RUN_MODE     = 0x7005;
static constexpr uint16_t PARAM_IQ_REF       = 0x7006;
static constexpr uint16_t PARAM_SPD_REF      = 0x700A;
static constexpr uint16_t PARAM_LIMIT_TORQUE = 0x700B;
static constexpr uint16_t PARAM_CUR_KP       = 0x700C;
static constexpr uint16_t PARAM_CUR_KI       = 0x700D;
static constexpr uint16_t PARAM_LOC_REF      = 0x7010;
static constexpr uint16_t PARAM_LIMIT_SPD    = 0x7011;
static constexpr uint16_t PARAM_LIMIT_CUR    = 0x7012;
static constexpr uint16_t PARAM_MECH_POS     = 0x7014;  // read-only
static constexpr uint16_t PARAM_MECH_VEL     = 0x7016;  // read-only
static constexpr uint16_t PARAM_CAN_STATUS   = 0x3041;

// Run mode values for PARAM_RUN_MODE
enum class RunMode : uint8_t {
    MIT      = 0,
    Position = 1,
    Speed    = 2,
    Current  = 3,
};

// Motor operating mode extracted from feedback CAN ID bits 22-23
enum class MotorMode : uint8_t {
    Reset       = 0,
    Calibration = 1,
    Run         = 2,
};

// ---------------------------------------------------------------------------
// MotorState — parsed feedback from the motor
// ---------------------------------------------------------------------------
struct MotorState {
    float     angle;        // radians
    float     velocity;     // rad/s
    float     torque;       // Nm
    float   temperature;  // degrees C
    uint8_t   fault;        // fault flags byte
    MotorMode mode;
};

// ---------------------------------------------------------------------------
// RobstrideMotor
// ---------------------------------------------------------------------------
class RobstrideMotor {
public:
    // transport: open SocketCANTransport passed in by caller (not owned here)
    // motor_id: CAN ID of this motor (1-127), confirmed 42 for GR-0X
    RobstrideMotor(Transport& transport, uint8_t motor_id);

    // --- Discovery ---
    // Scan CAN IDs [start, end], return list of responding IDs
    std::vector<uint8_t> scan(uint8_t start = 1, uint8_t end = 127);

    // --- Arming ---
    bool enable();
    bool disable(bool clear_fault = false);

    // --- Mode / parameter control ---
    bool setRunMode(RunMode mode);
    bool writeParam(uint16_t index, float value);
    bool readParam(uint16_t index, float& value_out);

    // --- MIT (torque+impedance) control ---
    // All values clamped to protocol ranges internally
    bool sendMIT(float angle, float vel, float kp, float kd, float torque);

    // --- Feedback ---
    // Request a feedback frame and parse it into state_out
    bool requestFeedback(MotorState& state_out);

    // Parse a raw CANFrame that is already known to be a feedback frame
    static bool parseFeedback(const CANFrame& frame, MotorState& state_out);

    // --- Utility ---
    bool setZero();

private:
    Transport& transport_;
    uint8_t    motor_id_;

    // Build a standard command CAN ID:
    //   can_id = (comm_type << 24) | (data_2 << 8) | motor_id
    static uint32_t makeCANId(uint8_t comm_type, uint16_t data_2, uint8_t motor_id);

    // Normalize a float from [min, max] to uint16 [0, 65535]
    static uint16_t normalize(float value, float min, float max);

    // Denormalize a uint16 [0, 65535] back to float [min, max]
    static float denormalize(uint16_t raw, float min, float max);

    // Wait for a feedback frame with timeout, return false on timeout
    bool waitFeedback(MotorState& state_out, int timeout_ms = 50);
};
