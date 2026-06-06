#pragma once
#include "transport.hpp"
#include "motor_config.hpp"
#include <cstdint>
#include <vector>

static constexpr uint8_t  ROBSTRIDE_HOST_ID  = 0xFD;
static constexpr uint8_t  COMM_OBTAIN_ID     = 0;
static constexpr uint8_t  COMM_CONTROL       = 1;
static constexpr uint8_t  COMM_FEEDBACK      = 2;
static constexpr uint8_t  COMM_ENABLE        = 3;
static constexpr uint8_t  COMM_STOP          = 4;
static constexpr uint8_t  COMM_SET_ZERO      = 6;
static constexpr uint8_t  COMM_SET_ID        = 7;
static constexpr uint8_t  COMM_READ_PARAM    = 17;
static constexpr uint8_t  COMM_WRITE_PARAM   = 18;

static constexpr uint16_t PARAM_RUN_MODE     = 0x7005;
static constexpr uint16_t PARAM_IQ_REF       = 0x7006;
static constexpr uint16_t PARAM_SPD_REF      = 0x700A;
static constexpr uint16_t PARAM_LIMIT_TORQUE = 0x700B;
static constexpr uint16_t PARAM_CUR_KP       = 0x700C;
static constexpr uint16_t PARAM_CUR_KI       = 0x700D;
static constexpr uint16_t PARAM_LOC_REF      = 0x7010;
static constexpr uint16_t PARAM_LIMIT_SPD    = 0x7011;
static constexpr uint16_t PARAM_LIMIT_CUR    = 0x7012;
static constexpr uint16_t PARAM_MECH_POS     = 0x7014;
static constexpr uint16_t PARAM_MECH_VEL     = 0x7016;
static constexpr uint16_t PARAM_CAN_STATUS   = 0x3041;

enum class RunMode : uint8_t {
    MIT      = 0,
    Position = 1,
    Speed    = 2,
    Current  = 3,
};

enum class MotorMode : uint8_t {
    Reset       = 0,
    Calibration = 1,
    Run         = 2,
};

struct MotorState {
    float     angle;
    float     velocity;
    float     torque;
    float     temperature;
    uint8_t   fault;
    MotorMode mode;
};

class RobstrideMotor {
public:
    RobstrideMotor(Transport& transport, uint8_t motor_id,
                   const MotorConfig& config);

    std::vector<uint8_t> scan(uint8_t start = 1, uint8_t end = 127);

    bool enable();
    bool disable(bool clear_fault = false);
    bool setRunMode(RunMode mode);
    bool writeParam(uint16_t index, float value);
    bool readParam(uint16_t index, float& value_out);

    // All values are clamped to model+joint limits before sending
    bool sendMIT(float angle, float vel, float kp, float kd, float torque);

    bool requestFeedback(MotorState& state_out);
    bool parseFeedback(const CANFrame& frame, MotorState& state_out) const;
    bool setZero();

    const MotorConfig& config() const { return config_; }
    uint8_t id() const { return motor_id_; }

private:
    Transport&  transport_;
    uint8_t     motor_id_;
    MotorConfig config_;

    static uint32_t makeCANId(uint8_t comm_type, uint16_t data_2, uint8_t motor_id);
    static uint16_t normalize(float value, float min, float max);
    static float    denormalize(uint16_t raw, float min, float max);
    bool waitFeedback(MotorState& state_out, int timeout_ms = 50);
};
