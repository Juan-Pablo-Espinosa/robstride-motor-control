#pragma once
#include <cstdint>
#include <string>
#include <limits>

// ---------------------------------------------------------------------------
// MotorModel — supported RobStride actuator models
// ---------------------------------------------------------------------------
enum class MotorModel : uint8_t {
    RS00 = 0,
    RS01 = 1,
    RS02 = 2,
    RS03 = 3,
    RS04 = 4,
    RS05 = 5,
    RS06 = 6,
    UNKNOWN = 255,
};

// ---------------------------------------------------------------------------
// ModelSpec — hardware limits for each motor model
// These are physical maximums from the RobStride datasheet.
// Never command beyond these — they are used for normalization and clamping.
// ---------------------------------------------------------------------------
struct ModelSpec {
    float max_torque;    // Nm
    float max_velocity;  // rad/s
    float max_kp;        // normalization ceiling
    float max_kd;        // normalization ceiling
    float max_angle;     // rad  (+4pi for all current models)
};

// Returns the hardware spec for a given model.
// This is the single source of truth for per-model limits.
inline ModelSpec getModelSpec(MotorModel model) {
    switch (model) {
        case MotorModel::RS00: return {  17.0f, 50.0f,  500.0f,   5.0f, 12.566f };
        case MotorModel::RS01: return {  17.0f, 44.0f,  500.0f,   5.0f, 12.566f };
        case MotorModel::RS02: return {  17.0f, 44.0f,  500.0f,   5.0f, 12.566f };
        case MotorModel::RS03: return {  60.0f, 30.0f, 5000.0f, 100.0f, 12.566f };
        case MotorModel::RS04: return { 120.0f, 15.0f, 5000.0f, 100.0f, 12.566f };
        case MotorModel::RS05: return {  17.0f, 33.0f,  500.0f,   5.0f, 12.566f };
        case MotorModel::RS06: return {  60.0f, 20.0f, 5000.0f, 100.0f, 12.566f };
        default:               return {  17.0f, 30.0f,  500.0f,   5.0f, 12.566f };
    }
}

inline const char* modelName(MotorModel model) {
    switch (model) {
        case MotorModel::RS00: return "RS-00";
        case MotorModel::RS01: return "RS-01";
        case MotorModel::RS02: return "RS-02";
        case MotorModel::RS03: return "RS-03";
        case MotorModel::RS04: return "RS-04";
        case MotorModel::RS05: return "RS-05";
        case MotorModel::RS06: return "RS-06";
        default:               return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// MotorConfig — full configuration for one motor instance
//
// Combines:
//   1. Hardware identity (model → physical limits)
//   2. Per-joint software limits (set by you for your robot's geometry)
//   3. Mounting info (direction inversion for mirrored joints)
//
// Software limits are ALWAYS <= hardware limits.
// If you don't set software limits they default to the hardware limits.
// ---------------------------------------------------------------------------
struct MotorConfig {
    // --- Identity ---
    MotorModel  model       = MotorModel::RS03;
    std::string joint_name  = "";   // human-readable label, e.g. "knee_right"

    // --- Per-joint angle limits (radians) ---
    // Set these to the safe range for this joint in your robot.
    // Commands outside this range are silently clamped.
    float min_angle = -12.566f;  // default: full motor range (-4pi)
    float max_angle =  12.566f;  // default: full motor range (+4pi)

    // --- Per-joint velocity limit (rad/s) ---
    // Must be <= model hardware max. Defaults to hardware max.
    float max_velocity = -1.0f;  // -1 means "use hardware limit"

    // --- Per-joint torque limit (Nm) ---
    // Must be <= model hardware max. Defaults to hardware max.
    float max_torque = -1.0f;    // -1 means "use hardware limit"

    // --- Acceleration limit (rad/s²) ---
    // Limits how fast the interpolated target angle moves each cycle.
    // -1.0 = unlimited (passthrough, no ramp). Recommended: 5-20 rad/s² for joints.
    float max_acceleration = -1.0f;

    // --- Direction inversion ---
    // Set to true for motors mounted mirrored (e.g. left vs right leg).
    // Inverts angle commands and feedback signs transparently.
    bool invert_direction = false;

    // --- Resolved limits (call resolve() after construction) ---
    // These are the final clamping values used at runtime.
    // Automatically set to min(software_limit, hardware_limit).
    struct Resolved {
        float max_torque;
        float max_velocity;
        float max_kp;
        float max_kd;
        float hw_max_angle;
        bool  valid = false;
    } resolved;

    // Call this after setting all fields.
    // Resolves software limits against hardware limits and validates.
    void resolve() {
        ModelSpec spec = getModelSpec(model);

        // Clamp software limits to hardware limits
        resolved.max_torque   = (max_torque   < 0.0f) ? spec.max_torque
                              : (max_torque   > spec.max_torque)   ? spec.max_torque
                              : max_torque;

        resolved.max_velocity = (max_velocity < 0.0f) ? spec.max_velocity
                              : (max_velocity > spec.max_velocity) ? spec.max_velocity
                              : max_velocity;

        // kp/kd always use hardware limits for normalization
        resolved.max_kp       = spec.max_kp;
        resolved.max_kd       = spec.max_kd;
        resolved.hw_max_angle = spec.max_angle;

        // Clamp angle limits to hardware range
        if (min_angle < -spec.max_angle) min_angle = -spec.max_angle;
        if (max_angle >  spec.max_angle) max_angle =  spec.max_angle;

        resolved.valid = true;
    }

    // Clamp an angle command to joint limits, applying direction inversion
    float clampAngle(float angle) const {
        if (invert_direction) angle = -angle;
        if (angle < min_angle) angle = min_angle;
        if (angle > max_angle) angle = max_angle;
        return angle;
    }

    // Clamp a velocity command to joint limits
    float clampVelocity(float vel) const {
        if (invert_direction) vel = -vel;
        float limit = resolved.max_velocity;
        if (vel >  limit) vel =  limit;
        if (vel < -limit) vel = -limit;
        return vel;
    }

    // Clamp a torque command to joint limits
    float clampTorque(float torque) const {
        if (invert_direction) torque = -torque;
        float limit = resolved.max_torque;
        if (torque >  limit) torque =  limit;
        if (torque < -limit) torque = -limit;
        return torque;
    }

    // Correct feedback angle sign for inverted motors
    float correctFeedbackAngle(float angle) const {
        return invert_direction ? -angle : angle;
    }
};
