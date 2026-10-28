#pragma once
#include <cstdint>

// ============================================
// EVENT TYPES
// ============================================
enum class EventType : uint8_t {
    // Sensor events
    IMU_RAW,            // Accel + gyro, already rotated into the body frame (FLU)
    SENSOR_FAULT,       // A sensor reports it is unhealthy (SensorFaultData)

    // Estimation events
    ATTITUDE_UPDATED,   // Roll, pitch, yaw angles

    // Input events
    RC_INPUT,           // Decoded RC channel values

    // Command events
    MOTOR_COMMAND,      // Throttle values for 4 motors

    // System events
    ARMED,              // Drone armed (ArmingStatusData)
    DISARMED,           // Drone disarmed (ArmingStatusData)
    ARMING_STATUS,      // Arming state, blockers or RC link changed (ArmingStatusData)
    MODE_CHANGED,       // Flight mode changed
    HEARTBEAT,          // Periodic liveness signal

    _COUNT              // Must be last — used to size arrays
};

// ============================================
// EVENT DATA STRUCTS (POD types)
// ============================================
// Body frame FLU: X forward, Y left, Z up. Level and still: az = +1 g.
struct ImuData {
    float ax, ay, az;   // Accelerometer (g)
    float gx, gy, gz;   // Gyroscope (deg/s)
};

struct AttitudeData {
    float roll, pitch, yaw;  // Euler angles (deg), FLU: roll + = right side down, pitch + = nose down, yaw + = nose left
};

struct RcInputData {
    float throttle;                // 0.0 - 1.0
    float roll, pitch, yaw;       // -1.0 to 1.0
    uint8_t aux[4];               // Aux channel switches (aux[ARM_AUX_CHANNEL] = arm switch)
    bool linkOk;                  // false = RC link lost (failsafe packet), values not from the pilot
};

struct MotorCommandData {
    float motors[4];  // 0.0 - 1.0 per motor
};

struct ModeChangeData {
    uint8_t newMode;   // Maps to flight mode enum
};

struct SensorFaultData {
    bool fault;   // true = sensor unhealthy, its data must not be trusted
};

// Arming blockers: bit flags. Arming is allowed only when no bit is set.
namespace ArmingBlocker {
    constexpr uint16_t ALIGNMENT_NOT_VERIFIED = 1 << 0;  // IMU_ALIGNMENT_VERIFIED is false
    constexpr uint16_t NO_RC_LINK             = 1 << 1;  // No valid RC link (or not stable long enough)
    constexpr uint16_t THROTTLE_NOT_LOW       = 1 << 2;  // Throttle above the arming limit
    constexpr uint16_t ARM_SWITCH_NOT_RESET   = 1 << 3;  // Arm switch not seen OFF since boot / last disarm / blocked attempt
    constexpr uint16_t NOT_LEVEL              = 1 << 4;  // Roll or pitch beyond the arming limit
    constexpr uint16_t ATTITUDE_STALE         = 1 << 5;  // No recent attitude estimate
    constexpr uint16_t SENSOR_FAULT           = 1 << 6;  // A sensor reported a fault
}

enum class DisarmReason : uint8_t {
    NONE,             // Never disarmed since boot / currently armed
    SWITCH_OFF,       // Pilot turned the arm switch off
    RC_LOSS_TIMEOUT,  // RC link lost longer than the hold + descend window
    SENSOR_FAULT,     // A sensor reported a fault
    ATTITUDE_STALE,   // Attitude estimate stopped updating
    CRASH,            // Tilt beyond the crash angle for too long
    AUTO_DISARM,      // Armed but throttle stayed low (never took off)
};

struct ArmingStatusData {
    bool armed;
    uint16_t blockers;              // ArmingBlocker bit flags
    DisarmReason lastDisarmReason;
    bool rcLinkOk;
};
