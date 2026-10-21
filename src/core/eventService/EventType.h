#pragma once
#include <cstdint>

// ============================================
// EVENT TYPES
// ============================================
enum class EventType : uint8_t {
    // Sensor events
    IMU_RAW,            // Accel + gyro, already rotated into the body frame (FLU)

    // Estimation events
    ATTITUDE_UPDATED,   // Roll, pitch, yaw angles

    // Input events
    RC_INPUT,           // Decoded RC channel values

    // Command events
    MOTOR_COMMAND,      // Throttle values for 4 motors

    // System events
    ARMED,              // Drone armed
    DISARMED,           // Drone disarmed
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
    uint8_t aux[4];               // Aux channel switches
};

struct MotorCommandData {
    float motors[4];  // 0.0 - 1.0 per motor
};

struct ModeChangeData {
    uint8_t newMode;   // Maps to flight mode enum
};
