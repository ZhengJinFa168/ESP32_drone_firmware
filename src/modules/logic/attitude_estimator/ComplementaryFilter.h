#pragma once
#include <cmath>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943f
#endif

class ComplementaryFilter {
private:
    float _roll = 0.0f;    // Estimated roll angle (deg)
    float _pitch = 0.0f;   // Estimated pitch angle (deg)
    float _yaw = 0.0f;     // Estimated yaw angle (deg)

    float _alpha = 0.98f;  // Gyro weight (0.98 = 98% gyro, 2% accel)
    bool _initialized = false;

public:
    explicit ComplementaryFilter(float alpha = 0.98f) : _alpha(alpha) {}

    // Reset filter states
    void reset() {
        _roll = 0.0f;
        _pitch = 0.0f;
        _yaw = 0.0f;
        _initialized = false;
    }

    // Set filter weighting coefficient (0.0 to 1.0)
    void setAlpha(float alpha) {
        if (alpha >= 0.0f && alpha <= 1.0f) {
            _alpha = alpha;
        }
    }

    float getAlpha() const { return _alpha; }
    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }
    bool isInitialized() const { return _initialized; }

    // Update filter with new IMU readings
    // ax, ay, az in g
    // gx, gy, gz in deg/s
    // dt in seconds
    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        if (dt <= 0.0f) return;

        // Calculate roll and pitch from accelerometer
        float rollAcc = std::atan2(ay, az) * RAD_TO_DEG;
        float pitchAcc = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD_TO_DEG;

        // On first reading, snap immediately to accelerometer reference
        if (!_initialized) {
            _roll = rollAcc;
            _pitch = pitchAcc;
            _yaw = 0.0f;
            _initialized = true;
            return;
        }

        // Accelerometer validity check: total magnitude in [0.5g, 1.5g]
        float normSq = ax * ax + ay * ay + az * az;
        bool accelValid = (normSq >= 0.25f && normSq <= 2.25f);

        if (accelValid) {
            // Fuse high-pass gyro integration with low-pass accelerometer gravity vector
            _roll = _alpha * (_roll + gx * dt) + (1.0f - _alpha) * rollAcc;
            _pitch = _alpha * (_pitch + gy * dt) + (1.0f - _alpha) * pitchAcc;
        } else {
            // High dynamic acceleration / free-fall: trust gyro integration only
            _roll += gx * dt;
            _pitch += gy * dt;
        }

        // Yaw has no gravity reference, integrate gyro z rate
        _yaw += gz * dt;

        // Wrap yaw to [-180, 180] degrees
        while (_yaw > 180.0f) _yaw -= 360.0f;
        while (_yaw < -180.0f) _yaw += 360.0f;
    }
};
