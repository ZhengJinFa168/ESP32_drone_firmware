#pragma once
#include <cmath>
#include "utils/MathConstants.h"

// ============================================
// COMPLEMENTARY FILTER (pure logic, no hardware)
// ============================================
// Fuses gyro integration (fast, drifts) with the accelerometer's gravity direction
// (slow, noisy, no drift) for roll and pitch. Yaw is gyro-only.
//
// Tuned by a time constant tau (seconds), not a fixed alpha: the accelerometer pulls
// the estimate back over roughly tau seconds, at ANY sample rate. Each update uses
//     alpha = tau / (tau + dt)
// with that sample's real dt. (A fixed alpha = 0.98 means tau = 0.12 s at 400 Hz but
// tau = 0.05 s at 1 kHz.)
// Larger tau: smoother, less vibration in the attitude, slower drift correction.
//
// tau has no default here: the firmware value lives in config/FlightConfig.h
// (ATTITUDE_FILTER_TAU_S). An invalid tau (not a positive finite number) makes the
// filter unusable instead of substituting a guess: update() does nothing, no
// attitude is produced, and arming stays blocked on a stale attitude.
class ComplementaryFilter {
private:
    float _roll = 0.0f;    // Estimated roll angle (deg)
    float _pitch = 0.0f;   // Estimated pitch angle (deg)
    float _yaw = 0.0f;     // Estimated yaw angle (deg)

    float _tauS = 0.0f;    // Time constant (s)
    bool _validConfig = false;
    bool _initialized = false;

    static bool isValidTau(float tauS) { return std::isfinite(tauS) && tauS > 0.0f; }

public:
    explicit ComplementaryFilter(float tauS) : _tauS(tauS), _validConfig(isValidTau(tauS)) {}

    // Reset filter states; the next update snaps to the accelerometer
    void reset() {
        _roll = 0.0f;
        _pitch = 0.0f;
        _yaw = 0.0f;
        _initialized = false;
    }

    // Returns false (and keeps the current value) if tauS is not a positive finite number
    bool setTimeConstant(float tauS) {
        if (!isValidTau(tauS)) return false;
        _tauS = tauS;
        _validConfig = true;
        return true;
    }

    // False if constructed with an invalid tau: the filter produces no attitude
    bool hasValidConfig() const { return _validConfig; }

    float getTimeConstant() const { return _tauS; }
    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }
    bool isInitialized() const { return _initialized; }

    // Update filter with new IMU readings (body frame FLU)
    // ax, ay, az in g; gx, gy, gz in deg/s; dt in seconds (time since the previous sample).
    // The first update after construction/reset snaps to the accelerometer (dt ignored).
    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        if (!_validConfig) return;  // SAFETY: never estimate with an invalid tau

        // Calculate roll and pitch from accelerometer
        const float rollAcc = std::atan2(ay, az) * RAD2DEG;
        const float pitchAcc = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD2DEG;

        // On first reading, snap immediately to accelerometer reference
        if (!_initialized) {
            _roll = rollAcc;
            _pitch = pitchAcc;
            _yaw = 0.0f;
            _initialized = true;
            return;
        }

        if (!(dt > 0.0f)) return;  // Also rejects NaN

        // Accelerometer validity check: total magnitude in [0.5g, 1.5g]
        const float normSq = ax * ax + ay * ay + az * az;
        const bool accelValid = (normSq >= 0.25f && normSq <= 2.25f);

        if (accelValid) {
            // Fuse high-pass gyro integration with low-pass accelerometer gravity vector
            const float alpha = _tauS / (_tauS + dt);
            _roll = alpha * (_roll + gx * dt) + (1.0f - alpha) * rollAcc;
            _pitch = alpha * (_pitch + gy * dt) + (1.0f - alpha) * pitchAcc;
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
