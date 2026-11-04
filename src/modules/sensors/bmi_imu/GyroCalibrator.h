#pragma once
#include <cmath>
#include <cstdint>
#include "config/SensorConfig.h"

// ============================================
// GYRO BIAS CALIBRATION (pure logic, no hardware)
// ============================================
// A still gyro does not read exactly 0: each axis has an offset (bias) that the
// attitude estimate would integrate into a steady drift. At boot the IMU driver feeds
// still samples here; the average over GYRO_CAL_SAMPLES becomes the bias that is
// subtracted from every later reading.
//
// SAFETY rules:
// - The window restarts if the drone moves (gyro spread or accel magnitude out of
//   limits) or the result is implausible (bias above GYRO_CAL_MAX_BIAS_DPS). A
//   moving drone would otherwise bake its motion into the bias.
// - There is no timeout: if the drone never stays still, calibration never finishes
//   and arming stays blocked (the driver logs every restart). An unknown bias is
//   never accepted silently.
// - Once calibrated, the bias is kept (also across sensor re-initialization).
// Works on body-frame samples (after axis alignment).
class GyroCalibrator {
public:
    enum class Result : uint8_t {
        IDLE,               // Already calibrated: sample ignored
        COLLECTING,         // Sample accepted, window not full yet
        COMPLETED,          // This sample completed the calibration
        RESTARTED_MOTION,   // Gyro spread too large: the drone moved
        RESTARTED_ACCEL,    // Accel magnitude outside the still window: moved/shaken
        RESTARTED_BIAS,     // Window complete but the bias is implausibly large
    };

    Result onSample(const float acc[3], const float gyro[3]) {
        if (_calibrated) return Result::IDLE;

        const float accNorm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (!(accNorm >= GYRO_CAL_ACC_MIN_G && accNorm <= GYRO_CAL_ACC_MAX_G)) {  // Also NaN
            return restartWith(Result::RESTARTED_ACCEL);
        }

        for (int i = 0; i < 3; i++) {
            if (!std::isfinite(gyro[i])) return restartWith(Result::RESTARTED_MOTION);
            if (_count == 0 || gyro[i] < _min[i]) _min[i] = gyro[i];
            if (_count == 0 || gyro[i] > _max[i]) _max[i] = gyro[i];
            if (_max[i] - _min[i] > GYRO_CAL_MAX_SPREAD_DPS) return restartWith(Result::RESTARTED_MOTION);
            _sum[i] += gyro[i];
        }
        if (++_count < GYRO_CAL_SAMPLES) return Result::COLLECTING;

        float bias[3];
        for (int i = 0; i < 3; i++) {
            bias[i] = _sum[i] / static_cast<float>(_count);
            if (std::fabs(bias[i]) > GYRO_CAL_MAX_BIAS_DPS) return restartWith(Result::RESTARTED_BIAS);
        }
        for (int i = 0; i < 3; i++) _bias[i] = bias[i];
        _calibrated = true;
        return Result::COMPLETED;
    }

    // Discard the current window (e.g. the sensor reported a fault). No effect once calibrated.
    void restart() {
        if (_calibrated) return;
        clearWindow();
    }

    // Subtract the bias in place. Only valid once calibrated.
    void apply(float gyro[3]) const {
        for (int i = 0; i < 3; i++) gyro[i] -= _bias[i];
    }

    bool isCalibrated() const { return _calibrated; }
    float bias(int axis) const { return (axis >= 0 && axis < 3) ? _bias[axis] : 0.0f; }
    uint32_t restarts() const { return _restarts; }
    uint16_t samplesCollected() const { return _count; }

private:
    Result restartWith(Result reason) {
        _restarts++;
        clearWindow();
        return reason;
    }

    void clearWindow() {
        _count = 0;
        for (int i = 0; i < 3; i++) {
            _sum[i] = 0.0f;
            _min[i] = 0.0f;
            _max[i] = 0.0f;
        }
    }

    bool _calibrated = false;
    float _bias[3] = {0.0f, 0.0f, 0.0f};
    float _sum[3] = {0.0f, 0.0f, 0.0f};
    float _min[3] = {0.0f, 0.0f, 0.0f};
    float _max[3] = {0.0f, 0.0f, 0.0f};
    uint16_t _count = 0;
    uint32_t _restarts = 0;
};

inline const char* gyroCalResultName(GyroCalibrator::Result result) {
    switch (result) {
        case GyroCalibrator::Result::IDLE:             return "IDLE";
        case GyroCalibrator::Result::COLLECTING:       return "COLLECTING";
        case GyroCalibrator::Result::COMPLETED:        return "COMPLETED";
        case GyroCalibrator::Result::RESTARTED_MOTION: return "MOTION";
        case GyroCalibrator::Result::RESTARTED_ACCEL:  return "ACCEL_NOT_STILL";
        case GyroCalibrator::Result::RESTARTED_BIAS:   return "BIAS_TOO_LARGE";
        default:                                       return "UNKNOWN";
    }
}
