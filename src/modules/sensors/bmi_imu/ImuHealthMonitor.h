#pragma once
#include <cstdint>
#include "core/eventService/EventType.h"
#include "config/SensorConfig.h"

// ============================================
// IMU HEALTH MONITOR (pure logic, no hardware)
// ============================================
// Decides whether IMU data can be trusted. Owned by the IMU driver, which feeds it
// every read result and acts on the verdict (publishes SENSOR_FAULT, stops
// publishing data, re-initializes the sensor).
//
// SAFETY rules:
// - Faults are raised after N bad samples IN A ROW, so single glitches that happen
//   in normal operation are tolerated: the all-zero sample right after init, a
//   sample read twice (polling and sensor clocks drift), an isolated bus error.
// - A fault clears only after IMU_HEALTH_RECOVERY_SAMPLES good samples in a row;
//   any new fault condition restarts that count.
// - Recovery (re-init) is attempted at most every IMU_RECOVERY_RETRY_MS and only
//   while no good data is arriving, so a working re-init is not interrupted.
class ImuHealthMonitor {
public:
    enum class Change : uint8_t { NONE, FAULT_RAISED, FAULT_CLEARED };

    static bool isAllZero(const float acc[3], const float gyro[3]) {
        return acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f &&
               gyro[0] == 0.0f && gyro[1] == 0.0f && gyro[2] == 0.0f;
    }

    Change onInitResult(bool ok) {
        return ok ? Change::NONE : raise(SensorFaultReason::INIT_FAILED);
    }

    Change onSample(const float acc[3], const float gyro[3]) {
        _readErrors = 0;

        const bool allZero = isAllZero(acc, gyro);
        const bool repeat = _haveLast && sameAsLast(acc, gyro);
        store(acc, gyro);

        _zeroRun = allZero ? _zeroRun + 1 : 0;
        _sameRun = repeat ? _sameRun + 1 : 1;

        if (_zeroRun >= IMU_HEALTH_ZERO_SAMPLES) return raise(SensorFaultReason::ZERO_DATA);
        if (_sameRun >= IMU_HEALTH_FROZEN_SAMPLES) return raise(SensorFaultReason::FROZEN_DATA);

        // Only fresh, non-zero samples count towards recovery; single glitches
        // neither count nor reset it.
        if (_faulty && !allZero && !repeat && ++_healthyRun >= IMU_HEALTH_RECOVERY_SAMPLES) {
            return clear();
        }
        return Change::NONE;
    }

    Change onReadError() {
        if (++_readErrors >= IMU_HEALTH_READ_ERROR_SAMPLES) return raise(SensorFaultReason::READ_ERRORS);
        return Change::NONE;
    }

    // The driver found the sensor's configuration registers reset (e.g. after a power glitch)
    Change onResetDetected() { return raise(SensorFaultReason::RESET_DETECTED); }

    bool shouldAttemptRecovery(uint32_t nowMs) const {
        if (!_faulty || _healthyRun > 0) return false;
        return !_attempted || (nowMs - _lastAttemptMs) >= IMU_RECOVERY_RETRY_MS;
    }

    void onRecoveryAttempt(uint32_t nowMs) {
        _attempted = true;
        _lastAttemptMs = nowMs;
    }

    bool isFaulty() const { return _faulty; }
    SensorFaultReason reason() const { return _reason; }
    uint32_t faultCount() const { return _faultCount; }

private:
    Change raise(SensorFaultReason reason) {
        _healthyRun = 0;
        if (_faulty) return Change::NONE;  // Keep the original cause until cleared
        _faulty = true;
        _reason = reason;
        _faultCount++;
        return Change::FAULT_RAISED;
    }

    Change clear() {
        _faulty = false;
        _reason = SensorFaultReason::NONE;
        _healthyRun = 0;
        return Change::FAULT_CLEARED;
    }

    bool sameAsLast(const float acc[3], const float gyro[3]) const {
        for (int i = 0; i < 3; i++) {
            if (acc[i] != _lastAcc[i] || gyro[i] != _lastGyro[i]) return false;
        }
        return true;
    }

    void store(const float acc[3], const float gyro[3]) {
        for (int i = 0; i < 3; i++) {
            _lastAcc[i] = acc[i];
            _lastGyro[i] = gyro[i];
        }
        _haveLast = true;
    }

    bool _faulty = false;
    SensorFaultReason _reason = SensorFaultReason::NONE;
    uint32_t _faultCount = 0;

    uint16_t _zeroRun = 0;
    uint16_t _sameRun = 0;
    uint16_t _readErrors = 0;
    uint16_t _healthyRun = 0;

    float _lastAcc[3] = {0.0f, 0.0f, 0.0f};
    float _lastGyro[3] = {0.0f, 0.0f, 0.0f};
    bool _haveLast = false;

    bool _attempted = false;
    uint32_t _lastAttemptMs = 0;
};

inline const char* sensorFaultReasonName(SensorFaultReason reason) {
    switch (reason) {
        case SensorFaultReason::NONE:           return "NONE";
        case SensorFaultReason::INIT_FAILED:    return "INIT_FAILED";
        case SensorFaultReason::ZERO_DATA:      return "ZERO_DATA";
        case SensorFaultReason::FROZEN_DATA:    return "FROZEN_DATA";
        case SensorFaultReason::READ_ERRORS:    return "READ_ERRORS";
        case SensorFaultReason::RESET_DETECTED: return "RESET_DETECTED";
        default:                                return "UNKNOWN";
    }
}
