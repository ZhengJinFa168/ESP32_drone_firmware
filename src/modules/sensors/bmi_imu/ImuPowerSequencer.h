#pragma once
#include <cstdint>
#include "config/SensorConfig.h"

// ============================================
// IMU POWER SEQUENCER (pure logic, no hardware)
// ============================================
// Times one power cycle of the IMU module: power off for IMU_POWER_OFF_MS, power on,
// wait IMU_POWER_ON_SETTLE_MS, then the driver may initialize the sensor.
// The driver performs the returned actions (pins, I2C); this class only decides when.
// Non-blocking: the caller polls update() from its loop, so the waits never stall
// other modules (no delay()).
//
// Why a full power cycle: a BMI270 that got partially powered (e.g. back-fed through
// SDA/SCL during a power dip) comes back stuck (CHIP_ID 0x20, init fails); only a real
// power-off clears it. Bench-verified 2026-11-06.
class ImuPowerSequencer {
public:
    enum class Action : uint8_t {
        NONE,           // Nothing to do now
        CUT_POWER,      // Drive SDA/SCL LOW, INT without pull-up, power pin LOW
        RESTORE_POWER,  // Release SDA/SCL, power pin HIGH
        INIT_SENSOR,    // Start I2C and initialize the sensor
    };

    // Starts a power cycle. Returns CUT_POWER, or NONE if a cycle is already running
    // (a second request must not shorten the off time of the current one).
    Action start(uint32_t nowMs) {
        if (_state != State::IDLE) return Action::NONE;
        _state = State::POWERED_OFF;
        _sinceMs = nowMs;
        _cycles++;
        return Action::CUT_POWER;
    }

    // Call every loop while busy(). Each action is returned exactly once.
    // Times are unsigned differences, so millis() wrap-around is handled.
    Action update(uint32_t nowMs) {
        switch (_state) {
            case State::POWERED_OFF:
                if (nowMs - _sinceMs < IMU_POWER_OFF_MS) return Action::NONE;
                _state = State::SETTLING;
                _sinceMs = nowMs;  // Settle time counts from the actual power-on
                return Action::RESTORE_POWER;
            case State::SETTLING:
                if (nowMs - _sinceMs < IMU_POWER_ON_SETTLE_MS) return Action::NONE;
                _state = State::IDLE;
                return Action::INIT_SENSOR;
            default:
                return Action::NONE;
        }
    }

    // True from start() until INIT_SENSOR has been returned: the sensor must not be accessed.
    bool busy() const { return _state != State::IDLE; }
    uint32_t cycles() const { return _cycles; }

private:
    enum class State : uint8_t { IDLE, POWERED_OFF, SETTLING };

    State _state = State::IDLE;
    uint32_t _sinceMs = 0;
    uint32_t _cycles = 0;
};
