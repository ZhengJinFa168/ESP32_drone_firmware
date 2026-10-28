#pragma once
#include <cmath>
#include <cstdint>
#include "core/eventService/EventType.h"
#include "config/ArmingConfig.h"

// ============================================
// ARMING STATE MACHINE (pure logic, no hardware)
// ============================================
// States: DISARMED / ARMED. Arming is allowed only when no ArmingBlocker is set
// and the pilot turns the arm switch ON (after it was seen OFF).
//
// SAFETY rules implemented here:
// - Default state is DISARMED; every uncertain input blocks arming.
// - The arm switch must be seen OFF after boot, after every disarm and after every
//   blocked arming attempt, so a switch left ON never arms the drone by itself.
// - Immediate disarm: sensor fault, stale attitude, crash, arm switch OFF.
// - RC link loss does NOT disarm immediately: the drone stays armed so flight logic
//   can hold and descend (see rcLostForMs()); arming disarms after RC_LOSS_MAX_ARMED_MS.
//
// Inputs are recorded by the on*() methods; all decisions happen in update(nowMs).
// Times are milliseconds from any monotonic clock; unsigned subtraction keeps the
// comparisons correct across uint32_t wrap-around.
class ArmingLogic {
public:
    enum class Transition : uint8_t { NONE, ARMED, DISARMED };

    explicit ArmingLogic(bool alignmentVerified) : _alignmentVerified(alignmentVerified) {}

    void onRcInput(const RcInputData& rc, uint32_t nowMs) {
        _rc = rc;
        _haveRc = true;
        _lastRcMs = nowMs;
    }

    void onAttitude(const AttitudeData& attitude, uint32_t nowMs) {
        _attitude = attitude;
        _haveAttitude = true;
        _lastAttitudeMs = nowMs;
    }

    // Faults are tracked per sensor: one sensor reporting OK never clears another's fault
    void onSensorFault(SensorId sensor, bool fault) {
        const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(sensor));
        _sensorFaults = fault ? (_sensorFaults | bit) : (_sensorFaults & ~bit);
    }

    Transition update(uint32_t nowMs) {
        const bool rcGoodNow = _haveRc && _rc.linkOk && (nowMs - _lastRcMs) <= RC_LINK_TIMEOUT_MS;
        updateLinkState(rcGoodNow, nowMs);

        // Switch position is only trusted from a valid, fresh packet
        if (rcGoodNow) {
            _switchOn = _rc.aux[ARM_AUX_CHANNEL] != 0;
            if (!_switchOn) _switchSeenOff = true;
        }

        const bool attitudeFresh = _haveAttitude && (nowMs - _lastAttitudeMs) <= ATTITUDE_STALE_MS;
        updateCrashDetection(attitudeFresh, nowMs);
        _blockers = computeBlockers(attitudeFresh);

        return _armed ? updateArmed(rcGoodNow, attitudeFresh, nowMs)
                      : updateDisarmed(rcGoodNow, nowMs);
    }

    bool isArmed() const { return _armed; }
    uint16_t blockers() const { return _blockers; }
    DisarmReason lastDisarmReason() const { return _lastDisarmReason; }
    bool rcLinkOk() const { return _rcLinkOk; }

    // How long the RC link has been lost (0 while the link is OK). Flight logic uses
    // this while armed: hold for RC_LOSS_HOLD_MS, then descend.
    uint32_t rcLostForMs(uint32_t nowMs) const { return _rcLinkOk ? 0 : nowMs - _rcLostSinceMs; }

    ArmingStatusData status() const { return {_armed, _blockers, _lastDisarmReason, _rcLinkOk}; }

private:
    // Link counts as lost as soon as packets stop or are flagged invalid, and only
    // counts as restored after RC_LINK_STABLE_MS of continuous good packets.
    void updateLinkState(bool rcGoodNow, uint32_t nowMs) {
        if (_rcLinkOk) {
            if (!rcGoodNow) {
                _rcLinkOk = false;
                _rcLostSinceMs = nowMs;
                _recovering = false;
            }
            return;
        }
        if (!rcGoodNow) {
            _recovering = false;
            return;
        }
        if (!_recovering) {
            _recovering = true;
            _rcGoodSinceMs = nowMs;
        }
        if (nowMs - _rcGoodSinceMs >= RC_LINK_STABLE_MS) {
            _rcLinkOk = true;
            _recovering = false;
        }
    }

    void updateCrashDetection(bool attitudeFresh, uint32_t nowMs) {
        const bool overTilt = attitudeFresh &&
            (std::fabs(_attitude.roll) > CRASH_TILT_DEG || std::fabs(_attitude.pitch) > CRASH_TILT_DEG);
        if (!overTilt) {
            _overTilt = false;
        } else if (!_overTilt) {
            _overTilt = true;
            _overTiltSinceMs = nowMs;
        }
    }

    uint16_t computeBlockers(bool attitudeFresh) const {
        uint16_t b = 0;
        if (!_alignmentVerified) b |= ArmingBlocker::ALIGNMENT_NOT_VERIFIED;
        if (!_rcLinkOk) b |= ArmingBlocker::NO_RC_LINK;
        if (!_haveRc || _rc.throttle > ARM_MAX_THROTTLE) b |= ArmingBlocker::THROTTLE_NOT_LOW;
        if (!_switchSeenOff) b |= ArmingBlocker::ARM_SWITCH_NOT_RESET;
        if (!attitudeFresh) {
            b |= ArmingBlocker::ATTITUDE_STALE;
        } else if (std::fabs(_attitude.roll) > ARM_MAX_TILT_DEG || std::fabs(_attitude.pitch) > ARM_MAX_TILT_DEG) {
            b |= ArmingBlocker::NOT_LEVEL;
        }
        if (_sensorFaults != 0) b |= ArmingBlocker::SENSOR_FAULT;
        return b;
    }

    Transition updateDisarmed(bool rcGoodNow, uint32_t nowMs) {
        if (!(rcGoodNow && _switchOn)) return Transition::NONE;

        if (_blockers != 0) {
            // SAFETY: a blocked attempt needs a fresh OFF→ON toggle, so the drone never
            // arms by surprise the moment the last blocker clears.
            _switchSeenOff = false;
            _blockers |= ArmingBlocker::ARM_SWITCH_NOT_RESET;
            return Transition::NONE;
        }

        _armed = true;
        _armedAtMs = nowMs;
        _autoDisarmPending = true;
        _lastDisarmReason = DisarmReason::NONE;
        return Transition::ARMED;
    }

    Transition updateArmed(bool rcGoodNow, bool attitudeFresh, uint32_t nowMs) {
        // Throttle raised once = the pilot is flying; auto-disarm no longer applies
        if (_autoDisarmPending && rcGoodNow && _rc.throttle > ARM_MAX_THROTTLE) {
            _autoDisarmPending = false;
        }

        // SAFETY: checked in order of severity
        if (_sensorFaults != 0) return disarm(DisarmReason::SENSOR_FAULT);
        if (!attitudeFresh) return disarm(DisarmReason::ATTITUDE_STALE);
        if (_overTilt && nowMs - _overTiltSinceMs >= CRASH_TIME_MS) return disarm(DisarmReason::CRASH);
        if (rcGoodNow && !_switchOn) return disarm(DisarmReason::SWITCH_OFF);
        if (!_rcLinkOk && nowMs - _rcLostSinceMs >= RC_LOSS_MAX_ARMED_MS) return disarm(DisarmReason::RC_LOSS_TIMEOUT);
        if (_autoDisarmPending && nowMs - _armedAtMs >= AUTO_DISARM_MS) return disarm(DisarmReason::AUTO_DISARM);
        return Transition::NONE;
    }

    Transition disarm(DisarmReason reason) {
        _armed = false;
        _lastDisarmReason = reason;
        _autoDisarmPending = false;
        // SAFETY: re-arming always requires the switch to be turned OFF first
        _switchSeenOff = false;
        _blockers |= ArmingBlocker::ARM_SWITCH_NOT_RESET;
        return Transition::DISARMED;
    }

    const bool _alignmentVerified;
    bool _armed = false;
    uint16_t _blockers = 0;
    DisarmReason _lastDisarmReason = DisarmReason::NONE;

    // RC
    RcInputData _rc{};
    bool _haveRc = false;
    uint32_t _lastRcMs = 0;
    bool _rcLinkOk = false;
    uint32_t _rcLostSinceMs = 0;
    bool _recovering = false;
    uint32_t _rcGoodSinceMs = 0;
    bool _switchOn = false;
    bool _switchSeenOff = false;

    // Attitude / sensors
    AttitudeData _attitude{};
    bool _haveAttitude = false;
    uint32_t _lastAttitudeMs = 0;
    uint8_t _sensorFaults = 0;  // One bit per SensorId
    bool _overTilt = false;
    uint32_t _overTiltSinceMs = 0;

    // Auto-disarm
    uint32_t _armedAtMs = 0;
    bool _autoDisarmPending = false;
};

// Human-readable names for logging and telemetry
inline const char* armingBlockerName(uint16_t bit) {
    switch (bit) {
        case ArmingBlocker::ALIGNMENT_NOT_VERIFIED: return "ALIGNMENT_NOT_VERIFIED";
        case ArmingBlocker::NO_RC_LINK:             return "NO_RC_LINK";
        case ArmingBlocker::THROTTLE_NOT_LOW:       return "THROTTLE_NOT_LOW";
        case ArmingBlocker::ARM_SWITCH_NOT_RESET:   return "ARM_SWITCH_NOT_RESET";
        case ArmingBlocker::NOT_LEVEL:              return "NOT_LEVEL";
        case ArmingBlocker::ATTITUDE_STALE:         return "ATTITUDE_STALE";
        case ArmingBlocker::SENSOR_FAULT:           return "SENSOR_FAULT";
        default:                                    return "UNKNOWN";
    }
}

inline const char* disarmReasonName(DisarmReason reason) {
    switch (reason) {
        case DisarmReason::NONE:            return "NONE";
        case DisarmReason::SWITCH_OFF:      return "SWITCH_OFF";
        case DisarmReason::RC_LOSS_TIMEOUT: return "RC_LOSS_TIMEOUT";
        case DisarmReason::SENSOR_FAULT:    return "SENSOR_FAULT";
        case DisarmReason::ATTITUDE_STALE:  return "ATTITUDE_STALE";
        case DisarmReason::CRASH:           return "CRASH";
        case DisarmReason::AUTO_DISARM:     return "AUTO_DISARM";
        default:                            return "UNKNOWN";
    }
}
