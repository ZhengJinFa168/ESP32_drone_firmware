#pragma once
#include <cmath>
#include "core/eventService/EventType.h"
#include "config/ArmingConfig.h"

// ============================================
// RC INPUT VALIDATION (pure logic, no hardware)
// ============================================
// Applied by RcInputModule to every packet from any RC provider (WebSocket today,
// CRSF later) before it is published as RC_INPUT.
//
// SAFETY: out-of-range values are REJECTED, never clamped. A corrupted packet that
// happens to clamp to full throttle would otherwise be flown. A rejected packet
// counts as "no packet", so a link that only sends garbage ends in RC failsafe.
// NaN and infinity must be checked explicitly: every range comparison with NaN is
// false, so "x < 0 || x > 1" would let NaN through (and sscanf accepts "nan").

inline bool isRcAxisValid(float value, float min, float max) {
    return std::isfinite(value) && value >= min && value <= max;
}

inline bool isValidRcInput(const RcInputData& rc) {
    if (!rc.linkOk) return false;  // Providers only deliver real pilot packets
    if (!isRcAxisValid(rc.throttle, 0.0f, 1.0f)) return false;
    if (!isRcAxisValid(rc.roll, -1.0f, 1.0f)) return false;
    if (!isRcAxisValid(rc.pitch, -1.0f, 1.0f)) return false;
    if (!isRcAxisValid(rc.yaw, -1.0f, 1.0f)) return false;
    // The arm switch is two-position; anything else is corruption
    if (rc.aux[ARM_AUX_CHANNEL] > 1) return false;
    return true;
}
