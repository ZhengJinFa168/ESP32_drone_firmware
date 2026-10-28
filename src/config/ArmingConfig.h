#pragma once
#include <cstdint>

// ============================================
// ARMING / DISARMING THRESHOLDS
// ============================================
// SAFETY: these values decide when the motors may run. Change them deliberately
// and re-run test_arming after any change.

// RC channel used as the arm switch (RcInputData::aux index, non-zero = ON)
constexpr uint8_t ARM_AUX_CHANNEL = 0;

// --- Arming conditions ---
constexpr float ARM_MAX_THROTTLE = 0.05f;      // Throttle must be below 5% to arm
constexpr float ARM_MAX_TILT_DEG = 25.0f;      // Roll and pitch within ±25° to arm

// --- RC link ---
constexpr uint32_t RC_LINK_TIMEOUT_MS = 500;   // No valid packet for this long = link lost
constexpr uint32_t RC_LINK_STABLE_MS = 1000;   // Link must be good this long to count as restored

// --- RC loss while armed ---
// The drone stays armed: flight logic holds for RC_LOSS_HOLD_MS, then descends.
// Arming disarms after RC_LOSS_MAX_ARMED_MS (touchdown detection will disarm earlier, Stage 4).
constexpr uint32_t RC_LOSS_HOLD_MS = 3000;
constexpr uint32_t RC_LOSS_MAX_ARMED_MS = 20000;

// --- Immediate disarm conditions ---
constexpr uint32_t ATTITUDE_STALE_MS = 50;     // Attitude older than this = estimator/sensor dead
constexpr float CRASH_TILT_DEG = 80.0f;        // Tilt beyond this...
constexpr uint32_t CRASH_TIME_MS = 200;        // ...for this long = crash

// --- Auto-disarm ---
// Armed but throttle never raised above ARM_MAX_THROTTLE within this time = disarm
// (prevents walking up to an armed, idling drone).
constexpr uint32_t AUTO_DISARM_MS = 5000;
