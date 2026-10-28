#pragma once
#include "utils/AxisAlignment.h"

// ============================================
// IMU MOUNTING ALIGNMENT
// ============================================
// How the BMI270 is mounted relative to the drone body (FLU: X forward, Y left, Z up).
// Determine it with the gravity test on the MOUNTED sensor
// (see "End-of-Build Checklist" in .agents/rules/hardware_map.md).
constexpr AxisAlignment IMU_ALIGNMENT = Align::CW0;

// SAFETY: a wrong alignment makes the controller correct tilt in the wrong
// direction, flipping the drone at takeoff. Set this to true ONLY after the
// gravity test confirms IMU_ALIGNMENT on the mounted sensor. The arming logic
// must refuse to arm while this is false.
constexpr bool IMU_ALIGNMENT_VERIFIED = false;

// ============================================
// IMU HEALTH MONITORING
// ============================================
// Counts are in samples (400 Hz → 2.5 ms each). A fault is raised after N bad
// samples in a row and cleared only after IMU_HEALTH_RECOVERY_SAMPLES good ones.
constexpr uint16_t IMU_HEALTH_ZERO_SAMPLES = 5;         // All axes exactly 0 (sensor reset) — ~12 ms
constexpr uint16_t IMU_HEALTH_FROZEN_SAMPLES = 20;      // Identical samples (data stopped) — ~50 ms
constexpr uint16_t IMU_HEALTH_READ_ERROR_SAMPLES = 10;  // Consecutive read failures — ~25 ms
constexpr uint16_t IMU_HEALTH_RECOVERY_SAMPLES = 400;   // Healthy samples before the fault clears — ~1 s
constexpr uint32_t IMU_RECOVERY_RETRY_MS = 1000;        // Minimum time between re-initialization attempts
constexpr uint32_t IMU_REG_CHECK_MS = 100;              // How often the driver checks the power-control register

static_assert(isValidRotation(IMU_ALIGNMENT),
              "IMU_ALIGNMENT is not a proper rotation (mirror image or invalid axes)");
