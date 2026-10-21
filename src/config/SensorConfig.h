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

static_assert(isValidRotation(IMU_ALIGNMENT),
              "IMU_ALIGNMENT is not a proper rotation (mirror image or invalid axes)");
