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
// The sensor is read only on its data-ready interrupt (INT, 400 Hz). No interrupt for
// this long (10 samples) = no data: NO_DATA_READY fault, recovery by power cycle.
// SAFETY: missing IMU data means the drone cannot fly level; today the fault disarms,
// the in-flight reaction (land) is decided in Stage 4.
constexpr uint32_t IMU_DRDY_MISSING_MS = 25;

// ============================================
// IMU POWER CYCLE (module powered from PIN_IMU_POWER)
// ============================================
// Every boot and every recovery attempt power-cycles the sensor: a sensor stuck after
// a power dip (CHIP_ID 0x20) is cleared only by a real power-off.
// Bench-measured 2026-11-06: 1 ms off already gives a real power-on reset; 50 ms is margin.
constexpr uint32_t IMU_POWER_OFF_MS = 50;         // Power pin LOW (SDA/SCL LOW) for this long
constexpr uint32_t IMU_POWER_ON_SETTLE_MS = 10;   // Wait after power-on before talking to the sensor

static_assert(IMU_POWER_OFF_MS > 0 && IMU_POWER_ON_SETTLE_MS > 0, "IMU power cycle times must be > 0");
static_assert(IMU_POWER_OFF_MS + IMU_POWER_ON_SETTLE_MS < IMU_RECOVERY_RETRY_MS,
              "A power cycle must finish before the next recovery attempt is due");

// ============================================
// GYRO CALIBRATION (at boot, drone must be still)
// ============================================
// Engineering estimates [TO TUNE on hardware]: log the gyro with the board still
// (Teleplot) and check the noise spread and the offset against these limits.
constexpr uint16_t GYRO_CAL_SAMPLES = 400;           // Still samples averaged (1 s at 400 Hz)
constexpr float GYRO_CAL_MAX_SPREAD_DPS = 2.0f;      // Max - min per axis during the window (else: moving)
constexpr float GYRO_CAL_ACC_MIN_G = 0.9f;           // Accel magnitude window: outside = being moved/shaken
constexpr float GYRO_CAL_ACC_MAX_G = 1.1f;
constexpr float GYRO_CAL_MAX_BIAS_DPS = 5.0f;        // Sanity limit; a healthy BMI270 offset is well below 1 deg/s
constexpr uint32_t GYRO_CAL_LOG_INTERVAL_MS = 1000;  // Restart warnings at most this often

static_assert(GYRO_CAL_SAMPLES > 0, "GYRO_CAL_SAMPLES must be > 0");

static_assert(isValidRotation(IMU_ALIGNMENT),
              "IMU_ALIGNMENT is not a proper rotation (mirror image or invalid axes)");
