#pragma once
#include <cstdint>
#include "config/ArmingConfig.h"
#include "config/SensorConfig.h"

// Include the available filters
#include "modules/logic/attitude_estimator/ComplementaryFilter.h"
// #include "modules/logic/attitude_estimator/MadgwickFilter.h" // Future use

// Select the active filter type
using ActiveAttitudeFilter = ComplementaryFilter;

// ============================================
// FLIGHT LOOP TIMING
// ============================================
// One loop pass per IMU sample (BMI270 at 400 Hz). A pass longer than this period is
// an overrun: the next sample is late and the control loop timing degrades.
// Stage 2 pass criteria (phone connected and sending, telemetry on):
//   loopHz 400 +/- 2, loopGapMaxUs <= 3000, loopOverruns 0, no IMU NO_DATA_READY fault.
constexpr uint32_t FLIGHT_LOOP_PERIOD_US = 2500;

// ============================================
// FLIGHT TASK (src/main.cpp)
// ============================================
// The whole App (every module, the event bus) runs in one FreeRTOS task on core 1,
// woken by the IMU data-ready interrupt. Core 0 runs WiFi and AsyncTCP.
// Priority 20: above every Arduino/WiFi task on core 1 (loopTask is 1), below the
// ESP-IDF system tasks (esp_timer 22, ipc 24).
constexpr int FLIGHT_TASK_CORE = 1;
constexpr uint32_t FLIGHT_TASK_PRIORITY = 20;
constexpr uint32_t FLIGHT_TASK_STACK_BYTES = 8192;
// Without an interrupt the task still runs every FLIGHT_TASK_WAIT_MS, so the modules
// keep running (arming, RC, telemetry) and the IMU driver can detect the missing INT.
constexpr uint32_t FLIGHT_TASK_WAIT_MS = 8;

static_assert(FLIGHT_TASK_WAIT_MS * 1000 > FLIGHT_LOOP_PERIOD_US,
              "The timeout must be longer than one sample, or it competes with the interrupt");
static_assert(FLIGHT_TASK_WAIT_MS < IMU_DRDY_MISSING_MS,
              "The task must run at least once within IMU_DRDY_MISSING_MS to detect a missing INT");

// ============================================
// ATTITUDE ESTIMATION
// ============================================
// Complementary filter time constant: the accelerometer corrects roll/pitch over
// about this many seconds. Larger = less motor vibration in the attitude, slower
// drift correction. [TO TUNE on hardware]
constexpr float ATTITUDE_FILTER_TAU_S = 0.5f;
static_assert(ATTITUDE_FILTER_TAU_S > 0.0f, "ATTITUDE_FILTER_TAU_S must be > 0");

// A gap between IMU samples longer than this means data was missing (fault,
// recovery, stall): the estimator re-initializes from the accelerometer and logs a
// warning instead of integrating across the gap.
constexpr uint32_t ESTIMATOR_MAX_GAP_MS = 50;

// SAFETY: a reset makes the attitude jump. The estimator therefore does NOT publish
// the sample it resets on, and every gap long enough to cause a reset must also be
// long enough to count as stale: arming then disarms (ATTITUDE_STALE) on its next
// check, even if the whole loop stalled and arming could not run during the gap.
static_assert(ESTIMATOR_MAX_GAP_MS >= ATTITUDE_STALE_MS,
              "ESTIMATOR_MAX_GAP_MS must be >= ATTITUDE_STALE_MS");
