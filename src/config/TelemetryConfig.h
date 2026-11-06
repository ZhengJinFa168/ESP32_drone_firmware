#pragma once
#include <cstddef>
#include <cstdint>

// ============================================
// TELEMETRY (Teleplot over the USB serial port)
// ============================================
// One frame of ">name:value" lines is sent every 1000 / TELEMETRY_RATE_HZ ms.
// Frames are dropped (and counted, see >tlmDrop) when the serial TX buffer is full:
// telemetry never blocks the flight loop. Teleplot's baud rate must match LOG_BAUD
// (921600, src/platform/esp32/Esp32Log.h).

constexpr bool TELEMETRY_ENABLED = true;
constexpr uint32_t TELEMETRY_RATE_HZ = 50;   // Plenty to watch live; use the blackbox (later) for PID tuning

// Groups of values to send (TelemetryGroup bit flags)
namespace TelemetryGroup {
    constexpr uint8_t ATTITUDE = 1 << 0;  // roll, pitch, yaw (deg)
    constexpr uint8_t IMU      = 1 << 1;  // gyro (deg/s, bias removed), accel (g), body frame
    constexpr uint8_t STATUS   = 1 << 2;  // armed, blockers, rcLink, imuFault, gyroCal, tlmDrop
    constexpr uint8_t RC       = 1 << 3;  // throttle, sticks, arm switch
    constexpr uint8_t LOOP     = 1 << 4;  // loop rate, longest pass/gap, overruns (LOOP_STATS, once per second)
    constexpr uint8_t ALL      = ATTITUDE | IMU | STATUS | RC | LOOP;
}
constexpr uint8_t TELEMETRY_GROUPS = TelemetryGroup::ALL;

// Largest frame (bytes). All groups: 359 B worst case (all values at their widest); a frame that does not fit is not sent
// (loud error) rather than sent truncated.
constexpr size_t TELEMETRY_FRAME_MAX = 384;

static_assert(TELEMETRY_RATE_HZ > 0 && TELEMETRY_RATE_HZ <= 1000, "TELEMETRY_RATE_HZ must be 1..1000");
