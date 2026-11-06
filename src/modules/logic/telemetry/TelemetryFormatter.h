#pragma once
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "core/eventService/EventType.h"
#include "config/ArmingConfig.h"
#include "config/TelemetryConfig.h"

// ============================================
// TELEMETRY FORMATTER (pure logic, no hardware)
// ============================================
// Builds one Teleplot frame (">name:value\n" lines) from the latest known values
// into a caller-provided buffer. No heap.
// Only values that have been received at least once are written.
// If the frame does not fit in the buffer, it returns false and outLen = 0 (nothing
// may be sent): a truncated frame would end in a broken line such as ">ro".
// outLen can also be 0 with a true return: nothing to send yet.

struct TelemetrySnapshot {
    bool haveAttitude = false;
    AttitudeData attitude{};
    bool haveImu = false;
    ImuData imu{};
    bool haveStatus = false;
    ArmingStatusData status{};
    bool haveRc = false;
    RcInputData rc{};
    bool haveLoop = false;
    LoopStatsData loop{};
    bool imuFault = false;
    bool gyroCalibrated = false;
    uint32_t droppedFrames = 0;   // Frames not sent because the TX buffer was full
};

namespace telemetry_detail {
    struct FrameWriter {
        char* buf;
        size_t cap;
        size_t len = 0;
        bool overflow = false;

        FrameWriter(char* b, size_t c) : buf(b), cap(c) {}

        void line(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
            if (overflow) return;
            va_list args;
            va_start(args, fmt);
            const int n = std::vsnprintf(buf + len, cap - len, fmt, args);
            va_end(args);
            if (n < 0 || static_cast<size_t>(n) >= cap - len) {
                overflow = true;   // vsnprintf never writes past cap
                return;
            }
            len += static_cast<size_t>(n);
        }
    };
}

// Returns false if the frame does not fit in `cap` bytes. outLen = frame length (0 on failure).
inline bool formatTelemetry(char* buf, size_t cap, const TelemetrySnapshot& s, uint8_t groups, size_t& outLen) {
    outLen = 0;
    if (!buf || cap == 0) return false;
    buf[0] = '\0';
    telemetry_detail::FrameWriter w(buf, cap);

    if ((groups & TelemetryGroup::ATTITUDE) && s.haveAttitude) {
        w.line(">roll:%.2f\n>pitch:%.2f\n>yaw:%.2f\n", s.attitude.roll, s.attitude.pitch, s.attitude.yaw);
    }
    if ((groups & TelemetryGroup::IMU) && s.haveImu) {
        w.line(">gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", s.imu.gx, s.imu.gy, s.imu.gz);
        w.line(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n", s.imu.ax, s.imu.ay, s.imu.az);
    }
    if (groups & TelemetryGroup::STATUS) {
        if (s.haveStatus) {
            w.line(">armed:%d\n>blockers:%u\n>rcLink:%d\n", s.status.armed ? 1 : 0,
                   static_cast<unsigned>(s.status.blockers), s.status.rcLinkOk ? 1 : 0);
        }
        w.line(">imuFault:%d\n>gyroCal:%d\n>tlmDrop:%lu\n", s.imuFault ? 1 : 0, s.gyroCalibrated ? 1 : 0,
               static_cast<unsigned long>(s.droppedFrames));
    }
    if ((groups & TelemetryGroup::RC) && s.haveRc) {
        w.line(">throttle:%.2f\n>rcRoll:%.2f\n>rcPitch:%.2f\n>rcYaw:%.2f\n>armSwitch:%u\n",
               s.rc.throttle, s.rc.roll, s.rc.pitch, s.rc.yaw, static_cast<unsigned>(s.rc.aux[ARM_AUX_CHANNEL]));
    }

    if ((groups & TelemetryGroup::LOOP) && s.haveLoop) {
        w.line(">loopHz:%lu\n>loopRunMaxUs:%lu\n>loopGapMaxUs:%lu\n>loopOverruns:%lu\n",
               static_cast<unsigned long>(s.loop.rateHz), static_cast<unsigned long>(s.loop.maxRunUs),
               static_cast<unsigned long>(s.loop.maxGapUs), static_cast<unsigned long>(s.loop.overruns));
    }

    if (w.overflow) return false;
    outLen = w.len;
    return true;
}
