#pragma once
#include <cstdint>
#include "core/Module.h"
#include "core/Log.h"
#include "config/TelemetryConfig.h"
#include "TelemetryFormatter.h"

// Streams live values to Teleplot.
// Subscribes: IMU_RAW, ATTITUDE_UPDATED, ARMING_STATUS, SENSOR_FAULT, GYRO_CALIBRATION, RC_INPUT
// Callbacks only store the latest value; loop() formats and sends one frame per
// period. SAFETY: output never blocks the flight loop — a frame that does not fit
// in the serial TX buffer is dropped and counted (>tlmDrop).
class TelemetryModule : public Module {
private:
    static constexpr uint32_t PERIOD_MS = 1000u / TELEMETRY_RATE_HZ;

    TelemetrySnapshot _snap;
    char _frame[TELEMETRY_FRAME_MAX];
    uint32_t _lastFrameMs = 0;
    uint32_t _oversizeFrames = 0;

    static uint32_t nowMs() { return static_cast<uint32_t>(millis()); }

    static TelemetryModule* self(void* ctx) { return static_cast<TelemetryModule*>(ctx); }

    static void onImu(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.imu = *static_cast<const ImuData*>(data);
        self(ctx)->_snap.haveImu = true;
    }

    static void onAttitude(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.attitude = *static_cast<const AttitudeData*>(data);
        self(ctx)->_snap.haveAttitude = true;
    }

    static void onArmingStatus(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.status = *static_cast<const ArmingStatusData*>(data);
        self(ctx)->_snap.haveStatus = true;
    }

    static void onSensorFault(void* ctx, const void* data) {
        if (!data) return;
        const auto* fault = static_cast<const SensorFaultData*>(data);
        if (fault->sensor == SensorId::IMU) self(ctx)->_snap.imuFault = fault->fault;
    }

    static void onGyroCalibration(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.gyroCalibrated = static_cast<const GyroCalibrationData*>(data)->calibrated;
    }

    static void onRcInput(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.rc = *static_cast<const RcInputData*>(data);
        self(ctx)->_snap.haveRc = true;
    }

public:
    TelemetryModule() : Module("TELEMETRY") {}

    void setup() override {
        if (!TELEMETRY_ENABLED) return;
        subscribe(EventType::IMU_RAW, onImu);
        subscribe(EventType::ATTITUDE_UPDATED, onAttitude);
        subscribe(EventType::ARMING_STATUS, onArmingStatus);
        subscribe(EventType::SENSOR_FAULT, onSensorFault);
        subscribe(EventType::GYRO_CALIBRATION, onGyroCalibration);
        subscribe(EventType::RC_INPUT, onRcInput);
        _lastFrameMs = nowMs();
    }

    void loop() override {
        if (!TELEMETRY_ENABLED) return;
        const uint32_t now = nowMs();
        if (now - _lastFrameMs < PERIOD_MS) return;
        _lastFrameMs = now;

        size_t len = 0;
        if (!formatTelemetry(_frame, sizeof(_frame), _snap, TELEMETRY_GROUPS, len)) {
            // Configuration error (too many values for TELEMETRY_FRAME_MAX): loud, once
            if (_oversizeFrames++ == 0) {
                LOG_E("TELEMETRY", "frame larger than TELEMETRY_FRAME_MAX (%u B), telemetry not sent",
                      static_cast<unsigned>(TELEMETRY_FRAME_MAX));
            }
            return;
        }
        if (len == 0) return;  // Nothing received yet
        if (!logTryWrite(_frame, len)) _snap.droppedFrames++;
    }

    uint32_t droppedFrames() const { return _snap.droppedFrames; }
    uint32_t oversizeFrames() const { return _oversizeFrames; }
};
