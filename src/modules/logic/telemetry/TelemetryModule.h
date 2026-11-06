#pragma once
#include <cstdint>
#include "core/Module.h"
#include "core/Log.h"
#include "core/Clock.h"
#include "config/TelemetryConfig.h"
#include "TelemetryFormatter.h"
#include "TelemetrySender.h"

// Streams live values to Teleplot.
// Subscribes: IMU_RAW, ATTITUDE_UPDATED, ARMING_STATUS, SENSOR_FAULT, GYRO_CALIBRATION,
// RC_INPUT, LOOP_STATS
// Collector: callbacks only store the latest values in a snapshot; once per period the
// snapshot goes to the output, which formats and writes it (never blocks).
// TOutput (compile time, no chip code here): begin(), send(const TelemetrySnapshot&),
// droppedFrames(), oversizeFrames().
// - Board: Esp32TelemetryOutput (formats on the core-0 serial writer task)
// - Native tests: TelemetrySender (formats and writes in send())
template <typename TOutput>
class TelemetryModule : public Module {
private:
    static constexpr uint32_t PERIOD_MS = 1000u / TELEMETRY_RATE_HZ;

    TelemetrySnapshot _snap;
    TOutput _output;
    uint32_t _lastFrameMs = 0;

    static uint32_t nowMs() { return clockMs(); }

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

    static void onLoopStats(void* ctx, const void* data) {
        if (!data) return;
        self(ctx)->_snap.loop = *static_cast<const LoopStatsData*>(data);
        self(ctx)->_snap.haveLoop = true;
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
        subscribe(EventType::LOOP_STATS, onLoopStats);
        _output.begin();
        _lastFrameMs = nowMs();
    }

    void loop() override {
        if (!TELEMETRY_ENABLED) return;
        const uint32_t now = nowMs();
        if (now - _lastFrameMs < PERIOD_MS) return;
        _lastFrameMs = now;
        _output.send(_snap);
    }

    uint32_t droppedFrames() const { return _output.droppedFrames(); }
    uint32_t oversizeFrames() const { return _output.oversizeFrames(); }
    const TOutput& output() const { return _output; }  // Tests
};
