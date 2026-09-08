#pragma once
#include <Arduino.h>
#include <utility>
#include "core/Module.h"

template <typename TFilter>
class AttitudeEstimatorModule : public Module {
private:
    TFilter _filter;
    unsigned long _lastTimeUs = 0;
    bool _telemetryStream = false;

    // Static C-style callback — no heap allocation, fully inlineable
    static void onImuRaw(void* ctx, const void* data) {
        if (!data) return;
        auto* self = static_cast<AttitudeEstimatorModule*>(ctx);
        const auto* imu = static_cast<const ImuData*>(data);

        unsigned long now = micros();
        float dt = 0.002f; // Default 500 Hz (2ms) fallback
        if (self->_lastTimeUs > 0 && now > self->_lastTimeUs) {
            dt = static_cast<float>(now - self->_lastTimeUs) * 1e-6f;
            // Clamp dt to avoid glitches if execution paused
            if (dt > 0.05f) dt = 0.002f;
        }
        self->_lastTimeUs = now;

        self->_filter.update(imu->ax, imu->ay, imu->az, imu->gx, imu->gy, imu->gz, dt);

        AttitudeData attitude{self->_filter.getRoll(), self->_filter.getPitch(), self->_filter.getYaw()};
        self->publish(EventType::ATTITUDE_UPDATED, &attitude);

        // Output CSV format "roll,pitch,yaw" for scripts/plot_telemetry.py if enabled
        if (self->_telemetryStream) {
            Serial.print(attitude.roll);
            Serial.print(",");
            Serial.print(attitude.pitch);
            Serial.print(",");
            Serial.println(attitude.yaw);
        }
    }

public:
    template <typename... Args>
    explicit AttitudeEstimatorModule(Args&&... args)
        : Module("ATTITUDE_ESTIMATOR"), _filter(std::forward<Args>(args)...) {}

    void setup() override {
        _filter.reset();
        _lastTimeUs = 0;
        subscribe(EventType::IMU_RAW, onImuRaw);
    }

    // loop() not needed — fully event-driven, uses default empty base implementation

    float getRoll() const { return _filter.getRoll(); }
    float getPitch() const { return _filter.getPitch(); }
    float getYaw() const { return _filter.getYaw(); }
    void reset() { _filter.reset(); _lastTimeUs = 0; }
    void enableTelemetryStream(bool enable) { _telemetryStream = enable; }
};
