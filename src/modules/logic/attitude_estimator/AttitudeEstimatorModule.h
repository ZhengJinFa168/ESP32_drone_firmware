#pragma once
#include <cstdint>
#include <utility>
#include "core/Module.h"
#include "core/Log.h"
#include "config/FlightConfig.h"

// Turns IMU_RAW into ATTITUDE_UPDATED with the configured filter.
// dt comes from the sample timestamps set by the IMU driver (ImuData::timestampUs),
// not from when the event is handled, so loop delays do not distort the integration.
// Uses no hardware clock itself.
template <typename TFilter>
class AttitudeEstimatorModule : public Module {
private:
    TFilter _filter;
    uint32_t _lastSampleUs = 0;
    bool _haveLastSample = false;
    uint32_t _gapResets = 0;

    static constexpr uint32_t MAX_GAP_US = ESTIMATOR_MAX_GAP_MS * 1000u;

    // Static C-style callback — no heap allocation, fully inlineable
    static void onImuRaw(void* ctx, const void* data) {
        if (!data) return;
        auto* self = static_cast<AttitudeEstimatorModule*>(ctx);
        self->handleSample(*static_cast<const ImuData*>(data));
    }

    void handleSample(const ImuData& imu) {
        float dt = 0.0f;  // First sample: the filter snaps to the accelerometer
        bool publishResult = true;

        if (_haveLastSample) {
            // Unsigned subtraction: correct across the 32-bit microsecond wrap (~71 min)
            const uint32_t dtUs = imu.timestampUs - _lastSampleUs;
            if (dtUs == 0) return;  // Same sample delivered twice: nothing new

            if (dtUs > MAX_GAP_US) {
                // Data was missing (or the timestamp went backwards, which wraps to a
                // huge dt). Integrating across the gap would be wrong: re-initialize.
                // SAFETY: the reset sample is not published (see FlightConfig.h).
                _gapResets++;
                LOG_W("ATTITUDE", "IMU gap of %lu us, estimator re-initialized (resets: %lu)",
                      static_cast<unsigned long>(dtUs), static_cast<unsigned long>(_gapResets));
                _filter.reset();
                publishResult = false;
            } else {
                dt = static_cast<float>(dtUs) * 1e-6f;
            }
        }
        _lastSampleUs = imu.timestampUs;
        _haveLastSample = true;

        _filter.update(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, dt);
        if (!publishResult) return;

        AttitudeData attitude{_filter.getRoll(), _filter.getPitch(), _filter.getYaw()};
        publish(EventType::ATTITUDE_UPDATED, &attitude);
    }

public:
    template <typename... Args>
    explicit AttitudeEstimatorModule(Args&&... args)
        : Module("ATTITUDE_ESTIMATOR"), _filter(std::forward<Args>(args)...) {}

    void setup() override {
        reset();
        subscribe(EventType::IMU_RAW, onImuRaw);
    }

    // loop() not needed — fully event-driven, uses default empty base implementation

    float getRoll() const { return _filter.getRoll(); }
    float getPitch() const { return _filter.getPitch(); }
    float getYaw() const { return _filter.getYaw(); }
    uint32_t gapResets() const { return _gapResets; }
    void reset() {
        _filter.reset();
        _haveLastSample = false;
        _lastSampleUs = 0;
    }
};
