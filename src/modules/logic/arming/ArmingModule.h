#pragma once
#include <cstdint>
#include "core/Module.h"
#include "config/SensorConfig.h"
#include "ArmingLogic.h"

// Connects ArmingLogic to the event bus.
// Subscribes: RC_INPUT, ATTITUDE_UPDATED, SENSOR_FAULT
// Publishes:  ARMED, DISARMED (on transitions), ARMING_STATUS (on every status change)
// All decisions live in ArmingLogic; this wrapper only supplies the clock and events.
class ArmingModule : public Module {
private:
    ArmingLogic _logic;
    ArmingStatusData _lastStatus{};
    bool _statusPublished = false;

    static uint32_t nowMs() { return static_cast<uint32_t>(millis()); }

    static void onRcInput(void* ctx, const void* data) {
        if (!data) return;
        auto* self = static_cast<ArmingModule*>(ctx);
        self->_logic.onRcInput(*static_cast<const RcInputData*>(data), nowMs());
        self->step();
    }

    static void onAttitude(void* ctx, const void* data) {
        if (!data) return;
        auto* self = static_cast<ArmingModule*>(ctx);
        self->_logic.onAttitude(*static_cast<const AttitudeData*>(data), nowMs());
        self->step();
    }

    static void onSensorFault(void* ctx, const void* data) {
        if (!data) return;
        auto* self = static_cast<ArmingModule*>(ctx);
        self->_logic.onSensorFault(static_cast<const SensorFaultData*>(data)->fault);
        self->step();
    }

    void step() {
        const ArmingLogic::Transition transition = _logic.update(nowMs());
        const ArmingStatusData status = _logic.status();

        if (transition == ArmingLogic::Transition::ARMED) {
            Serial.println("ARMING: ARMED");
            publish(EventType::ARMED, &status);
        } else if (transition == ArmingLogic::Transition::DISARMED) {
            Serial.print("ARMING: DISARMED, reason: ");
            Serial.println(disarmReasonName(status.lastDisarmReason));
            publish(EventType::DISARMED, &status);
        }

        if (!_statusPublished || statusChanged(status)) {
            _lastStatus = status;
            _statusPublished = true;
            if (!status.armed) logBlockers(status.blockers);
            publish(EventType::ARMING_STATUS, &status);
        }
    }

    bool statusChanged(const ArmingStatusData& s) const {
        return s.armed != _lastStatus.armed || s.blockers != _lastStatus.blockers ||
               s.lastDisarmReason != _lastStatus.lastDisarmReason || s.rcLinkOk != _lastStatus.rcLinkOk;
    }

    static void logBlockers(uint16_t blockers) {
        if (blockers == 0) {
            Serial.println("ARMING: ready to arm (turn the arm switch ON)");
            return;
        }
        Serial.print("ARMING: blocked by:");
        for (uint16_t bit = 1; bit != 0; bit <<= 1) {
            if (blockers & bit) {
                Serial.print(" ");
                Serial.print(armingBlockerName(bit));
            }
        }
        Serial.println("");
    }

public:
    explicit ArmingModule(bool alignmentVerified = IMU_ALIGNMENT_VERIFIED)
        : Module("ARMING"), _logic(alignmentVerified) {}

    void setup() override {
        subscribe(EventType::RC_INPUT, onRcInput);
        subscribe(EventType::ATTITUDE_UPDATED, onAttitude);
        subscribe(EventType::SENSOR_FAULT, onSensorFault);
        step();  // Publish the initial (disarmed) status
    }

    // Timeouts (RC loss, stale attitude, auto-disarm) must fire even when no events arrive
    void loop() override { step(); }

    bool isArmed() const { return _logic.isArmed(); }
    ArmingStatusData status() const { return _logic.status(); }
};
