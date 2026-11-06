#pragma once
#include <cstdint>
#include "core/Module.h"
#include "core/Clock.h"

class HeartbeatModule : public Module {
private:
    uint32_t lastBeat;
    uint32_t intervalMs;

public:
    explicit HeartbeatModule(uint32_t interval = 500)
        : Module("HEARTBEAT"), lastBeat(0), intervalMs(interval) {}

    void setup() override {
        lastBeat = clockMs();
    }

    void loop() override {
        const uint32_t now = clockMs();
        if (now - lastBeat >= intervalMs) {
            lastBeat = now;
            publish(EventType::HEARTBEAT);
        }
    }
};
