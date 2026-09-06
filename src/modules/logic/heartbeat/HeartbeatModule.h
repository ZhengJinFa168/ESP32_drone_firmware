#pragma once
#include <Arduino.h>
#include "core/Module.h"

class HeartbeatModule : public Module {
private:
    unsigned long lastBeat;
    int intervalMs;

public:
    explicit HeartbeatModule(int interval = 500)
        : Module("HEARTBEAT"), lastBeat(0), intervalMs(interval) {}

    void setup() override {
        lastBeat = millis();
    }

    void loop() override {
        unsigned long now = millis();
        if (now - lastBeat >= static_cast<unsigned long>(intervalMs)) {
            lastBeat = now;
            publish(EventType::HEARTBEAT);
        }
    }
};

