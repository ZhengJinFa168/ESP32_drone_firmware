#pragma once
#include <Arduino.h>
#include "core/Module.h"

class BlinkModule : public Module {
private:
    int pin;
    int intervalMs;
    unsigned long lastToggle;

public:
    // interval == 0: event-driven via EventType::HEARTBEAT
    // interval > 0: autonomous periodic blinking in loop()
    explicit BlinkModule(int p, int interval = 0)
        : Module("BLINK"), pin(p), intervalMs(interval), lastToggle(0) {}

    void setup() override {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);

        if (intervalMs == 0) {
            subscribe(EventType::HEARTBEAT, [this](const void*) {
                digitalWrite(pin, !digitalRead(pin));
            });
        }
    }

    void loop() override {
        if (intervalMs > 0) {
            unsigned long now = millis();
            if (now - lastToggle >= static_cast<unsigned long>(intervalMs)) {
                digitalWrite(pin, !digitalRead(pin));
                lastToggle = now;
            }
        }
    }

    bool isPinHigh() const {
        return digitalRead(pin) == HIGH;
    }
};

