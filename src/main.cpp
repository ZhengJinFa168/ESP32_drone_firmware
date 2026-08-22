#include "core/App.h"
#include "core/Module.h"

// ============================================
// BLINK MODULE – Temporary for Phase 1
// Future: Move to modules/actuators/blink/
// ============================================
class BlinkModule : public Module {
private:
    int pin;
    unsigned long lastToggle;
    int intervalMs;

public:
    BlinkModule(int p, int interval)
        : Module("BLINK"), pin(p), intervalMs(interval), lastToggle(0) {}

    void setup() override {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    void loop() override {
        unsigned long now = millis();
        if (now - lastToggle >= intervalMs) {
            digitalWrite(pin, !digitalRead(pin));
            lastToggle = now;
        }
    }
};

// ============================================
// APPLICATION
// ============================================
App drone;

void setup() {
    drone.registerModule(new BlinkModule(LED_BUILTIN, 500));
    drone.setup();
    Serial.println("System Running!");
}

void loop() {
    drone.loop();
}