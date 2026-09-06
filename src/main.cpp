#include "core/App.h"
#include "modules/actuators/blink/BlinkModule.h"
#include "modules/logic/heartbeat/HeartbeatModule.h"

// ============================================
// APPLICATION
// ============================================
App drone;

void setup() {
    drone.registerModule(new HeartbeatModule(500));
    drone.registerModule(new BlinkModule(LED_BUILTIN));
    drone.setup();
    Serial.println("System Running!");
}

void loop() {
    drone.loop();
}