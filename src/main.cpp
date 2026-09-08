#include "core/App.h"
#include "config/FlightConfig.h"
#include "modules/actuators/blink/BlinkModule.h"
#include "modules/logic/heartbeat/HeartbeatModule.h"
#include "modules/logic/attitude_estimator/AttitudeEstimatorModule.h"

// Define the drone's specific attitude module using the configured filter
using DroneAttitudeEstimator = AttitudeEstimatorModule<ActiveAttitudeFilter>;

// ============================================
// APPLICATION
// ============================================
App drone;

void setup() {
    drone.registerModule(new HeartbeatModule(500));
    drone.registerModule(new BlinkModule(LED_BUILTIN));
    drone.registerModule(new DroneAttitudeEstimator(0.98f));
    drone.setup();
    Serial.println("System Running!");
}

void loop() {
    drone.loop();
}