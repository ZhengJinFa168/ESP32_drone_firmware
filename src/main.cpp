#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "core/App.h"
#include "core/Log.h"
#include "config/FlightConfig.h"
#include "modules/actuators/blink/BlinkModule.h"
#include "modules/logic/heartbeat/HeartbeatModule.h"
#include "modules/logic/attitude_estimator/AttitudeEstimatorModule.h"
#include "modules/sensors/bmi_imu/BMI270/Bmi270Module.h"
#include "modules/communication/rc_input/RcInputModule.h"
#include "modules/logic/arming/ArmingModule.h"

// Define the drone's specific attitude module using the configured filter
using DroneAttitudeEstimator = AttitudeEstimatorModule<ActiveAttitudeFilter>;

// ============================================
// MODULES (static: no heap, see architecture.md)
// ============================================
// Constructors run before setup(); they must not touch hardware or publish events.
static HeartbeatModule heartbeat(500);
static BlinkModule blink(LED_BUILTIN);
static Bmi270Module imu;
static DroneAttitudeEstimator attitudeEstimator(0.98f);
static RcInputModule rcInput;
static ArmingModule arming;

static Module* const MODULES[] = {
    &heartbeat, &blink, &imu, &attitudeEstimator, &rcInput, &arming,
};

// ============================================
// APPLICATION
// ============================================
static App drone;

// SAFETY: a missing module (e.g. arming) means the drone runs without its checks.
// Halt loudly instead of flying with a partial system; motors are never started.
[[noreturn]] static void haltWithError(const char* what) {
    for (;;) {
        LOG_E("MAIN", "HALTED: %s", what);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    logInit();
    for (Module* module : MODULES) {
        if (!drone.registerModule(module)) haltWithError("module registration failed (see above)");
    }
    drone.setup();
    LOG_I("MAIN", "System running");
}

void loop() {
    drone.loop();
    // Yield one tick (1 ms) so lower-priority tasks (idle, watchdog) can run
    vTaskDelay(1);
}
