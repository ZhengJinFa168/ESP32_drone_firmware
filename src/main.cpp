#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "core/App.h"
#include "core/Log.h"
#include "config/FlightConfig.h"
#include "platform/esp32/Esp32Task.h"
#include "platform/esp32/Esp32TelemetryOutput.h"
#include "modules/actuators/blink/BlinkModule.h"
#include "modules/logic/heartbeat/HeartbeatModule.h"
#include "modules/logic/attitude_estimator/AttitudeEstimatorModule.h"
#include "modules/sensors/bmi_imu/BMI270/Bmi270Module.h"
#include "modules/communication/rc_input/RcInputModule.h"
#include "modules/logic/arming/ArmingModule.h"
#include "modules/logic/telemetry/TelemetryModule.h"

// Define the drone's specific attitude module using the configured filter
using DroneAttitudeEstimator = AttitudeEstimatorModule<ActiveAttitudeFilter>;

// ============================================
// MODULES (static: no heap, see architecture.md)
// ============================================
// Constructors run before setup(); they must not touch hardware or publish events.
static HeartbeatModule heartbeat(500);
static BlinkModule blink(LED_BUILTIN);
static Bmi270Module imu;
static DroneAttitudeEstimator attitudeEstimator(ATTITUDE_FILTER_TAU_S);
static RcInputModule rcInput;
static ArmingModule arming;
static TelemetryModule<Esp32TelemetryOutput> telemetry;

static Module* const MODULES[] = {
    &heartbeat, &blink, &imu, &attitudeEstimator, &rcInput, &arming, &telemetry,
};

// ============================================
// APPLICATION
// ============================================
static App drone(FLIGHT_LOOP_PERIOD_US);

// SAFETY: a missing module (e.g. arming) means the drone runs without its checks.
// Halt loudly instead of flying with a partial system; motors are never started.
[[noreturn]] static void haltWithError(const char* what) {
    for (;;) {
        LOG_E("MAIN", "HALTED: %s", what);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================
// FLIGHT TASK (core 1)
// ============================================
// SAFETY / threading: the event bus is synchronous and single-threaded, so the whole
// App runs in this one task (see architecture.md). It is woken by the BMI270
// data-ready interrupt (400 Hz): one pass per IMU sample, with the lowest latency
// from sample to motor output. Arduino's loopTask is not used (priority 1, would be
// preempted by anything else on core 1).
// Without an interrupt it still runs every FLIGHT_TASK_WAIT_MS, so arming, RC loss and
// the IMU's NO_DATA_READY check keep working while the sensor is off or missing.
static Esp32PinnedTask<FLIGHT_TASK_STACK_BYTES> flightTask;

[[noreturn]] static void flightLoop(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FLIGHT_TASK_WAIT_MS));
        drone.loop();
    }
}

void setup() {
    logInit();
    // From here on logging never blocks: lines go to the core-0 serial writer task.
    // Before any module logs, so the boot burst is queued too. A failure is logged and
    // logging stays direct (blocking), which is still safe on the ground.
    logStartWriter();
    for (Module* module : MODULES) {
        if (!drone.registerModule(module)) haltWithError("module registration failed (see above)");
    }
    // Runs here (loopTask, core 1): the IMU's interrupt is allocated on core 1 too
    drone.setup();

    // The flight task has a higher priority than this one: it starts running at once
    if (!flightTask.start("flight", flightLoop, nullptr, FLIGHT_TASK_PRIORITY, FLIGHT_TASK_CORE)) {
        haltWithError("flight task could not be created");
    }
    imu.setDataReadyTask(flightTask.handle());
    LOG_I("MAIN", "System running: flight task on core %d, priority %lu",
          FLIGHT_TASK_CORE, static_cast<unsigned long>(FLIGHT_TASK_PRIORITY));
}

void loop() {
    // Everything runs in the flight task; loopTask is not needed
    vTaskDelete(nullptr);
}
