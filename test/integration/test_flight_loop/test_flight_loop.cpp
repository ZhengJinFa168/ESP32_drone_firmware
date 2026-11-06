#include <Arduino.h>
#include <unity.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "core/App.h"
#include "core/eventService/EventService.h"
#include "config/FlightConfig.h"
#include "platform/esp32/Esp32Task.h"
#include "modules/sensors/bmi_imu/BMI270/Bmi270Module.h"

// Stage 2 timing check: the App runs in the flight task (core 1, priority
// FLIGHT_TASK_PRIORITY), woken by the BMI270 data-ready interrupt, as in main.cpp.
// Only the IMU is registered here; the full firmware (WiFi, phone, telemetry) is
// checked on the bench with the loop telemetry (see FlightConfig.h for the criteria).

static App app(FLIGHT_LOOP_PERIOD_US);
static Bmi270Module imu;
static Esp32PinnedTask<FLIGHT_TASK_STACK_BYTES> flightTask;

// Written by the flight task (event callback), read by the test task
static portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
static LoopStatsData lastStats{};
static uint32_t statsWindows = 0;

static void onLoopStats(void*, const void* data) {
    portENTER_CRITICAL(&statsMux);
    lastStats = *static_cast<const LoopStatsData*>(data);
    statsWindows++;
    portEXIT_CRITICAL(&statsMux);
}

static uint32_t windows(LoopStatsData* out = nullptr) {
    portENTER_CRITICAL(&statsMux);
    const uint32_t n = statsWindows;
    if (out) *out = lastStats;
    portEXIT_CRITICAL(&statsMux);
    return n;
}

[[noreturn]] static void flightLoop(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FLIGHT_TASK_WAIT_MS));
        app.loop();
    }
}

void setUp() {}
void tearDown() {}

void test_flight_loop_runs_at_imu_rate() {
    // The first windows contain the init (~300 ms blocking) and the gyro calibration.
    // Wait until calibrated, skip one more window, then check 3 full windows.
    unsigned long start = millis();
    while (millis() - start < 5000 && !imu.isGyroCalibrated()) vTaskDelay(pdMS_TO_TICKS(10));
    TEST_ASSERT_TRUE_MESSAGE(imu.isGyroCalibrated(), "Gyro not calibrated in 5 s (board moving? INT?)");

    const uint32_t first = windows() + 1;
    for (uint32_t w = first; w < first + 3; w++) {
        start = millis();
        while (windows() <= w && millis() - start < 2000) vTaskDelay(pdMS_TO_TICKS(10));
        LoopStatsData s;
        TEST_ASSERT_TRUE_MESSAGE(windows(&s) > w, "No LOOP_STATS for 2 s (flight task stopped?)");

        Serial.printf("window: loopHz %lu, run max %lu us, gap max %lu us, overruns %lu\n",
                      static_cast<unsigned long>(s.rateHz), static_cast<unsigned long>(s.maxRunUs),
                      static_cast<unsigned long>(s.maxGapUs), static_cast<unsigned long>(s.overruns));
        TEST_ASSERT_UINT32_WITHIN_MESSAGE(2, 400, s.rateHz, "Loop is not locked to the 400 Hz INT");
        TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(3000, s.maxGapUs, "Gap between passes > 3000 us");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, s.overruns, "A pass took longer than one sample period");
    }
    TEST_ASSERT_TRUE_MESSAGE(imu.isHealthy(), "IMU fault during the run (NO_DATA_READY?)");
    Serial.printf("flight task stack: %lu bytes never used\n",
                  static_cast<unsigned long>(flightTask.stackFreeMinBytes()));
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(1024, flightTask.stackFreeMinBytes(), "Flight task stack margin < 1 KB");
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial to stabilize

    EventService::instance().subscribe(EventType::LOOP_STATS, onLoopStats, nullptr);
    app.registerModule(&imu);
    app.setup();  // On core 1 (loopTask): the IMU interrupt is allocated there
    TEST_ASSERT_TRUE(flightTask.start("flight", flightLoop, nullptr, FLIGHT_TASK_PRIORITY, FLIGHT_TASK_CORE));
    imu.setDataReadyTask(flightTask.handle());

    UNITY_BEGIN();
    RUN_TEST(test_flight_loop_runs_at_imu_rate);
    vTaskSuspend(flightTask.handle());
    UNITY_END();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
