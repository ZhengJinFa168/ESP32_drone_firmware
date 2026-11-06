#include <Arduino.h>
#include <unity.h>
#include "core/App.h"
#include "core/eventService/EventService.h"
#include "modules/sensors/bmi_imu/BMI270/Bmi270Module.h"

static bool eventReceived = false;
static ImuData lastData;

void onImuData(void* ctx, const void* data) {
    if (!data) return;
    eventReceived = true;
    lastData = *static_cast<const ImuData*>(data);
}

void setUp(void) {
    // set stuff up here
    EventService::instance().reset();
    eventReceived = false;
}

void tearDown(void) {
    // clean stuff up here
}

// Runs the app until the sensor is initialized, at most 1 s. Returns isInitialized().
static bool runUntilInitialized(App& app, Bmi270Module& imu) {
    unsigned long start = millis();
    while (millis() - start < 1000 && !imu.isInitialized()) {
        app.loop();
        delay(1);
    }
    return imu.isInitialized();
}

void test_bmi270_initialization_and_publish() {
    App testApp;
    Bmi270Module imu;
    
    testApp.registerModule(&imu);
    testApp.setup();

    // Subscribe to IMU_RAW events
    EventService::instance().subscribe(EventType::IMU_RAW, onImuData, nullptr);

    // setup() only starts a power cycle (sensor powered from PIN_IMU_POWER); the
    // init runs in loop() about IMU_POWER_OFF_MS + IMU_POWER_ON_SETTLE_MS later.
    TEST_ASSERT_FALSE_MESSAGE(imu.isInitialized(), "Sensor must not be initialized before the power cycle");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, imu.powerCycles(), "setup() must start exactly one power cycle");

    // This is a hardware test: the sensor must be present. A failed init is a
    // real failure, not a skip (a skip previously hid an uninitialized-driver bug).
    TEST_ASSERT_TRUE_MESSAGE(runUntilInitialized(testApp, imu), "BMI270 not initialized 1 s after boot (power, wiring or stuck)");
    TEST_ASSERT_TRUE_MESSAGE(imu.isHealthy(), "BMI270 unhealthy after init (sensor fault)");

    // The board must lie still: gyro calibration (~1 s) runs before any IMU_RAW is
    // published. Allow 3 s for calibration (a restart or two) plus some samples.
    unsigned long start = millis();
    while (millis() - start < 3000 && !(imu.isGyroCalibrated() && eventReceived)) {
        testApp.loop();
        delay(1);
    }

    TEST_ASSERT_TRUE_MESSAGE(imu.isGyroCalibrated(), "Gyro calibration did not finish in 3 s (board moving?)");
    TEST_ASSERT_TRUE_MESSAGE(eventReceived, "Expected to receive IMU_RAW event, but did not.");

    // Print the data to see what it looks like
    Serial.printf("Acc: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f\n",
        lastData.ax, lastData.ay, lastData.az,
        lastData.gx, lastData.gy, lastData.gz);
}

// Every boot power-cycles the sensor: repeated boots (like repeated uploads/resets,
// which used to leave the sensor stuck with CHIP_ID 0x20) must always initialize.
void test_repeated_boot_power_cycles() {
    for (int i = 0; i < 5; i++) {
        EventService::instance().reset();
        App app;
        Bmi270Module imu;
        app.registerModule(&imu);
        app.setup();
        TEST_ASSERT_TRUE_MESSAGE(runUntilInitialized(app, imu), "BMI270 did not initialize after a power cycle");
        TEST_ASSERT_TRUE(imu.isHealthy());
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial to stabilize

    UNITY_BEGIN();

    RUN_TEST(test_bmi270_initialization_and_publish);
    RUN_TEST(test_repeated_boot_power_cycles);

    UNITY_END();
}

void loop() {
    // Empty – tests run once
}

