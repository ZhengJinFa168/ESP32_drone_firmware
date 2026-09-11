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

void test_bmi270_initialization_and_publish() {
    App testApp;
    Bmi270Module imu;
    
    testApp.registerModule(&imu);
    testApp.setup();

    // Subscribe to IMU_RAW events
    EventService::instance().subscribe(EventType::IMU_RAW, onImuData, nullptr);

    // If the sensor is connected, the module should be enabled
    if (imu.isEnabled()) {
        Serial.println("BMI270 is connected. Testing data read...");
        
        // Run loop a few times to allow time for polling
        unsigned long start = millis();
        while (millis() - start < 100) { // Run for 100ms
            testApp.loop();
            delay(1);
        }

        // We should have received at least one event
        TEST_ASSERT_TRUE_MESSAGE(eventReceived, "Expected to receive IMU_RAW event, but did not.");
        
        // Print the data to see what it looks like
        Serial.printf("Acc: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f\n", 
            lastData.ax, lastData.ay, lastData.az,
            lastData.gx, lastData.gy, lastData.gz);

    } else {
        // If not connected, we should skip the test gracefully rather than failing
        Serial.println("BMI270 is NOT connected. Skipping data test.");
        TEST_IGNORE_MESSAGE("BMI270 not connected, skipped data read test.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial to stabilize

    UNITY_BEGIN();

    RUN_TEST(test_bmi270_initialization_and_publish);

    UNITY_END();
}

void loop() {
    // Empty – tests run once
}

