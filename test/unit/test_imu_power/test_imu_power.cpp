#include <unity.h>
#include <cstdint>
#include "modules/sensors/bmi_imu/ImuPowerSequencer.h"

using Action = ImuPowerSequencer::Action;

void setUp() {}
void tearDown() {}

static void assertAction(Action expected, Action actual) {
    TEST_ASSERT_EQUAL(static_cast<int>(expected), static_cast<int>(actual));
}

void test_idle_does_nothing() {
    ImuPowerSequencer seq;
    TEST_ASSERT_FALSE(seq.busy());
    assertAction(Action::NONE, seq.update(0));
    assertAction(Action::NONE, seq.update(100000));
    TEST_ASSERT_EQUAL_UINT32(0, seq.cycles());
}

void test_full_cycle_order_and_timing() {
    ImuPowerSequencer seq;
    const uint32_t t0 = 1000;
    assertAction(Action::CUT_POWER, seq.start(t0));
    TEST_ASSERT_TRUE(seq.busy());

    // Power stays off for exactly IMU_POWER_OFF_MS
    assertAction(Action::NONE, seq.update(t0));
    assertAction(Action::NONE, seq.update(t0 + IMU_POWER_OFF_MS - 1));
    assertAction(Action::RESTORE_POWER, seq.update(t0 + IMU_POWER_OFF_MS));
    TEST_ASSERT_TRUE(seq.busy());

    // Then IMU_POWER_ON_SETTLE_MS before the sensor may be touched
    const uint32_t tOn = t0 + IMU_POWER_OFF_MS;
    assertAction(Action::NONE, seq.update(tOn + IMU_POWER_ON_SETTLE_MS - 1));
    assertAction(Action::INIT_SENSOR, seq.update(tOn + IMU_POWER_ON_SETTLE_MS));
    TEST_ASSERT_FALSE(seq.busy());
    TEST_ASSERT_EQUAL_UINT32(1, seq.cycles());

    // Each action is returned once
    assertAction(Action::NONE, seq.update(tOn + 10000));
}

void test_settle_counts_from_actual_power_on() {
    // A late update (loop stalled) must not shorten the settle time
    ImuPowerSequencer seq;
    seq.start(0);
    const uint32_t lateOn = IMU_POWER_OFF_MS + 500;
    assertAction(Action::RESTORE_POWER, seq.update(lateOn));
    assertAction(Action::NONE, seq.update(lateOn + IMU_POWER_ON_SETTLE_MS - 1));
    assertAction(Action::INIT_SENSOR, seq.update(lateOn + IMU_POWER_ON_SETTLE_MS));
}

void test_start_while_busy_is_ignored() {
    // A second request must not restart (and shorten) the current off time
    ImuPowerSequencer seq;
    assertAction(Action::CUT_POWER, seq.start(0));
    assertAction(Action::NONE, seq.start(IMU_POWER_OFF_MS - 1));
    assertAction(Action::RESTORE_POWER, seq.update(IMU_POWER_OFF_MS));
    assertAction(Action::NONE, seq.start(IMU_POWER_OFF_MS + 1));   // Also while settling
    TEST_ASSERT_EQUAL_UINT32(1, seq.cycles());
}

void test_millis_wraparound() {
    ImuPowerSequencer seq;
    const uint32_t t0 = 0xFFFFFFFFu - 10;  // Off period crosses the wrap
    seq.start(t0);
    assertAction(Action::NONE, seq.update(t0 + 5));
    assertAction(Action::RESTORE_POWER, seq.update(t0 + IMU_POWER_OFF_MS));
    assertAction(Action::INIT_SENSOR, seq.update(t0 + IMU_POWER_OFF_MS + IMU_POWER_ON_SETTLE_MS));
}

void test_can_cycle_again_after_completion() {
    ImuPowerSequencer seq;
    seq.start(0);
    seq.update(IMU_POWER_OFF_MS);
    seq.update(IMU_POWER_OFF_MS + IMU_POWER_ON_SETTLE_MS);
    assertAction(Action::CUT_POWER, seq.start(5000));
    TEST_ASSERT_TRUE(seq.busy());
    TEST_ASSERT_EQUAL_UINT32(2, seq.cycles());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_idle_does_nothing);
    RUN_TEST(test_full_cycle_order_and_timing);
    RUN_TEST(test_settle_counts_from_actual_power_on);
    RUN_TEST(test_start_while_busy_is_ignored);
    RUN_TEST(test_millis_wraparound);
    RUN_TEST(test_can_cycle_again_after_completion);
    return UNITY_END();
}
