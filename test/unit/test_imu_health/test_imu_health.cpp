#include <unity.h>
#include <cstdint>
#include "modules/sensors/bmi_imu/ImuHealthMonitor.h"

using Change = ImuHealthMonitor::Change;

// Realistic sample generator: gravity on Z plus a little noise that changes every call
struct SampleGen {
    uint32_t n = 0;
    float acc[3];
    float gyro[3];

    void next() {
        n++;
        const float noise = static_cast<float>(n % 7) * 0.001f;
        acc[0] = 0.01f + noise;
        acc[1] = -0.02f - noise;
        acc[2] = 1.0f + noise;
        gyro[0] = 0.1f + noise;
        gyro[1] = -0.2f;
        gyro[2] = 0.05f + noise;
    }
};

static const float ZERO[3] = {0.0f, 0.0f, 0.0f};

// Feed `count` good samples, return the last non-NONE change
static Change feedGood(ImuHealthMonitor& m, SampleGen& g, uint32_t count) {
    Change last = Change::NONE;
    for (uint32_t i = 0; i < count; i++) {
        g.next();
        Change c = m.onSample(g.acc, g.gyro);
        if (c != Change::NONE) last = c;
    }
    return last;
}

static Change feedZero(ImuHealthMonitor& m, uint32_t count) {
    Change last = Change::NONE;
    for (uint32_t i = 0; i < count; i++) {
        Change c = m.onSample(ZERO, ZERO);
        if (c != Change::NONE) last = c;
    }
    return last;
}

void setUp() {}
void tearDown() {}

// --- Normal operation ---

void test_good_data_is_healthy() {
    ImuHealthMonitor m;
    SampleGen g;
    TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(feedGood(m, g, 1000)));
    TEST_ASSERT_FALSE(m.isFaulty());
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::NONE), static_cast<int>(m.reason()));
}

void test_first_zero_sample_after_init_tolerated() {
    ImuHealthMonitor m;
    SampleGen g;
    feedZero(m, 1);                         // First read after init returns zeros
    feedGood(m, g, 100);
    TEST_ASSERT_FALSE(m.isFaulty());
}

void test_duplicate_samples_tolerated() {
    ImuHealthMonitor m;
    SampleGen g;
    for (int i = 0; i < 200; i++) {
        g.next();
        m.onSample(g.acc, g.gyro);
        m.onSample(g.acc, g.gyro);          // Same sample read twice (clock drift)
    }
    TEST_ASSERT_FALSE(m.isFaulty());
}

void test_isolated_read_errors_tolerated() {
    ImuHealthMonitor m;
    SampleGen g;
    for (int i = 0; i < 100; i++) {
        for (uint16_t e = 0; e < IMU_HEALTH_READ_ERROR_SAMPLES - 1; e++) m.onReadError();
        feedGood(m, g, 1);                  // A good read resets the error run
    }
    TEST_ASSERT_FALSE(m.isFaulty());
}

// --- Fault detection ---

void test_zero_data_raises_fault() {
    ImuHealthMonitor m;
    SampleGen g;
    feedGood(m, g, 10);
    TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(feedZero(m, IMU_HEALTH_ZERO_SAMPLES - 1)));
    TEST_ASSERT_FALSE(m.isFaulty());
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_RAISED), static_cast<int>(feedZero(m, 1)));
    TEST_ASSERT_TRUE(m.isFaulty());
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::ZERO_DATA), static_cast<int>(m.reason()));
}

void test_frozen_data_raises_fault() {
    ImuHealthMonitor m;
    SampleGen g;
    feedGood(m, g, 10);
    g.next();
    for (uint16_t i = 0; i < IMU_HEALTH_FROZEN_SAMPLES - 1; i++) {
        TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(m.onSample(g.acc, g.gyro)));
    }
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_RAISED), static_cast<int>(m.onSample(g.acc, g.gyro)));
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::FROZEN_DATA), static_cast<int>(m.reason()));
}

void test_read_errors_raise_fault() {
    ImuHealthMonitor m;
    for (uint16_t i = 0; i < IMU_HEALTH_READ_ERROR_SAMPLES - 1; i++) {
        TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(m.onReadError()));
    }
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_RAISED), static_cast<int>(m.onReadError()));
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::READ_ERRORS), static_cast<int>(m.reason()));
}

void test_init_failure_and_reset_raise_immediately() {
    ImuHealthMonitor a;
    TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(a.onInitResult(true)));
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_RAISED), static_cast<int>(a.onInitResult(false)));
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::INIT_FAILED), static_cast<int>(a.reason()));

    ImuHealthMonitor b;
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_RAISED), static_cast<int>(b.onResetDetected()));
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::RESET_DETECTED), static_cast<int>(b.reason()));
}

void test_fault_keeps_original_reason() {
    ImuHealthMonitor m;
    m.onResetDetected();
    feedZero(m, IMU_HEALTH_ZERO_SAMPLES);   // Second fault condition while already faulty
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::RESET_DETECTED), static_cast<int>(m.reason()));
    TEST_ASSERT_EQUAL_UINT32(1, m.faultCount());
}

// --- Recovery ---

void test_fault_clears_after_recovery_samples() {
    ImuHealthMonitor m;
    SampleGen g;
    feedZero(m, IMU_HEALTH_ZERO_SAMPLES);
    TEST_ASSERT_EQUAL(static_cast<int>(Change::NONE), static_cast<int>(feedGood(m, g, IMU_HEALTH_RECOVERY_SAMPLES - 1)));
    TEST_ASSERT_TRUE(m.isFaulty());
    TEST_ASSERT_EQUAL(static_cast<int>(Change::FAULT_CLEARED), static_cast<int>(feedGood(m, g, 1)));
    TEST_ASSERT_FALSE(m.isFaulty());
    TEST_ASSERT_EQUAL(static_cast<int>(SensorFaultReason::NONE), static_cast<int>(m.reason()));
}

void test_new_fault_during_recovery_restarts_count() {
    ImuHealthMonitor m;
    SampleGen g;
    feedZero(m, IMU_HEALTH_ZERO_SAMPLES);
    feedGood(m, g, IMU_HEALTH_RECOVERY_SAMPLES - 10);
    feedZero(m, IMU_HEALTH_ZERO_SAMPLES);   // Flaky contact: resets again
    feedGood(m, g, 20);
    TEST_ASSERT_TRUE(m.isFaulty());          // Would have cleared without the restart
    feedGood(m, g, IMU_HEALTH_RECOVERY_SAMPLES);
    TEST_ASSERT_FALSE(m.isFaulty());
}

void test_fault_count_increments_per_fault() {
    ImuHealthMonitor m;
    SampleGen g;
    for (int i = 0; i < 3; i++) {
        feedZero(m, IMU_HEALTH_ZERO_SAMPLES);
        feedGood(m, g, IMU_HEALTH_RECOVERY_SAMPLES);
    }
    TEST_ASSERT_EQUAL_UINT32(3, m.faultCount());
    TEST_ASSERT_FALSE(m.isFaulty());
}

void test_recovery_attempts_are_rate_limited() {
    ImuHealthMonitor m;
    TEST_ASSERT_FALSE(m.shouldAttemptRecovery(0));     // Healthy: nothing to recover
    m.onInitResult(false);
    TEST_ASSERT_TRUE(m.shouldAttemptRecovery(1000));   // First attempt allowed immediately
    m.onRecoveryAttempt(1000);
    TEST_ASSERT_FALSE(m.shouldAttemptRecovery(1000 + IMU_RECOVERY_RETRY_MS - 1));
    TEST_ASSERT_TRUE(m.shouldAttemptRecovery(1000 + IMU_RECOVERY_RETRY_MS));
}

void test_no_recovery_while_good_data_arrives() {
    ImuHealthMonitor m;
    SampleGen g;
    m.onResetDetected();
    m.onRecoveryAttempt(0);
    feedGood(m, g, 50);                                  // Re-init worked, data flowing
    TEST_ASSERT_TRUE(m.isFaulty());                      // Not cleared yet...
    TEST_ASSERT_FALSE(m.shouldAttemptRecovery(5000));    // ...but must not be re-initialized again
}

void test_recovery_retry_survives_clock_wraparound() {
    ImuHealthMonitor m;
    m.onInitResult(false);
    m.onRecoveryAttempt(UINT32_MAX - 100);
    TEST_ASSERT_FALSE(m.shouldAttemptRecovery(UINT32_MAX - 50));
    TEST_ASSERT_TRUE(m.shouldAttemptRecovery(IMU_RECOVERY_RETRY_MS));  // Wrapped past the retry time
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_good_data_is_healthy);
    RUN_TEST(test_first_zero_sample_after_init_tolerated);
    RUN_TEST(test_duplicate_samples_tolerated);
    RUN_TEST(test_isolated_read_errors_tolerated);

    RUN_TEST(test_zero_data_raises_fault);
    RUN_TEST(test_frozen_data_raises_fault);
    RUN_TEST(test_read_errors_raise_fault);
    RUN_TEST(test_init_failure_and_reset_raise_immediately);
    RUN_TEST(test_fault_keeps_original_reason);

    RUN_TEST(test_fault_clears_after_recovery_samples);
    RUN_TEST(test_new_fault_during_recovery_restarts_count);
    RUN_TEST(test_fault_count_increments_per_fault);
    RUN_TEST(test_recovery_attempts_are_rate_limited);
    RUN_TEST(test_no_recovery_while_good_data_arrives);
    RUN_TEST(test_recovery_retry_survives_clock_wraparound);

    return UNITY_END();
}
