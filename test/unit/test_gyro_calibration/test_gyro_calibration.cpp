#include <unity.h>
#include <cmath>
#include <cstdint>
#include "modules/sensors/bmi_imu/GyroCalibrator.h"

using Result = GyroCalibrator::Result;

static const float LEVEL[3] = {0.0f, 0.0f, 1.0f};

// Still sensor: a fixed bias plus small noise that changes every sample (±0.1 deg/s)
struct StillGyro {
    float bias[3];
    uint32_t n = 0;
    float out[3];

    StillGyro(float bx, float by, float bz) : bias{bx, by, bz} {}

    const float* next() {
        n++;
        const float noise = (static_cast<float>(n % 5) - 2.0f) * 0.05f;
        for (int i = 0; i < 3; i++) out[i] = bias[i] + noise;
        return out;
    }
};

// Feed `count` still samples, return the last result
static Result feed(GyroCalibrator& cal, StillGyro& g, uint32_t count, const float acc[3] = LEVEL) {
    Result last = Result::COLLECTING;
    for (uint32_t i = 0; i < count; i++) last = cal.onSample(acc, g.next());
    return last;
}

void setUp() {}
void tearDown() {}

void test_still_sensor_calibrates_to_its_bias() {
    GyroCalibrator cal;
    StillGyro g(0.4f, -0.3f, 0.2f);
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COLLECTING), static_cast<int>(feed(cal, g, GYRO_CAL_SAMPLES - 1)));
    TEST_ASSERT_FALSE(cal.isCalibrated());
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COMPLETED), static_cast<int>(feed(cal, g, 1)));
    TEST_ASSERT_TRUE(cal.isCalibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, cal.bias(0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.3f, cal.bias(1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, cal.bias(2));
    TEST_ASSERT_EQUAL_UINT32(0, cal.restarts());
}

void test_apply_removes_bias() {
    GyroCalibrator cal;
    StillGyro g(1.0f, -2.0f, 0.5f);
    feed(cal, g, GYRO_CAL_SAMPLES);
    float gyro[3] = {1.0f, -2.0f, 0.5f};   // Bias only = sensor at rest
    cal.apply(gyro);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gyro[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gyro[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gyro[2]);
}

void test_tilted_but_still_is_accepted() {
    GyroCalibrator cal;
    StillGyro g(0.1f, 0.1f, 0.1f);
    const float tilted[3] = {0.5f, 0.0f, 0.866f};   // 30 deg pitch, magnitude 1 g
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COMPLETED), static_cast<int>(feed(cal, g, GYRO_CAL_SAMPLES, tilted)));
}

void test_motion_restarts_window() {
    GyroCalibrator cal;
    StillGyro g(0.2f, 0.2f, 0.2f);
    feed(cal, g, GYRO_CAL_SAMPLES / 2);
    const float bump[3] = {0.2f + GYRO_CAL_MAX_SPREAD_DPS + 1.0f, 0.2f, 0.2f};  // Someone touched it
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_MOTION), static_cast<int>(cal.onSample(LEVEL, bump)));
    TEST_ASSERT_EQUAL_UINT16(0, cal.samplesCollected());
    TEST_ASSERT_EQUAL_UINT32(1, cal.restarts());

    // A full window is needed again
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COLLECTING), static_cast<int>(feed(cal, g, GYRO_CAL_SAMPLES - 1)));
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COMPLETED), static_cast<int>(feed(cal, g, 1)));
}

void test_slow_drift_beyond_spread_restarts() {
    GyroCalibrator cal;
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    Result r = Result::COLLECTING;
    // Slow rotation: each step small, but the window spread grows past the limit
    for (uint32_t i = 0; i < GYRO_CAL_SAMPLES && r == Result::COLLECTING; i++) {
        gyro[2] = static_cast<float>(i) * 0.01f;
        r = cal.onSample(LEVEL, gyro);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_MOTION), static_cast<int>(r));
    TEST_ASSERT_FALSE(cal.isCalibrated());
}

void test_accel_not_one_g_restarts() {
    GyroCalibrator cal;
    StillGyro g(0.0f, 0.0f, 0.0f);
    const float shaken[3] = {0.0f, 0.0f, 1.3f};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_ACCEL), static_cast<int>(cal.onSample(shaken, g.next())));
    const float freeFall[3] = {0.0f, 0.0f, 0.1f};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_ACCEL), static_cast<int>(cal.onSample(freeFall, g.next())));
    TEST_ASSERT_EQUAL_UINT32(2, cal.restarts());
}

void test_implausible_bias_rejected() {
    GyroCalibrator cal;
    StillGyro g(GYRO_CAL_MAX_BIAS_DPS + 1.0f, 0.0f, 0.0f);   // Still, but offset too large
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_BIAS), static_cast<int>(feed(cal, g, GYRO_CAL_SAMPLES)));
    TEST_ASSERT_FALSE(cal.isCalibrated());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.bias(0));       // No bias adopted
}

void test_non_finite_samples_restart() {
    GyroCalibrator cal;
    const float nanGyro[3] = {NAN, 0.0f, 0.0f};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_MOTION), static_cast<int>(cal.onSample(LEVEL, nanGyro)));
    const float nanAcc[3] = {NAN, 0.0f, 1.0f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::RESTARTED_ACCEL), static_cast<int>(cal.onSample(nanAcc, zero)));
}

void test_external_restart_discards_window() {
    GyroCalibrator cal;
    StillGyro g(0.1f, 0.1f, 0.1f);
    feed(cal, g, GYRO_CAL_SAMPLES - 1);
    cal.restart();                                            // e.g. sensor fault
    TEST_ASSERT_EQUAL(static_cast<int>(Result::COLLECTING), static_cast<int>(feed(cal, g, 1)));
    TEST_ASSERT_EQUAL_UINT16(1, cal.samplesCollected());
}

void test_calibration_is_kept_once_done() {
    GyroCalibrator cal;
    StillGyro g(0.3f, 0.0f, 0.0f);
    feed(cal, g, GYRO_CAL_SAMPLES);
    cal.restart();                                            // Ignored once calibrated
    const float bump[3] = {50.0f, 0.0f, 0.0f};
    TEST_ASSERT_EQUAL(static_cast<int>(Result::IDLE), static_cast<int>(cal.onSample(LEVEL, bump)));
    TEST_ASSERT_TRUE(cal.isCalibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, cal.bias(0));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_still_sensor_calibrates_to_its_bias);
    RUN_TEST(test_apply_removes_bias);
    RUN_TEST(test_tilted_but_still_is_accepted);
    RUN_TEST(test_motion_restarts_window);
    RUN_TEST(test_slow_drift_beyond_spread_restarts);
    RUN_TEST(test_accel_not_one_g_restarts);
    RUN_TEST(test_implausible_bias_rejected);
    RUN_TEST(test_non_finite_samples_restart);
    RUN_TEST(test_external_restart_discards_window);
    RUN_TEST(test_calibration_is_kept_once_done);
    return UNITY_END();
}
