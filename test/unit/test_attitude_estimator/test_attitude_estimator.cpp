#include <unity.h>
#include <cmath>
#include "modules/logic/attitude_estimator/ComplementaryFilter.h"
#include "modules/logic/attitude_estimator/AttitudeEstimatorModule.h"

void setUp() {
    EventService::instance().reset();
}

void tearDown() {
    EventService::instance().reset();
}

void test_filter_level_steady_state() {
    ComplementaryFilter filter;
    // Flat on table: gravity points downward through Z axis (1g)
    filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getPitch());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getYaw());
}

void test_filter_roll_step_convergence() {
    ComplementaryFilter filter;
    // 30 degree roll tilt: ay = sin(30 deg) = 0.5g, az = cos(30 deg) = 0.866025g
    const float angleRad = 30.0f * (3.14159265f / 180.0f);
    const float ay = std::sin(angleRad);
    const float az = std::cos(angleRad);

    filter.update(0.0f, ay, az, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, filter.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, filter.getPitch());
}

void test_filter_pitch_step_convergence() {
    ComplementaryFilter filter;
    // -25 degree pitch tilt: ax = -sin(-25 deg), az = cos(-25 deg)
    const float angleRad = -25.0f * (3.14159265f / 180.0f);
    const float ax = -std::sin(angleRad);
    const float az = std::cos(angleRad);

    filter.update(ax, 0.0f, az, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, -25.0f, filter.getPitch());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, filter.getRoll());
}

void test_filter_yaw_integration() {
    ComplementaryFilter filter;
    filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.002f);

    // Rotate at 60 deg/s around Z axis for 1.0s (100 steps of 0.01s)
    const float dt = 0.01f;
    const float yawRate = 60.0f;
    for (int i = 0; i < 100; i++) {
        filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, yawRate, dt);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, filter.getYaw());
}

void test_filter_gyro_fast_response_roll() {
    ComplementaryFilter filter;
    // Start aligned level
    filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.002f);

    // Rapid roll spin at 100 deg/s for 100 ms (50 cycles of 0.002s) -> ~10 deg change
    for (int i = 0; i < 50; i++) {
        filter.update(0.0f, 0.0f, 1.0f, 100.0f, 0.0f, 0.0f, 0.002f);
    }

    // Filter should quickly follow the gyro rate (exponential response towards ~6.2 deg)
    TEST_ASSERT_GREATER_THAN(5.0f, filter.getRoll());
}

void test_module_event_integration() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.98f);
    estimator.setup();

    struct Ctx { AttitudeData data{}; bool received = false; };
    Ctx ctx;

    EventService::instance().subscribe(EventType::ATTITUDE_UPDATED, [](void* c, const void* data) {
        if (data) {
            auto* ctx = static_cast<Ctx*>(c);
            ctx->data = *static_cast<const AttitudeData*>(data);
            ctx->received = true;
        }
    }, &ctx);

    // 45 degree roll: ay = sin(45 deg) = 0.7071g, az = cos(45 deg) = 0.7071g
    ImuData rawImu{0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f, 0.0f};
    EventService::instance().publish(EventType::IMU_RAW, &rawImu);

    TEST_ASSERT_TRUE(ctx.received);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, ctx.data.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ctx.data.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, estimator.getRoll());
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_filter_level_steady_state);
    RUN_TEST(test_filter_roll_step_convergence);
    RUN_TEST(test_filter_pitch_step_convergence);
    RUN_TEST(test_filter_yaw_integration);
    RUN_TEST(test_filter_gyro_fast_response_roll);
    RUN_TEST(test_module_event_integration);

    return UNITY_END();
}
