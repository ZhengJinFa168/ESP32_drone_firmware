#include <unity.h>
#include <cmath>
#include <cstdint>
#include "modules/logic/attitude_estimator/ComplementaryFilter.h"
#include "modules/logic/attitude_estimator/AttitudeEstimatorModule.h"

void setUp() {
    EventService::instance().reset();
    g_mockLog.reset();
}

void tearDown() {
    EventService::instance().reset();
}

void test_filter_level_steady_state() {
    ComplementaryFilter filter(0.5f);
    // Flat on table: gravity points downward through Z axis (1g)
    filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getPitch());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, filter.getYaw());
}

void test_filter_roll_step_convergence() {
    ComplementaryFilter filter(0.5f);
    // 30 degree roll tilt: ay = sin(30 deg) = 0.5g, az = cos(30 deg) = 0.866025g
    const float angleRad = 30.0f * (3.14159265f / 180.0f);
    const float ay = std::sin(angleRad);
    const float az = std::cos(angleRad);

    filter.update(0.0f, ay, az, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, filter.getRoll());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, filter.getPitch());
}

void test_filter_pitch_step_convergence() {
    ComplementaryFilter filter(0.5f);
    // -25 degree pitch tilt: ax = -sin(-25 deg), az = cos(-25 deg)
    const float angleRad = -25.0f * (3.14159265f / 180.0f);
    const float ax = -std::sin(angleRad);
    const float az = std::cos(angleRad);

    filter.update(ax, 0.0f, az, 0.0f, 0.0f, 0.0f, 0.002f);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, -25.0f, filter.getPitch());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, filter.getRoll());
}

void test_filter_yaw_integration() {
    ComplementaryFilter filter(0.5f);
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
    ComplementaryFilter filter(0.5f);
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
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
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

// --- Time constant ---

// Level, then the accelerometer reports a 30 deg roll with no gyro motion (the case
// where only the accelerometer correction acts). After one time constant the
// estimate must be ~63% of the way there, at any sample rate.
static float rollAfter(float tauS, float rateHz, float seconds) {
    ComplementaryFilter filter(tauS);
    filter.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // Snap level
    const float angleRad = 30.0f * DEG2RAD;
    const float dt = 1.0f / rateHz;
    const int steps = static_cast<int>(seconds * rateHz + 0.5f);
    for (int i = 0; i < steps; i++) {
        filter.update(0.0f, std::sin(angleRad), std::cos(angleRad), 0.0f, 0.0f, 0.0f, dt);
    }
    return filter.getRoll();
}

void test_time_constant_independent_of_rate() {
    const float expected = 30.0f * (1.0f - std::exp(-1.0f));    // ~18.96 deg after 1 tau
    const float at400 = rollAfter(0.5f, 400.0f, 0.5f);
    const float at1000 = rollAfter(0.5f, 1000.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, expected, at400);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, expected, at1000);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, at400, at1000);
}

void test_invalid_time_constant_rejected() {
    ComplementaryFilter filter(0.5f);
    TEST_ASSERT_FALSE(filter.setTimeConstant(0.0f));
    TEST_ASSERT_FALSE(filter.setTimeConstant(-1.0f));
    TEST_ASSERT_FALSE(filter.setTimeConstant(NAN));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, filter.getTimeConstant());
    TEST_ASSERT_TRUE(filter.setTimeConstant(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, filter.getTimeConstant());
}

// --- Estimator timing (dt from sample timestamps) ---

struct AttitudeSink {
    AttitudeData last{};
    int count = 0;
};

static void subscribeSink(AttitudeSink& sink) {
    EventService::instance().subscribe(EventType::ATTITUDE_UPDATED, [](void* c, const void* data) {
        auto* s = static_cast<AttitudeSink*>(c);
        s->last = *static_cast<const AttitudeData*>(data);
        s->count++;
    }, &sink);
}

static void publishImu(float gx, uint32_t timestampUs, float ay = 0.0f, float az = 1.0f) {
    ImuData imu{0.0f, ay, az, gx, 0.0f, 0.0f, timestampUs};
    EventService::instance().publish(EventType::IMU_RAW, &imu);
}

void test_invalid_construction_produces_no_attitude() {
    ComplementaryFilter filter(0.0f);
    TEST_ASSERT_FALSE(filter.hasValidConfig());
    filter.update(0.0f, 0.5f, 0.866f, 0.0f, 0.0f, 0.0f, 0.0025f);
    TEST_ASSERT_FALSE(filter.isInitialized());           // Nothing estimated

    AttitudeEstimatorModule<ComplementaryFilter> estimator(NAN);
    estimator.setup();
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::ERROR));  // Loud at setup
    AttitudeSink sink;
    subscribeSink(sink);
    publishImu(0.0f, 1000);
    publishImu(0.0f, 3500);
    TEST_ASSERT_EQUAL(0, sink.count);                    // Never publishes: arming stays blocked
}

void test_dt_comes_from_timestamps() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
    estimator.setup();
    ComplementaryFilter reference(0.5f);

    // 100 deg/s roll for 0.25 s at 400 Hz, timestamps 2500 us apart
    publishImu(0.0f, 1000);
    reference.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    for (uint32_t i = 1; i <= 100; i++) {
        publishImu(100.0f, 1000 + i * 2500);
        reference.update(0.0f, 0.0f, 1.0f, 100.0f, 0.0f, 0.0f, 0.0025f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, reference.getRoll(), estimator.getRoll());
    TEST_ASSERT_GREATER_THAN(15.0f, estimator.getRoll());   // ~25 deg of gyro minus the accel pull-back
}

void test_microsecond_wraparound_is_not_a_gap() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
    estimator.setup();
    AttitudeSink sink;
    subscribeSink(sink);

    uint32_t t = UINT32_MAX - 10 * 2500;
    for (int i = 0; i < 20; i++, t += 2500) publishImu(0.0f, t);   // Crosses 2^32
    TEST_ASSERT_EQUAL(20, sink.count);
    TEST_ASSERT_EQUAL_UINT32(0, estimator.gapResets());
    TEST_ASSERT_EQUAL(0, g_mockLog.count(LogLevel::WARN));
}

void test_gap_reinitializes_and_skips_publish() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
    estimator.setup();
    AttitudeSink sink;
    subscribeSink(sink);

    for (uint32_t i = 0; i < 10; i++) publishImu(0.0f, i * 2500);   // Level
    TEST_ASSERT_EQUAL(10, sink.count);

    // 100 ms gap, then the drone is at 30 deg roll
    const float a = 30.0f * DEG2RAD;
    publishImu(0.0f, 9 * 2500 + 100000, std::sin(a), std::cos(a));
    TEST_ASSERT_EQUAL(10, sink.count);                     // Reset sample NOT published
    TEST_ASSERT_EQUAL_UINT32(1, estimator.gapResets());
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::WARN));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, estimator.getRoll());  // Snapped, not integrated

    publishImu(0.0f, 9 * 2500 + 102500, std::sin(a), std::cos(a));
    TEST_ASSERT_EQUAL(11, sink.count);                     // Normal again
}

void test_timestamp_going_backwards_is_a_gap() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
    estimator.setup();
    publishImu(0.0f, 50000);
    publishImu(0.0f, 52500);
    publishImu(0.0f, 40000);                               // Earlier than the last sample
    TEST_ASSERT_EQUAL_UINT32(1, estimator.gapResets());
}

void test_repeated_timestamp_is_skipped() {
    AttitudeEstimatorModule<ComplementaryFilter> estimator(0.5f);
    estimator.setup();
    AttitudeSink sink;
    subscribeSink(sink);
    publishImu(0.0f, 1000);
    publishImu(100.0f, 3500);
    publishImu(100.0f, 3500);                              // Same sample again
    TEST_ASSERT_EQUAL(2, sink.count);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_filter_level_steady_state);
    RUN_TEST(test_filter_roll_step_convergence);
    RUN_TEST(test_filter_pitch_step_convergence);
    RUN_TEST(test_filter_yaw_integration);
    RUN_TEST(test_filter_gyro_fast_response_roll);
    RUN_TEST(test_module_event_integration);
    RUN_TEST(test_time_constant_independent_of_rate);
    RUN_TEST(test_invalid_time_constant_rejected);
    RUN_TEST(test_invalid_construction_produces_no_attitude);
    RUN_TEST(test_dt_comes_from_timestamps);
    RUN_TEST(test_microsecond_wraparound_is_not_a_gap);
    RUN_TEST(test_gap_reinitializes_and_skips_publish);
    RUN_TEST(test_timestamp_going_backwards_is_a_gap);
    RUN_TEST(test_repeated_timestamp_is_skipped);

    return UNITY_END();
}
