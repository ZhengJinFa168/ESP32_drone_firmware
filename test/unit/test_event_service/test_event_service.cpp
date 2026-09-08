#include <unity.h>
#include "core/eventService/EventService.h"
#include "core/Module.h"

// Test module to verify Module-level publish and subscribe
class TestEventModule : public Module {
public:
    int heartbeatCount = 0;

    TestEventModule() : Module("TestEventModule") {}

    void setup() override {
        subscribe(EventType::HEARTBEAT, [](void* ctx, const void*) {
            static_cast<TestEventModule*>(ctx)->heartbeatCount++;
        });
    }

    // loop() not needed — uses default empty base implementation

    void sendHeartbeat() {
        publish(EventType::HEARTBEAT, nullptr);
    }
};

void setUp() {
    EventService::instance().reset();
}

void tearDown() {
    EventService::instance().reset();
}

void test_subscribe_and_publish() {
    bool callbackFired = false;
    EventService::instance().subscribe(EventType::HEARTBEAT, [](void* ctx, const void*) {
        *static_cast<bool*>(ctx) = true;
    }, &callbackFired);

    EventService::instance().publish(EventType::HEARTBEAT);
    TEST_ASSERT_TRUE(callbackFired);
}

void test_multiple_subscribers() {
    int count1 = 0;
    int count2 = 0;

    EventService::instance().subscribe(EventType::HEARTBEAT, [](void* ctx, const void*) {
        (*static_cast<int*>(ctx))++;
    }, &count1);
    EventService::instance().subscribe(EventType::HEARTBEAT, [](void* ctx, const void*) {
        (*static_cast<int*>(ctx))++;
    }, &count2);

    EventService::instance().publish(EventType::HEARTBEAT);
    TEST_ASSERT_EQUAL(1, count1);
    TEST_ASSERT_EQUAL(1, count2);
}

void test_different_event_types() {
    bool imuFired = false;
    bool armedFired = false;

    EventService::instance().subscribe(EventType::IMU_RAW, [](void* ctx, const void*) {
        *static_cast<bool*>(ctx) = true;
    }, &imuFired);
    EventService::instance().subscribe(EventType::ARMED, [](void* ctx, const void*) {
        *static_cast<bool*>(ctx) = true;
    }, &armedFired);

    EventService::instance().publish(EventType::IMU_RAW);
    TEST_ASSERT_TRUE(imuFired);
    TEST_ASSERT_FALSE(armedFired);

    EventService::instance().publish(EventType::ARMED);
    TEST_ASSERT_TRUE(armedFired);
}

void test_publish_with_data() {
    struct Ctx { ImuData data{}; bool received = false; };
    Ctx ctx;

    EventService::instance().subscribe(EventType::IMU_RAW, [](void* c, const void* data) {
        if (data) {
            auto* ctx = static_cast<Ctx*>(c);
            ctx->data = *static_cast<const ImuData*>(data);
            ctx->received = true;
        }
    }, &ctx);

    ImuData sendData{1.0f, 2.0f, 9.81f, 0.1f, -0.2f, 0.05f};
    EventService::instance().publish(EventType::IMU_RAW, &sendData);

    TEST_ASSERT_TRUE(ctx.received);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ctx.data.ax);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ctx.data.ay);
    TEST_ASSERT_EQUAL_FLOAT(9.81f, ctx.data.az);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, ctx.data.gx);
    TEST_ASSERT_EQUAL_FLOAT(-0.2f, ctx.data.gy);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, ctx.data.gz);
}

void test_max_listeners_exceeded() {
    // MAX_LISTENERS_PER_EVENT is 4
    for (int i = 0; i < EventService::MAX_LISTENERS_PER_EVENT; i++) {
        bool ok = EventService::instance().subscribe(EventType::HEARTBEAT, [](void*, const void*) {});
        TEST_ASSERT_TRUE(ok);
    }

    // 5th listener should fail
    bool fifthOk = EventService::instance().subscribe(EventType::HEARTBEAT, [](void*, const void*) {});
    TEST_ASSERT_FALSE(fifthOk);
}

void test_publish_without_subscribers() {
    // Should not crash or throw
    EventService::instance().publish(EventType::DISARMED, nullptr);
    TEST_ASSERT_TRUE(true);
}

void test_reset_clears_subscriptions() {
    int calls = 0;
    EventService::instance().subscribe(EventType::HEARTBEAT, [](void* ctx, const void*) {
        (*static_cast<int*>(ctx))++;
    }, &calls);

    EventService::instance().publish(EventType::HEARTBEAT);
    TEST_ASSERT_EQUAL(1, calls);

    EventService::instance().reset();
    EventService::instance().publish(EventType::HEARTBEAT);
    TEST_ASSERT_EQUAL(1, calls);  // Did not increment after reset
}

void test_singleton_identity() {
    EventService& inst1 = EventService::instance();
    EventService& inst2 = EventService::instance();
    TEST_ASSERT_EQUAL_PTR(&inst1, &inst2);
}

void test_module_publish_subscribe() {
    TestEventModule module;
    module.setup();

    module.sendHeartbeat();
    TEST_ASSERT_EQUAL(1, module.heartbeatCount);

    module.sendHeartbeat();
    TEST_ASSERT_EQUAL(2, module.heartbeatCount);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_subscribe_and_publish);
    RUN_TEST(test_multiple_subscribers);
    RUN_TEST(test_different_event_types);
    RUN_TEST(test_publish_with_data);
    RUN_TEST(test_max_listeners_exceeded);
    RUN_TEST(test_publish_without_subscribers);
    RUN_TEST(test_reset_clears_subscriptions);
    RUN_TEST(test_singleton_identity);
    RUN_TEST(test_module_publish_subscribe);

    return UNITY_END();
}
