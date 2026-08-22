#include <unity.h>
// ============================================
// INCLUDE THE MODULE AND APP HEADERS
// ============================================
#include "core/App.h"
#include "core/Module.h"

// ============================================
// MOCK MODULE – For testing App behavior
// ============================================
class MockModule : public Module {
private:
    bool setupCalled;
    bool loopCalled;
    int loopCount;

public:
    // Default constructor (needed for arrays)
    MockModule() : Module("Mock"), setupCalled(false), loopCalled(false), loopCount(0) {}

    // Constructor with custom name
    MockModule(const String& name)
        : Module(name), setupCalled(false), loopCalled(false), loopCount(0) {}

    void setup() override {
        setupCalled = true;
    }

    void loop() override {
        loopCalled = true;
        loopCount++;
    }

    bool wasSetupCalled() const { return setupCalled; }
    bool wasLoopCalled() const { return loopCalled; }
    int getLoopCount() const { return loopCount; }
    void reset() {
        setupCalled = false;
        loopCalled = false;
        loopCount = 0;
    }
};

// ============================================
// TEST CASES
// ============================================

void test_app_registers_modules() {
    App testApp;
    MockModule m1("Module1");
    MockModule m2("Module2");

    testApp.registerModule(&m1);
    testApp.registerModule(&m2);
    
    TEST_ASSERT_EQUAL(2, testApp.getModuleCount());
    TEST_ASSERT_EQUAL_STRING("Module1", testApp.getModule(0)->getName().c_str());
    TEST_ASSERT_EQUAL_STRING("Module2", testApp.getModule(1)->getName().c_str());
}

void test_app_setup_calls_setup_on_all_modules() {
    App testApp;
    MockModule m1("Module1");
    MockModule m2("Module2");

    testApp.registerModule(&m1);
    testApp.registerModule(&m2);
    testApp.setup();

    TEST_ASSERT_TRUE(m1.wasSetupCalled());
    TEST_ASSERT_TRUE(m2.wasSetupCalled());
}

void test_app_loop_calls_loop_on_all_modules() {
    App testApp;
    MockModule m1("Module1");
    MockModule m2("Module2");

    testApp.registerModule(&m1);
    testApp.registerModule(&m2);
    testApp.setup();

    for (int i = 0; i < 5; i++) {
        testApp.loop();
    }

    TEST_ASSERT_TRUE(m1.wasLoopCalled());
    TEST_ASSERT_TRUE(m2.wasLoopCalled());
    TEST_ASSERT_EQUAL(5, m1.getLoopCount());
    TEST_ASSERT_EQUAL(5, m2.getLoopCount());
}

void test_app_disabled_module_does_not_run() {
    App testApp;
    MockModule m1("Module1");
    MockModule m2("Module2");

    testApp.registerModule(&m1);
    testApp.registerModule(&m2);

    m1.disable();

    testApp.setup();
    testApp.loop();

    TEST_ASSERT_TRUE(m1.wasSetupCalled());   // Setup is still called
    TEST_ASSERT_FALSE(m1.wasLoopCalled());   // Loop is skipped
    TEST_ASSERT_TRUE(m2.wasLoopCalled());    // Module 2 runs normally
}

void test_app_handles_max_modules() {
    App testApp;
    const int MAX = App::MAX_MODULES;

    // Create an array of MockModules
    MockModule modules[MAX];
    for (int i = 0; i < MAX; i++) {
        testApp.registerModule(&modules[i]);
    }

    // Try to register one extra (should be ignored)
    MockModule extra("Extra");
    testApp.registerModule(&extra);

    TEST_ASSERT_EQUAL(MAX, testApp.getModuleCount());
}

// ============================================
// TEST RUNNER
// ============================================
int main() {
    UNITY_BEGIN();

    RUN_TEST(test_app_registers_modules);
    RUN_TEST(test_app_setup_calls_setup_on_all_modules);
    RUN_TEST(test_app_loop_calls_loop_on_all_modules);
    RUN_TEST(test_app_disabled_module_does_not_run);
    RUN_TEST(test_app_handles_max_modules);

    return UNITY_END();
}