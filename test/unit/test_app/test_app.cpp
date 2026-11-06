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
    MockModule(const char* name)
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
    TEST_ASSERT_EQUAL_STRING("Module1", testApp.getModule(0)->getName());
    TEST_ASSERT_EQUAL_STRING("Module2", testApp.getModule(1)->getName());
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

    // Try to register one extra: must be refused loudly, not silently ignored
    g_mockLog.reset();
    MockModule extra("Extra");
    TEST_ASSERT_FALSE(testApp.registerModule(&extra));
    TEST_ASSERT_FALSE(testApp.registerModule(nullptr));

    TEST_ASSERT_EQUAL(MAX, testApp.getModuleCount());
    TEST_ASSERT_EQUAL(2, g_mockLog.count(LogLevel::ERROR));
    TEST_ASSERT_TRUE(g_mockLog.contains("'Extra' not registered"));
}

void test_app_register_returns_true_on_success() {
    App testApp;
    MockModule m1("Module1");
    TEST_ASSERT_TRUE(testApp.registerModule(&m1));
}

// --- Loop timing (LOOP_STATS) ---

// A module whose loop() takes `runUs` of (mock) time
class SlowModule : public Module {
public:
    uint32_t runUs;
    explicit SlowModule(uint32_t r) : Module("Slow"), runUs(r) {}
    void setup() override {}
    void loop() override { g_mockMicros += runUs; }
};

static int g_statsEvents = 0;
static LoopStatsData g_lastStats{};
static void onLoopStats(void*, const void* data) {
    g_statsEvents++;
    g_lastStats = *static_cast<const LoopStatsData*>(data);
}

// Runs `passes` loop passes, one every periodUs (mock clock)
static void runLoop(App& app, int passes, uint32_t periodUs) {
    for (int i = 0; i < passes; i++) {
        app.loop();
        g_mockMicros += periodUs;
        g_mockMillis = g_mockMicros / 1000;
    }
}

static void resetLoopTiming() {
    EventService::instance().reset();
    g_mockLog.reset();
    g_mockMicros = 0;
    g_mockMillis = 0;
    g_statsEvents = 0;
    g_lastStats = {};
    EventService::instance().subscribe(EventType::LOOP_STATS, onLoopStats, nullptr);
}

void test_app_publishes_loop_stats_once_per_window() {
    resetLoopTiming();
    App app(2500);
    SlowModule slow(200);
    app.registerModule(&slow);
    app.setup();

    runLoop(app, 300, 2300);                 // 2300 us idle + 200 us run = 2500 us per pass
    TEST_ASSERT_EQUAL(0, g_statsEvents);     // 0.75 s: window not complete
    runLoop(app, 120, 2300);                 // past 1 s
    TEST_ASSERT_EQUAL(1, g_statsEvents);
    TEST_ASSERT_UINT32_WITHIN(2, 400, g_lastStats.rateHz);
    TEST_ASSERT_EQUAL_UINT32(200, g_lastStats.maxRunUs);
    TEST_ASSERT_EQUAL_UINT32(2500, g_lastStats.maxGapUs);
    TEST_ASSERT_EQUAL_UINT32(0, g_lastStats.overruns);
    TEST_ASSERT_EQUAL(0, g_mockLog.count(LogLevel::WARN));
}

void test_app_warns_on_overruns_once_per_window() {
    resetLoopTiming();
    App app(2500);
    SlowModule slow(200);
    app.registerModule(&slow);
    app.setup();

    slow.runUs = 3000;                       // Every pass misses the 2500 us deadline
    runLoop(app, 500, 0);                    // 1.5 s of back-to-back overrunning passes
    TEST_ASSERT_EQUAL(1, g_statsEvents);
    TEST_ASSERT_TRUE(g_lastStats.overruns > 300);
    TEST_ASSERT_EQUAL_UINT32(3000, g_lastStats.maxRunUs);
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::WARN));   // One line per window, not per pass
    TEST_ASSERT_TRUE(g_mockLog.contains("loop overruns"));
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
    RUN_TEST(test_app_register_returns_true_on_success);
    RUN_TEST(test_app_publishes_loop_stats_once_per_window);
    RUN_TEST(test_app_warns_on_overruns_once_per_window);

    return UNITY_END();
}