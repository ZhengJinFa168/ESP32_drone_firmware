#include <unity.h>
#include <cstdint>
#include "core/LoopStats.h"

void setUp() {}
void tearDown() {}

// Runs passes every periodUs taking runUs each, from t0 (us); returns true and fills
// `out` on the first completed window. Wall clock in ms = us / 1000.
static bool runPasses(LoopStats& stats, uint32_t t0Us, uint32_t periodUs, uint32_t runUs, int count,
                      LoopStatsData& out) {
    for (int i = 0; i < count; i++) {
        const uint32_t start = t0Us + static_cast<uint32_t>(i) * periodUs;
        if (stats.record(start, start + runUs, start / 1000, out)) return true;
    }
    return false;
}

void test_no_report_before_window_ends() {
    LoopStats stats(2500, 1000);
    LoopStatsData out{};
    TEST_ASSERT_FALSE(runPasses(stats, 0, 2500, 100, 400, out));   // 0 .. 997.5 ms
}

void test_rate_run_and_gap_at_400hz() {
    LoopStats stats(2500, 1000);
    LoopStatsData out{};
    TEST_ASSERT_TRUE(runPasses(stats, 0, 2500, 300, 401, out));    // Pass 401 starts at 1000 ms
    TEST_ASSERT_EQUAL_UINT32(401, out.rateHz);                     // 401 passes in 1000 ms
    TEST_ASSERT_EQUAL_UINT32(300, out.maxRunUs);
    TEST_ASSERT_EQUAL_UINT32(2500, out.maxGapUs);
    TEST_ASSERT_EQUAL_UINT32(0, out.overruns);
}

void test_overruns_and_longest_gap() {
    LoopStats stats(2500, 1000);
    LoopStatsData out{};
    stats.record(0, 100, 0, out);
    stats.record(2500, 2600, 2, out);
    stats.record(5000, 8000, 5, out);        // 3000 us pass: overrun
    stats.record(9000, 9100, 9, out);        // 4000 us gap after it
    TEST_ASSERT_TRUE(stats.record(1000000, 1000100, 1000, out));
    TEST_ASSERT_EQUAL_UINT32(1, out.overruns);
    TEST_ASSERT_EQUAL_UINT32(3000, out.maxRunUs);
    TEST_ASSERT_EQUAL_UINT32(1000000 - 9000, out.maxGapUs);   // The long idle gap before the last pass
}

void test_zero_period_disables_overrun_check() {
    LoopStats stats(0, 1000);
    LoopStatsData out{};
    stats.record(0, 50000, 0, out);
    TEST_ASSERT_TRUE(stats.record(1000000, 1000001, 1000, out));
    TEST_ASSERT_EQUAL_UINT32(0, out.overruns);
    TEST_ASSERT_EQUAL_UINT32(50000, out.maxRunUs);
}

void test_window_resets_but_gap_continues() {
    LoopStats stats(2500, 1000);
    LoopStatsData out{};
    TEST_ASSERT_TRUE(runPasses(stats, 0, 2500, 3000, 401, out));   // Every pass an overrun
    TEST_ASSERT_EQUAL_UINT32(401, out.overruns);
    // Next window: normal passes; the first gap (from the last pass of the window) counts
    LoopStatsData next{};
    TEST_ASSERT_TRUE(runPasses(stats, 1002500, 2500, 100, 401, next));
    TEST_ASSERT_EQUAL_UINT32(0, next.overruns);
    TEST_ASSERT_EQUAL_UINT32(100, next.maxRunUs);
    TEST_ASSERT_EQUAL_UINT32(2500, next.maxGapUs);
}

void test_microsecond_clock_wraparound() {
    LoopStats stats(2500, 1000);
    LoopStatsData out{};
    const uint32_t nearWrap = 0xFFFFFFFFu - 1000;
    stats.record(nearWrap, nearWrap + 200, 0, out);         // Run crosses the wrap
    stats.record(nearWrap + 2500, nearWrap + 2600, 3, out); // Gap crosses the wrap
    TEST_ASSERT_TRUE(stats.record(nearWrap + 5000, nearWrap + 5100, 1000, out));
    TEST_ASSERT_EQUAL_UINT32(200, out.maxRunUs);
    TEST_ASSERT_EQUAL_UINT32(2500, out.maxGapUs);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_no_report_before_window_ends);
    RUN_TEST(test_rate_run_and_gap_at_400hz);
    RUN_TEST(test_overruns_and_longest_gap);
    RUN_TEST(test_zero_period_disables_overrun_check);
    RUN_TEST(test_window_resets_but_gap_continues);
    RUN_TEST(test_microsecond_clock_wraparound);
    return UNITY_END();
}
