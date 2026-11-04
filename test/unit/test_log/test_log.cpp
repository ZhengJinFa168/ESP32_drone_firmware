#include <unity.h>
#include <cstring>
#include <cstdarg>
#include "core/Log.h"

void setUp() { g_mockLog.reset(); }
void tearDown() {}

// Calls logFormatLine with a variadic list, as the platform implementations do
static size_t formatLine(char* buf, size_t cap, LogLevel level, const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const size_t len = logFormatLine(buf, cap, level, tag, fmt, args);
    va_end(args);
    return len;
}

void test_line_has_level_tag_and_newline() {
    char buf[LOG_LINE_MAX];
    const size_t len = formatLine(buf, sizeof(buf), LogLevel::ERROR, "BMI270", "fault %d", 3);
    TEST_ASSERT_EQUAL_STRING("[E][BMI270] fault 3\n", buf);
    TEST_ASSERT_EQUAL_UINT32(std::strlen(buf), len);
}

void test_long_message_is_truncated_safely() {
    char buf[24];
    std::memset(buf, 'X', sizeof(buf));
    const size_t len = formatLine(buf, sizeof(buf), LogLevel::INFO, "TAG",
                                  "%s", "a message that is far too long for the buffer");
    TEST_ASSERT_EQUAL_UINT32(sizeof(buf) - 1, len);  // Uses the whole buffer
    TEST_ASSERT_EQUAL_CHAR('\n', buf[len - 1]);       // Still a complete line
    TEST_ASSERT_EQUAL_CHAR('\0', buf[len]);           // Still terminated, no overflow
}

void test_tiny_buffer_never_overflows() {
    char buf[4] = {'X', 'X', 'X', 'X'};
    const size_t len = formatLine(buf, 3, LogLevel::WARN, "TAG", "hello");
    TEST_ASSERT_EQUAL_UINT32(2, len);
    TEST_ASSERT_EQUAL_CHAR('\n', buf[1]);
    TEST_ASSERT_EQUAL_CHAR('\0', buf[2]);
    TEST_ASSERT_EQUAL_CHAR('X', buf[3]);              // Byte past cap untouched
}

void test_macros_record_level_and_text() {
    LOG_E("ARMING", "DISARMED, reason: %s", "CRASH");
    LOG_W("RC", "warn");
    LOG_I("APP", "info %d", 1);
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::ERROR));
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::WARN));
    TEST_ASSERT_EQUAL(1, g_mockLog.count(LogLevel::INFO));
    TEST_ASSERT_TRUE(g_mockLog.contains("[E][ARMING] DISARMED, reason: CRASH"));
}

void test_debug_is_compiled_out_at_default_level() {
    int evaluated = 0;
    LOG_D("APP", "%d", ++evaluated);                  // FW_LOG_LEVEL 2 = INFO
    TEST_ASSERT_EQUAL(0, g_mockLog.count(LogLevel::DEBUG));
    TEST_ASSERT_EQUAL(0, evaluated);                  // Arguments not evaluated
}

void test_try_write_drops_instead_of_blocking() {
    g_mockLog.txFree = 8;
    TEST_ASSERT_TRUE(logTryWrite(">a:1\n", 5));
    TEST_ASSERT_FALSE(logTryWrite(">b:2\n", 5));      // Only 3 bytes left
    TEST_ASSERT_EQUAL(1, g_mockLog.txDropped);
    TEST_ASSERT_EQUAL_UINT32(5, g_mockLog.txLen);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_line_has_level_tag_and_newline);
    RUN_TEST(test_long_message_is_truncated_safely);
    RUN_TEST(test_tiny_buffer_never_overflows);
    RUN_TEST(test_macros_record_level_and_text);
    RUN_TEST(test_debug_is_compiled_out_at_default_level);
    RUN_TEST(test_try_write_drops_instead_of_blocking);
    return UNITY_END();
}
