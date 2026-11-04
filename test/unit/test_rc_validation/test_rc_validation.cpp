#include <unity.h>
#include <cmath>
#include <cstring>
#include <limits>
#include "modules/communication/rc_input/RcValidation.h"
#include "modules/communication/rc_input/WebSocketRcPacket.h"

void setUp() {}
void tearDown() {}

static RcInputData validRc() {
    RcInputData rc = {0.5f, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}, true};
    return rc;
}

static bool parse(const char* msg, RcInputData& out) {
    return parseWebSocketRcPacket(msg, std::strlen(msg), out);
}

// --- Range validation ---

void test_valid_packet_accepted() {
    RcInputData rc = validRc();
    TEST_ASSERT_TRUE(isValidRcInput(rc));
    rc.throttle = 1.0f; rc.roll = -1.0f; rc.pitch = 1.0f; rc.yaw = -1.0f;  // Limits are inclusive
    rc.aux[ARM_AUX_CHANNEL] = 1;
    TEST_ASSERT_TRUE(isValidRcInput(rc));
}

void test_out_of_range_rejected_not_clamped() {
    RcInputData rc = validRc();
    rc.throttle = 1.01f;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.throttle = -0.01f;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.roll = -1.5f;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.pitch = 2.0f;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.yaw = 1.0001f;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
}

void test_nan_and_infinity_rejected() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    RcInputData rc = validRc(); rc.throttle = nan;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.roll = nan;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
    rc = validRc(); rc.yaw = -inf;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
}

void test_invalid_arm_switch_rejected() {
    RcInputData rc = validRc();
    rc.aux[ARM_AUX_CHANNEL] = 2;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
}

void test_failsafe_packet_is_not_valid_input() {
    RcInputData rc = validRc();
    rc.linkOk = false;
    TEST_ASSERT_FALSE(isValidRcInput(rc));
}

// --- WebSocket packet parsing ---

void test_parse_valid_packet() {
    RcInputData rc{};
    TEST_ASSERT_TRUE(parse("T:0.25,Y:-0.50,P:0.10,R:-1.00,A:1", rc));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, rc.throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.50f, rc.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.10f, rc.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.00f, rc.roll);
    TEST_ASSERT_EQUAL_UINT8(1, rc.aux[ARM_AUX_CHANNEL]);
    TEST_ASSERT_TRUE(rc.linkOk);
    TEST_ASSERT_TRUE(isValidRcInput(rc));
}

void test_parse_rejects_malformed() {
    RcInputData rc = validRc();
    const RcInputData before = rc;
    TEST_ASSERT_FALSE(parse("T:0.25,Y:0.00,P:0.00,R:0.00", rc));        // Missing arm field
    TEST_ASSERT_FALSE(parse("T:0.25,Y:0.00,P:0.00,R:0.00,A:1X", rc));   // Trailing garbage
    TEST_ASSERT_FALSE(parse("T:0.25,Y:0.00,P:0.00,R:0.00,A:1\n", rc));  // Trailing newline
    TEST_ASSERT_FALSE(parse("hello", rc));
    TEST_ASSERT_FALSE(parse("", rc));
    TEST_ASSERT_EQUAL_MEMORY(&before, &rc, sizeof(rc));                  // Output untouched
}

void test_parse_rejects_bad_arm_values() {
    RcInputData rc{};
    TEST_ASSERT_FALSE(parse("T:0.00,Y:0.00,P:0.00,R:0.00,A:2", rc));
    TEST_ASSERT_FALSE(parse("T:0.00,Y:0.00,P:0.00,R:0.00,A:256", rc));  // Would wrap to 0 in a uint8_t
    TEST_ASSERT_FALSE(parse("T:0.00,Y:0.00,P:0.00,R:0.00,A:-1", rc));
}

void test_parse_accepts_nan_but_validation_rejects_it() {
    RcInputData rc{};
    TEST_ASSERT_TRUE(parse("T:nan,Y:0.00,P:0.00,R:0.00,A:0", rc));      // sscanf reads "nan"
    TEST_ASSERT_FALSE(isValidRcInput(rc));                              // ...the range check catches it
}

void test_parse_rejects_oversized_and_unterminated_input() {
    char big[WS_RC_MSG_MAX + 10];
    std::memset(big, '0', sizeof(big));
    RcInputData rc{};
    TEST_ASSERT_FALSE(parseWebSocketRcPacket(big, sizeof(big), rc));     // Longer than the limit

    // Frame is not null-terminated: only `len` bytes may be read
    const char frame[] = "T:0.00,Y:0.00,P:0.00,R:0.00,A:0GARBAGE";
    TEST_ASSERT_TRUE(parseWebSocketRcPacket(frame, 31, rc));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_valid_packet_accepted);
    RUN_TEST(test_out_of_range_rejected_not_clamped);
    RUN_TEST(test_nan_and_infinity_rejected);
    RUN_TEST(test_invalid_arm_switch_rejected);
    RUN_TEST(test_failsafe_packet_is_not_valid_input);
    RUN_TEST(test_parse_valid_packet);
    RUN_TEST(test_parse_rejects_malformed);
    RUN_TEST(test_parse_rejects_bad_arm_values);
    RUN_TEST(test_parse_accepts_nan_but_validation_rejects_it);
    RUN_TEST(test_parse_rejects_oversized_and_unterminated_input);
    return UNITY_END();
}
