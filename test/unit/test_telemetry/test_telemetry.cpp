#include <unity.h>
#include <cstring>
#include <string>
#include "modules/logic/telemetry/TelemetryFormatter.h"
#include "modules/logic/telemetry/TelemetryModule.h"

void setUp() {
    EventService::instance().reset();
    g_mockLog.reset();
    g_mockMillis = 0;
}
void tearDown() {}

static TelemetrySnapshot fullSnapshot() {
    TelemetrySnapshot s;
    s.haveAttitude = true;
    s.attitude = {1.5f, -2.25f, 90.0f};
    s.haveImu = true;
    s.imu = {0.01f, -0.02f, 1.0f, 0.5f, -0.5f, 10.0f, 0};
    s.haveStatus = true;
    s.status = {false, 0x0081, DisarmReason::NONE, true};
    s.haveRc = true;
    s.rc = {0.25f, 0.0f, -1.0f, 0.5f, {0, 0, 0, 0}, true};
    s.rc.aux[ARM_AUX_CHANNEL] = 1;
    s.imuFault = false;
    s.gyroCalibrated = true;
    s.droppedFrames = 3;
    return s;
}

static int countFrames(const char* text) {
    int n = 0;
    for (const char* p = text; (p = std::strstr(p, ">imuFault:")) != nullptr; p++) n++;
    return n;
}

// --- Formatter ---

void test_full_frame_exact_text() {
    char buf[TELEMETRY_FRAME_MAX];
    size_t len = 0;
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), fullSnapshot(), TelemetryGroup::ALL, len));
    TEST_ASSERT_EQUAL_STRING(
        ">roll:1.50\n>pitch:-2.25\n>yaw:90.00\n"
        ">gx:0.50\n>gy:-0.50\n>gz:10.00\n"
        ">ax:0.010\n>ay:-0.020\n>az:1.000\n"
        ">armed:0\n>blockers:129\n>rcLink:1\n"
        ">imuFault:0\n>gyroCal:1\n>tlmDrop:3\n"
        ">throttle:0.25\n>rcRoll:0.00\n>rcPitch:-1.00\n>rcYaw:0.50\n>armSwitch:1\n",
        buf);
    TEST_ASSERT_EQUAL_UINT32(std::strlen(buf), len);
}

void test_groups_select_lines() {
    char buf[TELEMETRY_FRAME_MAX];
    size_t len = 0;
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), fullSnapshot(), TelemetryGroup::ATTITUDE, len));
    TEST_ASSERT_EQUAL_STRING(">roll:1.50\n>pitch:-2.25\n>yaw:90.00\n", buf);
}

void test_values_never_received_are_not_sent() {
    char buf[TELEMETRY_FRAME_MAX];
    size_t len = 0;
    TelemetrySnapshot empty;
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), empty, TelemetryGroup::ATTITUDE, len));
    TEST_ASSERT_EQUAL_UINT32(0, len);                     // Nothing to send yet (not an error)
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), empty, TelemetryGroup::ALL, len));
    TEST_ASSERT_EQUAL_STRING(">imuFault:0\n>gyroCal:0\n>tlmDrop:0\n", buf);
}

void test_frame_too_big_is_refused_not_truncated() {
    char buf[64 + 8];
    std::memset(buf, '#', sizeof(buf));
    size_t len = 123;
    TEST_ASSERT_FALSE(formatTelemetry(buf, 64, fullSnapshot(), TelemetryGroup::ALL, len));
    TEST_ASSERT_EQUAL_UINT32(0, len);
    for (size_t i = 64; i < sizeof(buf); i++) TEST_ASSERT_EQUAL_CHAR('#', buf[i]);  // No overflow
}

void test_default_frame_fits_buffer() {
    char buf[TELEMETRY_FRAME_MAX];
    size_t len = 0;
    TelemetrySnapshot worst = fullSnapshot();
    worst.attitude = {-179.99f, -179.99f, -179.99f};
    worst.imu = {-15.999f, -15.999f, -15.999f, -1999.99f, -1999.99f, -1999.99f, 0};
    worst.status.blockers = 0xFFFF;
    worst.droppedFrames = 4294967295u;
    worst.haveLoop = true;
    worst.loop = {4294967295u, 4294967295u, 4294967295u, 4294967295u};
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), worst, TELEMETRY_GROUPS, len));
}

void test_loop_group_lines() {
    char buf[TELEMETRY_FRAME_MAX];
    size_t len = 0;
    TelemetrySnapshot s;
    s.haveLoop = true;
    s.loop = {400, 310, 2560, 2};
    TEST_ASSERT_TRUE(formatTelemetry(buf, sizeof(buf), s, TelemetryGroup::LOOP, len));
    TEST_ASSERT_EQUAL_STRING(">loopHz:400\n>loopRunMaxUs:310\n>loopGapMaxUs:2560\n>loopOverruns:2\n", buf);
}

// --- Sender ---

void test_sender_counts_drops_and_reports_them() {
    TelemetrySender sender;
    TelemetrySnapshot s;                                  // Status lines are always sent
    g_mockLog.txFree = 0;
    sender.send(s);
    sender.send(s);
    TEST_ASSERT_EQUAL_UINT32(2, sender.droppedFrames());
    g_mockLog.txFree = MockLog::TX_CAPTURE_SIZE;
    sender.send(s);
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">tlmDrop:2\n"));
}

// --- Module ---

void test_module_sends_at_configured_rate() {
    TelemetryModule<TelemetrySender> tlm;
    tlm.setup();
    AttitudeData att{1.0f, 2.0f, 3.0f};
    EventService::instance().publish(EventType::ATTITUDE_UPDATED, &att);

    const uint32_t period = 1000 / TELEMETRY_RATE_HZ;
    for (uint32_t t = 0; t <= 5 * period; t += period / 4) {
        g_mockMillis = t;
        tlm.loop();
    }
    TEST_ASSERT_EQUAL(5, countFrames(g_mockLog.tx));
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">roll:1.00\n"));
}

void test_module_drops_and_counts_when_buffer_full() {
    TelemetryModule<TelemetrySender> tlm;
    tlm.setup();
    const uint32_t period = 1000 / TELEMETRY_RATE_HZ;

    g_mockLog.txFree = 0;                                 // Serial TX buffer full
    g_mockMillis = period;
    tlm.loop();
    TEST_ASSERT_EQUAL_UINT32(1, tlm.droppedFrames());
    TEST_ASSERT_EQUAL_UINT32(0, g_mockLog.txLen);

    g_mockLog.txFree = MockLog::TX_CAPTURE_SIZE;          // Drained
    g_mockMillis = 2 * period;
    tlm.loop();
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">tlmDrop:1\n"));
}

void test_module_tracks_imu_fault_only() {
    TelemetryModule<TelemetrySender> tlm;
    tlm.setup();
    SensorFaultData baro{SensorId::BAROMETER, true, SensorFaultReason::READ_ERRORS};
    EventService::instance().publish(EventType::SENSOR_FAULT, &baro);
    g_mockMillis = 1000;
    tlm.loop();
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">imuFault:0\n"));

    SensorFaultData imu{SensorId::IMU, true, SensorFaultReason::ZERO_DATA};
    EventService::instance().publish(EventType::SENSOR_FAULT, &imu);
    GyroCalibrationData cal{true, 0.1f, 0.2f, 0.3f};
    EventService::instance().publish(EventType::GYRO_CALIBRATION, &cal);
    g_mockMillis = 2000;
    tlm.loop();
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">imuFault:1\n>gyroCal:1\n"));
}

void test_module_forwards_loop_stats() {
    TelemetryModule<TelemetrySender> tlm;
    tlm.setup();
    LoopStatsData stats{399, 420, 2600, 0};
    EventService::instance().publish(EventType::LOOP_STATS, &stats);
    g_mockMillis = 1000;
    tlm.loop();
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">loopHz:399\n>loopRunMaxUs:420\n>loopGapMaxUs:2600\n"));
}

// Stands in for the board output (Esp32TelemetryOutput): records what the module hands over
struct FakeOutput {
    int begins = 0;
    int sends = 0;
    TelemetrySnapshot last{};
    uint32_t dropped = 0;
    void begin() { begins++; }
    void send(const TelemetrySnapshot& s) { sends++; last = s; }
    void addDropped(uint32_t n) { dropped += n; }
    uint32_t droppedFrames() const { return dropped; }
    uint32_t oversizeFrames() const { return 0; }
};

void test_module_hands_latest_snapshot_to_output_once_per_period() {
    TelemetryModule<FakeOutput> tlm;
    tlm.setup();
    const uint32_t period = 1000 / TELEMETRY_RATE_HZ;

    AttitudeData a1{1.0f, 0.0f, 0.0f};
    EventService::instance().publish(EventType::ATTITUDE_UPDATED, &a1);
    AttitudeData a2{2.0f, 0.0f, 0.0f};
    EventService::instance().publish(EventType::ATTITUDE_UPDATED, &a2);  // Newer value wins

    for (uint32_t t = 0; t < period; t++) {
        g_mockMillis = t;
        tlm.loop();
    }
    TEST_ASSERT_EQUAL(0, tlm.droppedFrames());
    g_mockMillis = period;
    tlm.loop();
    tlm.loop();  // Same period: no second frame
    TEST_ASSERT_EQUAL_FLOAT(2.0f, tlm.output().last.attitude.roll);
    TEST_ASSERT_TRUE(tlm.output().last.haveAttitude);
    TEST_ASSERT_EQUAL(1, tlm.output().sends);
    TEST_ASSERT_EQUAL(1, tlm.output().begins);
    TEST_ASSERT_EQUAL(0, g_mockLog.txLen);  // The module itself never writes
}

void test_sender_add_dropped_shows_in_frame() {
    TelemetrySender sender;
    sender.addDropped(2);  // e.g. replaced in the board's mailbox
    TelemetrySnapshot s;
    s.haveStatus = true;
    sender.send(s);
    TEST_ASSERT_EQUAL_UINT32(2, sender.droppedFrames());
    TEST_ASSERT_NOT_NULL(std::strstr(g_mockLog.tx, ">tlmDrop:2\n"));
}

void test_subscribe_failure_is_logged() {
    // Fill IMU_RAW's listener slots, then the module's subscription must fail loudly
    for (int i = 0; i < EventService::MAX_LISTENERS_PER_EVENT; i++) {
        EventService::instance().subscribe(EventType::IMU_RAW, [](void*, const void*) {}, nullptr);
    }
    TelemetryModule<TelemetrySender> tlm;
    tlm.setup();
    TEST_ASSERT_TRUE(g_mockLog.contains("TELEMETRY: subscribe failed"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_frame_exact_text);
    RUN_TEST(test_groups_select_lines);
    RUN_TEST(test_values_never_received_are_not_sent);
    RUN_TEST(test_frame_too_big_is_refused_not_truncated);
    RUN_TEST(test_default_frame_fits_buffer);
    RUN_TEST(test_loop_group_lines);
    RUN_TEST(test_sender_counts_drops_and_reports_them);
    RUN_TEST(test_module_sends_at_configured_rate);
    RUN_TEST(test_module_drops_and_counts_when_buffer_full);
    RUN_TEST(test_module_tracks_imu_fault_only);
    RUN_TEST(test_module_forwards_loop_stats);
    RUN_TEST(test_module_hands_latest_snapshot_to_output_once_per_period);
    RUN_TEST(test_sender_add_dropped_shows_in_frame);
    RUN_TEST(test_subscribe_failure_is_logged);
    return UNITY_END();
}
