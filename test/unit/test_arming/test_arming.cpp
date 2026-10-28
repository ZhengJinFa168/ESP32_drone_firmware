#include <unity.h>
#include <cstdint>
#include "modules/logic/arming/ArmingLogic.h"
#include "modules/logic/arming/ArmingModule.h"
#include "config/SensorConfig.h"

using Transition = ArmingLogic::Transition;

static RcInputData makeRc(float throttle, bool armSwitch, bool linkOk = true) {
    RcInputData rc{throttle, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}, linkOk};
    rc.aux[ARM_AUX_CHANNEL] = armSwitch ? 1 : 0;
    return rc;
}

// Simulates the event stream: every 10 ms an RC packet and an attitude update
// (unless disabled), then ArmingLogic::update().
struct Sim {
    ArmingLogic logic;
    uint32_t t;
    RcInputData rc = makeRc(0.0f, false);
    AttitudeData attitude{0.0f, 0.0f, 0.0f};
    bool sendRc = true;
    bool sendAttitude = true;
    int armedCount = 0;
    int disarmedCount = 0;

    explicit Sim(bool alignmentVerified = true, uint32_t startMs = 0)
        : logic(alignmentVerified), t(startMs) {}

    void run(uint32_t ms) {
        for (uint32_t elapsed = 0; elapsed < ms; elapsed += 10) {
            t += 10;
            if (sendRc) logic.onRcInput(rc, t);
            if (sendAttitude) logic.onAttitude(attitude, t);
            Transition tr = logic.update(t);
            if (tr == Transition::ARMED) armedCount++;
            if (tr == Transition::DISARMED) disarmedCount++;
        }
    }

    // Good link (stable), switch OFF, throttle low, level
    void makeReady() { run(1100); }

    void armSwitch(bool on) { rc.aux[ARM_AUX_CHANNEL] = on ? 1 : 0; }

    void arm() {
        makeReady();
        armSwitch(true);
        run(10);
    }

    bool has(uint16_t blocker) const { return (logic.blockers() & blocker) != 0; }
};

void setUp() {
    EventService::instance().reset();
    g_mockMillis = 0;
}

void tearDown() {
    EventService::instance().reset();
}

// --- Boot and arming conditions ---

void test_boot_is_disarmed_with_blockers() {
    Sim s(false);
    s.logic.update(0);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ALIGNMENT_NOT_VERIFIED));
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::NO_RC_LINK));
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::THROTTLE_NOT_LOW));
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ARM_SWITCH_NOT_RESET));
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ATTITUDE_STALE));
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::NONE), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_arms_when_all_clear() {
    Sim s;
    s.makeReady();
    TEST_ASSERT_EQUAL_UINT16(0, s.logic.blockers());
    s.armSwitch(true);
    s.run(10);
    TEST_ASSERT_TRUE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(1, s.armedCount);
}

void test_link_must_be_stable_before_arming() {
    Sim s;
    s.run(500);  // Link good, but not yet for RC_LINK_STABLE_MS
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::NO_RC_LINK));
    s.armSwitch(true);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
}

void test_alignment_not_verified_blocks() {
    Sim s(false);
    s.arm();
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ALIGNMENT_NOT_VERIFIED));
}

void test_throttle_not_low_blocks() {
    Sim s;
    s.rc.throttle = 0.3f;
    s.arm();
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::THROTTLE_NOT_LOW));
}

void test_not_level_blocks() {
    Sim s;
    s.attitude.roll = 30.0f;
    s.arm();
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::NOT_LEVEL));
}

void test_stale_attitude_blocks() {
    Sim s;
    s.makeReady();
    s.sendAttitude = false;
    s.run(100);
    s.armSwitch(true);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ATTITUDE_STALE));
}

void test_sensor_fault_blocks() {
    Sim s;
    s.logic.onSensorFault(true);
    s.arm();
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::SENSOR_FAULT));
}

void test_switch_on_at_boot_never_arms() {
    Sim s;
    s.armSwitch(true);
    s.run(3000);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_TRUE(s.has(ArmingBlocker::ARM_SWITCH_NOT_RESET));
}

void test_blocked_attempt_requires_retoggle() {
    Sim s;
    s.rc.throttle = 0.3f;
    s.arm();                       // Blocked: throttle high
    TEST_ASSERT_FALSE(s.logic.isArmed());
    s.rc.throttle = 0.0f;          // Last blocker clears, switch still ON
    s.run(100);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    s.armSwitch(false);
    s.run(10);
    s.armSwitch(true);
    s.run(10);
    TEST_ASSERT_TRUE(s.logic.isArmed());
}

// --- Disarming ---

void test_switch_off_disarms() {
    Sim s;
    s.arm();
    s.armSwitch(false);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::SWITCH_OFF), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_rearm_requires_switch_off_after_fault_disarm() {
    Sim s;
    s.arm();
    s.logic.onSensorFault(true);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    s.logic.onSensorFault(false);  // Fault clears, switch still ON
    s.run(100);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    s.armSwitch(false);
    s.run(10);
    s.armSwitch(true);
    s.run(10);
    TEST_ASSERT_TRUE(s.logic.isArmed());
}

void test_sensor_fault_disarms_immediately() {
    Sim s;
    s.arm();
    s.logic.onSensorFault(true);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::SENSOR_FAULT), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_stale_attitude_disarms() {
    Sim s;
    s.arm();
    s.sendAttitude = false;
    s.run(40);
    TEST_ASSERT_TRUE(s.logic.isArmed());   // Within ATTITUDE_STALE_MS
    s.run(30);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::ATTITUDE_STALE), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_crash_disarms_only_when_sustained() {
    Sim s;
    s.arm();
    s.attitude.roll = 85.0f;
    s.run(100);                            // Short spike
    TEST_ASSERT_TRUE(s.logic.isArmed());
    s.attitude.roll = 0.0f;
    s.run(50);
    s.attitude.pitch = -85.0f;
    s.run(250);                            // Sustained beyond CRASH_TIME_MS
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::CRASH), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_auto_disarm_when_throttle_stays_low() {
    Sim s;
    s.arm();
    s.run(AUTO_DISARM_MS - 100);
    TEST_ASSERT_TRUE(s.logic.isArmed());
    s.run(200);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::AUTO_DISARM), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_raising_throttle_cancels_auto_disarm() {
    Sim s;
    s.arm();
    s.rc.throttle = 0.2f;
    s.run(100);
    s.rc.throttle = 0.0f;
    s.run(AUTO_DISARM_MS + 2000);
    TEST_ASSERT_TRUE(s.logic.isArmed());
}

// --- RC link loss while armed ---

void test_rc_timeout_keeps_armed_then_disarms() {
    Sim s;
    s.arm();
    s.rc.throttle = 0.5f;                  // Flying
    s.run(100);
    s.sendRc = false;                      // Packets stop
    s.run(RC_LOSS_HOLD_MS);
    TEST_ASSERT_TRUE(s.logic.isArmed());   // Hold phase: still armed
    TEST_ASSERT_FALSE(s.logic.rcLinkOk());
    TEST_ASSERT_TRUE(s.logic.rcLostForMs(s.t) > 0);
    s.run(RC_LOSS_MAX_ARMED_MS - RC_LOSS_HOLD_MS);
    TEST_ASSERT_TRUE(s.logic.isArmed());   // Loss detected RC_LINK_TIMEOUT_MS after the last packet
    s.run(RC_LINK_TIMEOUT_MS + 100);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::RC_LOSS_TIMEOUT), static_cast<int>(s.logic.lastDisarmReason()));
}

void test_failsafe_packet_counts_as_link_loss() {
    Sim s;
    s.arm();
    s.rc.throttle = 0.5f;
    s.run(100);
    s.rc = makeRc(0.0f, false, false);     // RcInputModule failsafe packet: linkOk = false
    s.run(20);
    TEST_ASSERT_FALSE(s.logic.rcLinkOk());
    TEST_ASSERT_TRUE(s.logic.isArmed());   // Not a switch-off: values are not from the pilot
}

void test_rc_recovery_returns_control() {
    Sim s;
    s.arm();
    s.rc.throttle = 0.5f;
    s.run(100);
    s.sendRc = false;
    s.run(2000);
    TEST_ASSERT_FALSE(s.logic.rcLinkOk());
    s.sendRc = true;                       // Link back
    s.run(RC_LINK_STABLE_MS - 100);
    TEST_ASSERT_FALSE(s.logic.rcLinkOk()); // Not stable long enough yet
    s.run(200);
    TEST_ASSERT_TRUE(s.logic.rcLinkOk());
    TEST_ASSERT_EQUAL_UINT32(0, s.logic.rcLostForMs(s.t));
    s.run(RC_LOSS_MAX_ARMED_MS + 1000);
    TEST_ASSERT_TRUE(s.logic.isArmed());   // Loss timer stopped after recovery
}

void test_switch_off_during_recovery_disarms() {
    Sim s;
    s.arm();
    s.rc.throttle = 0.5f;
    s.run(100);
    s.sendRc = false;
    s.run(1000);
    s.sendRc = true;                       // First good packets: switch OFF must act immediately
    s.armSwitch(false);
    s.run(10);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::SWITCH_OFF), static_cast<int>(s.logic.lastDisarmReason()));
}

// --- Clock wrap-around ---

void test_timing_survives_clock_wraparound() {
    Sim s(true, UINT32_MAX - 2000);        // millis() wraps during the test
    s.arm();
    TEST_ASSERT_TRUE(s.logic.isArmed());
    s.run(AUTO_DISARM_MS - 100);
    TEST_ASSERT_TRUE(s.logic.isArmed());
    s.run(200);
    TEST_ASSERT_FALSE(s.logic.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::AUTO_DISARM), static_cast<int>(s.logic.lastDisarmReason()));
}

// --- Real configuration ---

void test_real_config_arms_only_if_alignment_verified() {
    Sim s(IMU_ALIGNMENT_VERIFIED);
    s.arm();
    TEST_ASSERT_EQUAL(IMU_ALIGNMENT_VERIFIED, s.logic.isArmed());
}

// --- Module: events in, events out ---

struct EventCounts {
    int armed = 0;
    int disarmed = 0;
    int status = 0;
    ArmingStatusData last{};
};

void test_module_publishes_events() {
    EventCounts counts;
    EventService::instance().subscribe(EventType::ARMED, [](void* c, const void*) {
        static_cast<EventCounts*>(c)->armed++;
    }, &counts);
    EventService::instance().subscribe(EventType::DISARMED, [](void* c, const void*) {
        static_cast<EventCounts*>(c)->disarmed++;
    }, &counts);
    EventService::instance().subscribe(EventType::ARMING_STATUS, [](void* c, const void* d) {
        auto* ec = static_cast<EventCounts*>(c);
        ec->status++;
        ec->last = *static_cast<const ArmingStatusData*>(d);
    }, &counts);

    ArmingModule arming(true);
    arming.setup();
    TEST_ASSERT_EQUAL(1, counts.status);   // Initial status published at setup
    TEST_ASSERT_FALSE(counts.last.armed);

    RcInputData rc = makeRc(0.0f, false);
    AttitudeData level{0.0f, 0.0f, 0.0f};
    auto feed = [&](uint32_t ms) {
        for (uint32_t e = 0; e < ms; e += 10) {
            g_mockMillis += 10;
            EventService::instance().publish(EventType::RC_INPUT, &rc);
            EventService::instance().publish(EventType::ATTITUDE_UPDATED, &level);
        }
    };

    feed(1100);
    rc.aux[ARM_AUX_CHANNEL] = 1;
    feed(10);
    TEST_ASSERT_TRUE(arming.isArmed());
    TEST_ASSERT_EQUAL(1, counts.armed);
    TEST_ASSERT_TRUE(counts.last.armed);

    rc.aux[ARM_AUX_CHANNEL] = 0;
    feed(10);
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_EQUAL(1, counts.disarmed);
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::SWITCH_OFF), static_cast<int>(counts.last.lastDisarmReason));

    SensorFaultData fault{true};
    EventService::instance().publish(EventType::SENSOR_FAULT, &fault);
    TEST_ASSERT_TRUE((counts.last.blockers & ArmingBlocker::SENSOR_FAULT) != 0);
}

void test_module_loop_fires_timeouts_without_events() {
    ArmingModule arming(true);
    arming.setup();
    RcInputData rc = makeRc(0.0f, false);
    AttitudeData level{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 111; i++) {
        g_mockMillis += 10;
        EventService::instance().publish(EventType::RC_INPUT, &rc);
        EventService::instance().publish(EventType::ATTITUDE_UPDATED, &level);
    }
    rc.aux[ARM_AUX_CHANNEL] = 1;
    g_mockMillis += 10;
    EventService::instance().publish(EventType::RC_INPUT, &rc);
    TEST_ASSERT_TRUE(arming.isArmed());

    g_mockMillis += 100;                   // No events at all: attitude goes stale
    arming.loop();
    TEST_ASSERT_FALSE(arming.isArmed());
    TEST_ASSERT_EQUAL(static_cast<int>(DisarmReason::ATTITUDE_STALE), static_cast<int>(arming.status().lastDisarmReason));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_boot_is_disarmed_with_blockers);
    RUN_TEST(test_arms_when_all_clear);
    RUN_TEST(test_link_must_be_stable_before_arming);
    RUN_TEST(test_alignment_not_verified_blocks);
    RUN_TEST(test_throttle_not_low_blocks);
    RUN_TEST(test_not_level_blocks);
    RUN_TEST(test_stale_attitude_blocks);
    RUN_TEST(test_sensor_fault_blocks);
    RUN_TEST(test_switch_on_at_boot_never_arms);
    RUN_TEST(test_blocked_attempt_requires_retoggle);

    RUN_TEST(test_switch_off_disarms);
    RUN_TEST(test_rearm_requires_switch_off_after_fault_disarm);
    RUN_TEST(test_sensor_fault_disarms_immediately);
    RUN_TEST(test_stale_attitude_disarms);
    RUN_TEST(test_crash_disarms_only_when_sustained);
    RUN_TEST(test_auto_disarm_when_throttle_stays_low);
    RUN_TEST(test_raising_throttle_cancels_auto_disarm);

    RUN_TEST(test_rc_timeout_keeps_armed_then_disarms);
    RUN_TEST(test_failsafe_packet_counts_as_link_loss);
    RUN_TEST(test_rc_recovery_returns_control);
    RUN_TEST(test_switch_off_during_recovery_disarms);

    RUN_TEST(test_timing_survives_clock_wraparound);
    RUN_TEST(test_real_config_arms_only_if_alignment_verified);

    RUN_TEST(test_module_publishes_events);
    RUN_TEST(test_module_loop_fires_timeouts_without_events);

    return UNITY_END();
}
