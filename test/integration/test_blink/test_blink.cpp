#include <Arduino.h>
#include <unity.h>
#include "core/App.h"
#include "core/Module.h"

// ============================================
// BLINK MODULE (same as in main.cpp)
// ============================================
class BlinkModule : public Module {
private:
    int pin;
    unsigned long lastToggle;
    int intervalMs;

public:
    BlinkModule(int p, int interval)
        : Module("BLINK"), pin(p), intervalMs(interval), lastToggle(0) {}

    void setup() override {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    void loop() override {
        unsigned long now = millis();
        if (now - lastToggle >= intervalMs) {
            digitalWrite(pin, !digitalRead(pin));
            lastToggle = now;
        }
    }

    bool isPinHigh() const {
        return digitalRead(pin) == HIGH;
    }
};

// ============================================
// TEST FIXTURE
// ============================================
void test_blink_toggles_led() {
    App testApp;
    BlinkModule testBlink(LED_BUILTIN, 500);
    testApp.registerModule(&testBlink);
    testApp.setup();

    bool firstState = testBlink.isPinHigh();
    Serial.println(firstState ? "Initial state: HIGH" : "Initial state: LOW");

    unsigned long start = millis();
    
    testApp.loop();

    bool secondState = testBlink.isPinHigh();
    Serial.println(secondState ? "Final state: HIGH" : "Final state: LOW");

    TEST_ASSERT_NOT_EQUAL(firstState, secondState);
}

void test_blink_disable_stops_toggling() {
    App testApp;
    BlinkModule testBlink(LED_BUILTIN, 100);
    testApp.registerModule(&testBlink);
    testApp.setup();

    // Helper lambda to run the app loop for a given duration (non-blocking)
    auto runFor = [&](unsigned long duration) {
        unsigned long start = millis();
        while (millis() - start < duration) {
            testApp.loop(); // CRITICAL: This runs the blink logic!
        }
    };

    // Run for 350ms (should toggle at 100ms, 200ms, 300ms -> ends HIGH at 350ms? 
    // Let's just run long enough to guarantee a toggle)
    runFor(150); // 150ms > 100ms, guarantees exactly 1 toggle (LOW -> HIGH)
    bool stateAfterStart = testBlink.isPinHigh(); // Expected: HIGH

    // Disable the module
    testBlink.disable();

    // Run for another 300ms (which would be 3 toggles if enabled)
    runFor(300);
    bool stillSameState = testBlink.isPinHigh(); // Expected: Still HIGH

    // Assert it never changed after disabling
    TEST_ASSERT_EQUAL(stateAfterStart, stillSameState); // PASSES!
}

// ============================================
// ARDUINO ENTRY POINT
// ============================================
void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial to stabilize

    UNITY_BEGIN();

    Serial.println("INIZIO TEST");
    delay(10000);

    RUN_TEST(test_blink_toggles_led);
    delay(1000);  // Wait for serial to stabilize
    RUN_TEST(test_blink_disable_stops_toggling);

    UNITY_END();
}

void loop() {
    // Empty – tests run once
}