#pragma once
#include <cstdint>

// ============================================
// CLOCK (platform-independent time source)
// ============================================
// Logic modules take time only from here, never from Arduino's millis()/micros(),
// so they stay free of chip/framework headers and run unchanged in native tests.
//
// clockMs(): milliseconds since boot (wraps after ~49.7 days)
// clockUs(): microseconds since boot (wraps after ~71.6 minutes)
// Both wrap: always compare times with unsigned differences (now - then).
//
// --- Platform implementation (one per build) ---
#if defined(ESP32)
#include "platform/esp32/Esp32Clock.h"
#elif defined(UNIT_TEST)
#include "MockClock.h"  // test/unit/mocks
#else
#error "No clock implementation for this platform (add one in src/platform/<chip>/)"
#endif
