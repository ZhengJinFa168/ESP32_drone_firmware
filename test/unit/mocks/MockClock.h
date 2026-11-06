#pragma once
// Native test implementation of core/Clock.h: time is whatever the test sets.
// Include core/Clock.h, not this file (the Arduino.h mock includes it too).
#include <cstdint>

inline unsigned long g_mockMillis = 0;   // Returned by clockMs() and the mock millis()
inline unsigned long g_mockMicros = 0;   // Returned by clockUs() and the mock micros()

inline uint32_t clockMs() { return static_cast<uint32_t>(g_mockMillis); }
inline uint32_t clockUs() { return static_cast<uint32_t>(g_mockMicros); }
