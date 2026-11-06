#pragma once
// ESP32 implementation of core/Clock.h. Include core/Clock.h, not this file.
#include <cstdint>
#include <esp_timer.h>

// esp_timer is the 64-bit microsecond timer Arduino's millis()/micros() also use
inline uint32_t clockMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
inline uint32_t clockUs() { return static_cast<uint32_t>(esp_timer_get_time()); }
