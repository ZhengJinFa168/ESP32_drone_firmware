#pragma once

// Single-precision conversion constants. Deliberately NOT named RAD_TO_DEG /
// DEG_TO_RAD: Arduino.h defines those as double literals, which would silently turn
// every conversion into software-emulated double math on the ESP32 (its FPU is
// single-precision only) and make the firmware compile different code than the
// native tests.
constexpr float RAD2DEG = 57.29577951308232f;
constexpr float DEG2RAD = 0.017453292519943295f;
