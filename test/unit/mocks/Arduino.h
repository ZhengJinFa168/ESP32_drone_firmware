// test/unit/mocks/Arduino.h
#ifndef ARDUINO_H
#define ARDUINO_H

#include "WString.h"
#include "MockClock.h"

#define LOW 0
#define HIGH 1
#define OUTPUT 0
#define INPUT 0
#define LED_BUILTIN 2

// Controllable clock for tests (shared with core/Clock.h): set g_mockMillis/g_mockMicros
inline unsigned long millis() { return g_mockMillis; }
inline unsigned long micros() { return g_mockMicros; }
inline void delay(unsigned long) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }

class MockSerial {
public:
    void begin(int) {}
    void println(const char*) {}
    void println(const String& s) { println(s.c_str()); }
    void print(const char*) {}
    void print(const String& s) { print(s.c_str()); }
};

inline MockSerial Serial;   // <-- C++17 inline variable: safe to define in a header

#endif // ARDUINO_H