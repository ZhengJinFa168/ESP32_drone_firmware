#pragma once
// ESP32 implementation of core/Log.h. Include core/Log.h, not this file.
#include <Arduino.h>

// 921600 leaves room for telemetry (~13 KB/s at 50 Hz); set Teleplot and the
// serial monitor (monitor_speed in platformio.ini) to the same rate.
constexpr unsigned long LOG_BAUD = 921600;
// Software TX buffer behind the 128-byte UART FIFO. Lets logTryWrite() queue a full
// telemetry frame without waiting; normal log lines block only when it is full.
constexpr size_t LOG_TX_BUFFER_SIZE = 1024;

inline void logInit() {
    Serial.setTxBufferSize(LOG_TX_BUFFER_SIZE);  // Must be called before begin()
    Serial.begin(LOG_BAUD);
}

inline void logWrite(LogLevel level, const char* tag, const char* fmt, ...) {
    char line[LOG_LINE_MAX];
    va_list args;
    va_start(args, fmt);
    const size_t len = logFormatLine(line, sizeof(line), level, tag, fmt, args);
    va_end(args);
    // Single write per line: the UART driver locks per call, so lines from
    // different tasks do not interleave.
    Serial.write(reinterpret_cast<const uint8_t*>(line), len);
}

inline bool logTryWrite(const char* data, size_t len) {
    if (!data || len == 0) return true;
    const int room = Serial.availableForWrite();
    if (room < 0 || static_cast<size_t>(room) < len) return false;  // Would block: drop
    Serial.write(reinterpret_cast<const uint8_t*>(data), len);
    return true;
}
