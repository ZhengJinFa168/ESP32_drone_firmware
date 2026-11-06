#pragma once
// ESP32 implementation of core/Log.h. Include core/Log.h, not this file.
#include <Arduino.h>
#include "platform/esp32/Esp32SerialWriter.h"

// 921600 leaves room for telemetry (~13 KB/s at 50 Hz); set Teleplot and the
// serial monitor (monitor_speed in platformio.ini) to the same rate.
constexpr unsigned long LOG_BAUD = 921600;
// Software TX buffer behind the 128-byte UART FIFO. Lets logTryWrite() queue a full
// telemetry frame without waiting.
constexpr size_t LOG_TX_BUFFER_SIZE = 1024;

inline void logInit() {
    Serial.setTxBufferSize(LOG_TX_BUFFER_SIZE);  // Must be called before begin()
    Serial.begin(LOG_BAUD);
}

inline bool logStartWriter() {
    if (g_serialWriter.start()) return true;
    // Loud, but not fatal: direct logging still works (it blocks when the buffer is full)
    logWrite(LogLevel::ERROR, "LOG", "serial writer task could not start, logging stays direct (blocking)");
    return false;
}

inline void logWrite(LogLevel level, const char* tag, const char* fmt, ...) {
    Esp32SerialWriter::Line line;
    va_list args;
    va_start(args, fmt);
    line.len = static_cast<uint16_t>(logFormatLine(line.text, sizeof(line.text), level, tag, fmt, args));
    va_end(args);
    if (g_serialWriter.running()) {
        g_serialWriter.push(line);  // Never blocks; the core-0 task writes it
    } else {
        // Before logStartWriter() (boot, hardware tests): single write per line, the
        // UART driver locks per call, so lines from different tasks do not interleave.
        Serial.write(reinterpret_cast<const uint8_t*>(line.text), line.len);
    }
}

// Call only from the serial writer task (its workers) once it runs: it is the only
// UART writer, so the room checked here is still there for the write.
inline bool logTryWrite(const char* data, size_t len) {
    if (!data || len == 0) return true;
    const int room = Serial.availableForWrite();
    if (room < 0 || static_cast<size_t>(room) < len) return false;  // Would block: drop
    Serial.write(reinterpret_cast<const uint8_t*>(data), len);
    return true;
}
