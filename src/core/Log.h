#pragma once
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>

// ============================================
// LOGGING
// ============================================
// One logging API for every module: LOG_E / LOG_W / LOG_I / LOG_D(tag, printf-format, ...).
// Output line: "[E][TAG] message\n".
//
// - Each line is formatted completely on the caller's stack and handed over whole, so
//   lines from different tasks (flight task, async_tcp) never interleave.
// - No heap: a fixed-size stack buffer; longer messages are truncated, never overflow.
// - The level filter is compile-time (FW_LOG_LEVEL): filtered calls cost nothing and
//   their arguments are not evaluated.
// - The implementation is chosen at compile time (platform layer or test mock), so
//   logic modules can log without depending on the chip.
// - After logStartWriter() logging never blocks: the line is queued for a writer task
//   (ESP32: core 0) and dropped and counted if the queue is full. Before it, lines are
//   written directly and may block. Either way: state changes and errors only, never
//   per sample. High-rate output (telemetry) uses logTryWrite() from the writer task.

enum class LogLevel : uint8_t {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3,
};

// Highest level that is compiled in (0 = errors only ... 3 = everything)
#ifndef FW_LOG_LEVEL
#define FW_LOG_LEVEL 2
#endif

constexpr size_t LOG_LINE_MAX = 160;  // Bytes per line including "\n" and the terminator

inline char logLevelChar(LogLevel level) {
    switch (level) {
        case LogLevel::ERROR: return 'E';
        case LogLevel::WARN:  return 'W';
        case LogLevel::INFO:  return 'I';
        case LogLevel::DEBUG: return 'D';
        default:              return '?';
    }
}

// Format one complete line into buf: "[L][TAG] message\n".
// Always newline- and null-terminated, even when truncated. Returns the length
// without the terminator (0 only if cap is too small to hold anything useful).
inline size_t logFormatLine(char* buf, size_t cap, LogLevel level, const char* tag,
                            const char* fmt, va_list args) {
    if (!buf || cap < 2) return 0;

    int used = std::snprintf(buf, cap, "[%c][%s] ", logLevelChar(level), tag ? tag : "?");
    if (used < 0) used = 0;
    size_t len = static_cast<size_t>(used) < cap ? static_cast<size_t>(used) : cap - 1;

    if (fmt && len < cap - 1) {
        const int body = std::vsnprintf(buf + len, cap - len, fmt, args);
        if (body > 0) {
            len += static_cast<size_t>(body);
            if (len > cap - 1) len = cap - 1;  // Truncated
        }
    }

    // Reserve room for the newline (overwrite the last character if full)
    if (len > cap - 2) len = cap - 2;
    buf[len++] = '\n';
    buf[len] = '\0';
    return len;
}

// --- Platform implementation (one per build) ---
// logInit():        prepare the output (call once at boot, before anything logs)
// logStartWriter(): start the writer task; from then on logWrite() never blocks.
//                   False (logged) = it could not start, logging stays direct
// logWrite():       format one line and queue it (or write it directly before the
//                   writer runs)
// logTryWrite():    write raw bytes only if they fit in the output buffer right now;
//                   returns false (nothing written) instead of blocking
inline void logInit();
inline bool logStartWriter();
inline void logWrite(LogLevel level, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
inline bool logTryWrite(const char* data, size_t len);

#if defined(ESP32)
#include "platform/esp32/Esp32Log.h"
#elif defined(UNIT_TEST)
#include "MockLog.h"  // test/unit/mocks
#else
#error "No log implementation for this platform (add one in src/platform/<chip>/)"
#endif

#define FW_LOG_AT(level, tag, ...)                                       \
    do {                                                                 \
        if (static_cast<int>(level) <= FW_LOG_LEVEL) {                   \
            logWrite(level, tag, __VA_ARGS__);                           \
        }                                                                \
    } while (0)

#define LOG_E(tag, ...) FW_LOG_AT(LogLevel::ERROR, tag, __VA_ARGS__)
#define LOG_W(tag, ...) FW_LOG_AT(LogLevel::WARN, tag, __VA_ARGS__)
#define LOG_I(tag, ...) FW_LOG_AT(LogLevel::INFO, tag, __VA_ARGS__)
#define LOG_D(tag, ...) FW_LOG_AT(LogLevel::DEBUG, tag, __VA_ARGS__)
