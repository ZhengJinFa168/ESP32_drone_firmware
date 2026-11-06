#pragma once
// Native test implementation of core/Log.h: records log lines so tests can assert
// that errors and warnings were reported. Include core/Log.h, not this file.
#include <cstring>

struct MockLog {
    static constexpr int MAX_LINES = 32;
    static constexpr size_t TX_CAPTURE_SIZE = 4096;

    char lines[MAX_LINES][LOG_LINE_MAX] = {};
    LogLevel levels[MAX_LINES] = {};
    int lineCount = 0;      // Lines stored (oldest kept, extra lines only counted)
    int total[4] = {};      // Lines per level, including the ones not stored

    // logTryWrite(): bytes the "output buffer" accepts right now, and what was written
    size_t txFree = TX_CAPTURE_SIZE;
    char tx[TX_CAPTURE_SIZE] = {};
    size_t txLen = 0;
    int txDropped = 0;

    void reset() { *this = MockLog{}; }

    int count(LogLevel level) const { return total[static_cast<int>(level)]; }

    // True if any stored line contains `text`
    bool contains(const char* text) const {
        for (int i = 0; i < lineCount; i++) {
            if (std::strstr(lines[i], text)) return true;
        }
        return false;
    }
};

inline MockLog g_mockLog;

inline void logInit() {}
inline bool logStartWriter() { return true; }

inline void logWrite(LogLevel level, const char* tag, const char* fmt, ...) {
    g_mockLog.total[static_cast<int>(level)]++;
    if (g_mockLog.lineCount >= MockLog::MAX_LINES) return;
    va_list args;
    va_start(args, fmt);
    logFormatLine(g_mockLog.lines[g_mockLog.lineCount], LOG_LINE_MAX, level, tag, fmt, args);
    va_end(args);
    g_mockLog.levels[g_mockLog.lineCount] = level;
    g_mockLog.lineCount++;
}

inline bool logTryWrite(const char* data, size_t len) {
    if (!data || len == 0) return true;
    if (len > g_mockLog.txFree || g_mockLog.txLen + len > MockLog::TX_CAPTURE_SIZE) {
        g_mockLog.txDropped++;
        return false;
    }
    std::memcpy(g_mockLog.tx + g_mockLog.txLen, data, len);
    g_mockLog.txLen += len;
    g_mockLog.txFree -= len;
    return true;
}
