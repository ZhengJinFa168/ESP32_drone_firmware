#pragma once
#include <cstdint>
#include "eventService/EventType.h"

// ============================================
// LOOP STATS (pure logic, no hardware)
// ============================================
// Measures the main loop over fixed windows: passes per second, longest pass,
// longest gap between pass starts and overruns (passes longer than the period).
// Fed by App::loop() with clock readings; all times use unsigned differences, so
// clock wrap-around is handled.
class LoopStats {
public:
    // periodUs: loop deadline (0 = no overrun check). windowMs: reporting window.
    LoopStats(uint32_t periodUs, uint32_t windowMs) : _periodUs(periodUs), _windowMs(windowMs) {}

    // Records one pass. Returns true when a window has completed: `out` then holds
    // that window's stats and a new window starts.
    bool record(uint32_t startUs, uint32_t endUs, uint32_t nowMs, LoopStatsData& out) {
        if (!_started) {
            _started = true;
            _windowStartMs = nowMs;
        } else {
            const uint32_t gapUs = startUs - _lastStartUs;
            if (gapUs > _maxGapUs) _maxGapUs = gapUs;
        }
        _lastStartUs = startUs;

        const uint32_t runUs = endUs - startUs;
        if (runUs > _maxRunUs) _maxRunUs = runUs;
        if (_periodUs > 0 && runUs > _periodUs) _overruns++;
        _passes++;

        const uint32_t elapsedMs = nowMs - _windowStartMs;
        if (_windowMs == 0 || elapsedMs < _windowMs) return false;

        out.rateHz = static_cast<uint32_t>((static_cast<uint64_t>(_passes) * 1000u + elapsedMs / 2) / elapsedMs);
        out.maxRunUs = _maxRunUs;
        out.maxGapUs = _maxGapUs;
        out.overruns = _overruns;

        _windowStartMs = nowMs;
        _passes = 0;
        _maxRunUs = 0;
        _maxGapUs = 0;   // The gap into the next window's first pass is still measured
        _overruns = 0;
        return true;
    }

private:
    uint32_t _periodUs;
    uint32_t _windowMs;
    bool _started = false;
    uint32_t _windowStartMs = 0;
    uint32_t _lastStartUs = 0;
    uint32_t _passes = 0;
    uint32_t _maxRunUs = 0;
    uint32_t _maxGapUs = 0;
    uint32_t _overruns = 0;
};
