#pragma once
#include <cstddef>
#include <cstdint>
#include <atomic>
#include "core/Log.h"
#include "config/TelemetryConfig.h"
#include "TelemetryFormatter.h"

// ============================================
// TELEMETRY SENDER (no events, no module)
// ============================================
// Formats one snapshot and writes it with logTryWrite(). SAFETY: never blocks: a frame
// that does not fit in the serial TX buffer is dropped and counted (>tlmDrop).
//
// Kept separate from TelemetryModule (which collects values from events in the flight
// task): on the board it runs in the core-0 serial writer task, fed through a one-slot
// mailbox (platform/esp32/Esp32TelemetryOutput.h). In native tests it is the module's
// output directly. Call send() from one task only; the counters are atomic, so they
// can be read (and addDropped() called) from another.
class TelemetrySender {
public:
    void begin() {}  // TOutput interface (TelemetryModule): nothing to start
    void send(const TelemetrySnapshot& snapshot) {
        TelemetrySnapshot s = snapshot;
        s.droppedFrames = _droppedFrames.load();

        size_t len = 0;
        if (!formatTelemetry(_frame, sizeof(_frame), s, TELEMETRY_GROUPS, len)) {
            // Configuration error (too many values for TELEMETRY_FRAME_MAX): loud, once
            if (_oversizeFrames++ == 0) {
                LOG_E("TELEMETRY", "frame larger than TELEMETRY_FRAME_MAX (%u B), telemetry not sent",
                      static_cast<unsigned>(TELEMETRY_FRAME_MAX));
            }
            return;
        }
        if (len == 0) return;  // Nothing received yet
        if (!logTryWrite(_frame, len)) _droppedFrames++;
    }

    // Frames lost before reaching send() (e.g. replaced in a mailbox): also in >tlmDrop
    void addDropped(uint32_t frames) { _droppedFrames.fetch_add(frames); }

    uint32_t droppedFrames() const { return _droppedFrames.load(); }
    uint32_t oversizeFrames() const { return _oversizeFrames.load(); }

private:
    char _frame[TELEMETRY_FRAME_MAX];
    std::atomic<uint32_t> _droppedFrames{0};
    std::atomic<uint32_t> _oversizeFrames{0};
};
