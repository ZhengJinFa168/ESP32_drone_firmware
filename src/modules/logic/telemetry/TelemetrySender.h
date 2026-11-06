#pragma once
#include <cstddef>
#include <cstdint>
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
// task) so it can be moved to another task, e.g. on core 0: hand it snapshot copies
// through a one-slot mailbox (xQueueOverwrite, like WebSocketRcProvider) and call
// send() only from that task. send() touches only its own members and logTryWrite()
// (the UART driver locks per write). Never call it from two tasks.
class TelemetrySender {
public:
    void send(const TelemetrySnapshot& snapshot) {
        TelemetrySnapshot s = snapshot;
        s.droppedFrames = _droppedFrames;

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

    uint32_t droppedFrames() const { return _droppedFrames; }
    uint32_t oversizeFrames() const { return _oversizeFrames; }

private:
    char _frame[TELEMETRY_FRAME_MAX];
    uint32_t _droppedFrames = 0;
    uint32_t _oversizeFrames = 0;
};
