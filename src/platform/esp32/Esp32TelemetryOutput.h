#pragma once
// ESP32 telemetry output: formats and writes frames on the core-0 serial writer task,
// so the ~1.2 ms of float formatting is off the flight task (core 1).
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "core/Log.h"
#include "modules/logic/telemetry/TelemetrySender.h"

// TOutput for TelemetryModule. send() runs in the flight task, the formatting in the
// serial writer task. Handover: a one-slot mailbox (newest snapshot wins).
class Esp32TelemetryOutput {
public:
    // From TelemetryModule::setup(). Needs logStartWriter() first (main.cpp).
    void begin() {
        if (_queue) return;
        _queue = xQueueCreateStatic(1, sizeof(TelemetrySnapshot), _storage, &_queueControl);
        if (!_queue) {
            LOG_E("TELEMETRY", "mailbox could not be created, telemetry disabled");
            return;
        }
        if (!g_serialWriter.running() || !g_serialWriter.addWorker(onWork, this)) {
            LOG_E("TELEMETRY", "serial writer not running or full, telemetry disabled");
            _queue = nullptr;
        }
    }

    // Flight task. Never blocks: copies the snapshot (~120 B) and wakes the writer.
    void send(const TelemetrySnapshot& snapshot) {
        if (!_queue) return;
        // The previous frame was not sent yet (writer behind, e.g. WiFi load): it is
        // replaced, so count it as dropped (>tlmDrop)
        if (uxQueueMessagesWaiting(_queue) > 0) _sender.addDropped(1);
        xQueueOverwrite(_queue, &snapshot);
        g_serialWriter.wake();
    }

    uint32_t droppedFrames() const { return _sender.droppedFrames(); }
    uint32_t oversizeFrames() const { return _sender.oversizeFrames(); }

private:
    // Serial writer task (core 0): the only caller of _sender.send()
    static void onWork(void* ctx) {
        Esp32TelemetryOutput* self = static_cast<Esp32TelemetryOutput*>(ctx);
        if (xQueueReceive(self->_queue, &self->_rx, 0) == pdTRUE) self->_sender.send(self->_rx);
    }

    StaticQueue_t _queueControl;
    uint8_t _storage[sizeof(TelemetrySnapshot)];
    QueueHandle_t _queue = nullptr;
    TelemetrySnapshot _rx;  // Writer task only
    TelemetrySender _sender;
};
