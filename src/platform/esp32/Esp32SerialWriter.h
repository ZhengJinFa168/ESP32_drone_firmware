#pragma once
// ESP32 serial writer task: after logStartWriter() it is the only writer of the log
// UART. Include core/Log.h, not this file (it uses LOG_LINE_MAX from there).
#include <Arduino.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "platform/esp32/Esp32Task.h"

// Core 0, low priority: below WiFi (23), lwIP (18) and AsyncTCP (10), so serial output
// never delays RC input. Under WiFi load lines and frames are late, not lost (lines)
// or dropped and counted (telemetry).
constexpr int SERIAL_TASK_CORE = 0;
constexpr UBaseType_t SERIAL_TASK_PRIORITY = 2;
constexpr uint32_t SERIAL_TASK_STACK_BYTES = 4096;  // Float printf for telemetry; check the high-water mark
constexpr size_t SERIAL_TASK_MAX_WORKERS = 2;
// Log lines waiting for the UART. Full = the line is dropped and counted, never waited for.
constexpr size_t LOG_QUEUE_LINES = 32;

// SAFETY / timing: producers (the flight task on core 1, AsyncTCP, ...) only copy a
// line or a snapshot into a queue and wake this task; they never wait for the UART.
// Each wake-up the task writes every queued log line first (logs have priority), then
// runs its workers (telemetry formats and writes its frame here, not on core 1).
// Never call from an ISR.
class Esp32SerialWriter {
public:
    using Worker = void (*)(void* ctx);

    struct Line {
        uint16_t len;
        char text[LOG_LINE_MAX];
    };

    // Creates the line queue and starts the task (once). False: logging stays direct.
    bool start() {
        if (running()) return true;
        if (!_queue) _queue = xQueueCreateStatic(LOG_QUEUE_LINES, sizeof(Line), _storage, &_queueControl);
        if (!_queue) return false;
        if (!_task.start("serial", taskMain, this, SERIAL_TASK_PRIORITY, SERIAL_TASK_CORE)) return false;
        _running.store(true, std::memory_order_release);
        return true;
    }

    bool running() const { return _running.load(std::memory_order_acquire); }

    // Queues one line without blocking. Full queue: dropped, counted, reported later.
    void push(const Line& line) {
        if (xQueueSend(_queue, &line, 0) != pdTRUE) _droppedLines.fetch_add(1);
        wake();
    }

    // Registers work to run in the task on every wake-up (e.g. send the latest telemetry
    // frame). Call at boot; fixed capacity, no heap. False if full.
    bool addWorker(Worker fn, void* ctx) {
        const size_t n = _workerCount.load(std::memory_order_acquire);
        if (!fn || n >= SERIAL_TASK_MAX_WORKERS) return false;
        _workers[n] = {fn, ctx};
        _workerCount.store(n + 1, std::memory_order_release);  // Publish after the entry is written
        return true;
    }

    // Wakes the task (new data in a worker's mailbox)
    void wake() {
        TaskHandle_t task = _task.handle();
        if (task) xTaskNotifyGive(task);
    }

    uint32_t droppedLines() const { return _droppedLines.load(); }
    uint32_t stackFreeMinBytes() const { return _task.stackFreeMinBytes(); }

private:
    struct WorkerEntry {
        Worker fn;
        void* ctx;
    };

    [[noreturn]] static void taskMain(void* arg) {
        Esp32SerialWriter* self = static_cast<Esp32SerialWriter*>(arg);
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            self->drainLines();
            const size_t n = self->_workerCount.load(std::memory_order_acquire);
            for (size_t i = 0; i < n; i++) self->_workers[i].fn(self->_workers[i].ctx);
        }
    }

    void drainLines() {
        while (xQueueReceive(_queue, &_rxLine, 0) == pdTRUE) {
            Serial.write(reinterpret_cast<const uint8_t*>(_rxLine.text), _rxLine.len);
        }
        // Loud: lost lines are reported once there is room again
        const uint32_t dropped = _droppedLines.exchange(0);
        if (dropped > 0) {
            const int len = std::snprintf(_rxLine.text, sizeof(_rxLine.text),
                                          "[W][LOG] %lu lines dropped (log queue full)\n",
                                          static_cast<unsigned long>(dropped));
            if (len > 0) Serial.write(reinterpret_cast<const uint8_t*>(_rxLine.text), static_cast<size_t>(len));
        }
    }

    StaticQueue_t _queueControl;
    uint8_t _storage[LOG_QUEUE_LINES * sizeof(Line)];
    QueueHandle_t _queue = nullptr;
    Esp32PinnedTask<SERIAL_TASK_STACK_BYTES> _task;
    std::atomic<bool> _running{false};
    std::atomic<uint32_t> _droppedLines{0};
    WorkerEntry _workers[SERIAL_TASK_MAX_WORKERS] = {};
    std::atomic<size_t> _workerCount{0};
    Line _rxLine;  // Used only by the task (keeps it off the task stack)
};

// One writer per firmware (it owns the UART). Zero-initialized global, no heap.
inline Esp32SerialWriter g_serialWriter;
