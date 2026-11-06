#pragma once
// ESP32 (FreeRTOS) task with a statically allocated stack and control block.
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// One task pinned to one core. No heap: the stack and the TCB are members, so the
// object must live for the whole program (a global or static).
// StackBytes: on ESP-IDF the stack depth is in bytes (StackType_t is uint8_t).
template <uint32_t StackBytes>
class Esp32PinnedTask {
public:
    // Creates and starts the task. A task with a higher priority than the caller on
    // the same core runs immediately, before this returns.
    bool start(const char* name, TaskFunction_t fn, void* arg, UBaseType_t priority, BaseType_t core) {
        if (_handle) return false;  // Started once only (the stack is in use)
        _handle = xTaskCreateStaticPinnedToCore(fn, name, StackBytes, arg, priority, _stack, &_tcb, core);
        return _handle != nullptr;
    }

    TaskHandle_t handle() const { return _handle; }

    // Smallest free stack seen so far (bytes): check it has margin after a long run.
    uint32_t stackFreeMinBytes() const { return _handle ? uxTaskGetStackHighWaterMark(_handle) : 0; }

private:
    StackType_t _stack[StackBytes / sizeof(StackType_t)];
    StaticTask_t _tcb;
    TaskHandle_t _handle = nullptr;
};
