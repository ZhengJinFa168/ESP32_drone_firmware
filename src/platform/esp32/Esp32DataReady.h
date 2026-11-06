#pragma once
// ESP32 data-ready interrupt: a rising edge on a sensor's INT pin wakes a task.
#include <Arduino.h>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// The ISR only sets a flag and notifies the task (no I2C, no logging, no events):
// the sensor is read in the task, by the driver, after consume() returns true.
// - Edges that arrive before setTask() (or with no task) only set the flag, so
//   drivers can also be polled (hardware tests without a flight task).
// - The interrupt is allocated on the core that calls begin(): call it from the core
//   the woken task runs on (core 1), so the notification does not cross cores.
// - Several edges before one consume() count as one: the driver reads the latest
//   sample (the sensor keeps only that one anyway).
class Esp32DataReady {
public:
    Esp32DataReady() = default;
    Esp32DataReady(const Esp32DataReady&) = delete;
    Esp32DataReady& operator=(const Esp32DataReady&) = delete;

    // The ISR holds a pointer to this object: detach before it goes away
    // (only happens in hardware tests; firmware drivers are static).
    ~Esp32DataReady() { end(); }

    // Rising edge on pin. The pin mode is not changed (the driver owns it: no pull-up).
    void begin(uint8_t pin) {
        end();
        _pin = pin;
        attachInterruptArg(pin, onEdge, this, RISING);
        _attached = true;
    }

    void end() {
        if (!_attached) return;
        detachInterrupt(_pin);
        _attached = false;
    }

    // Task to wake on every edge (nullptr: flag only)
    void setTask(TaskHandle_t task) {
        portENTER_CRITICAL(&_mux);
        _task = task;
        portEXIT_CRITICAL(&_mux);
    }

    // True if at least one edge arrived since the last call (clears it)
    bool consume() {
        portENTER_CRITICAL(&_mux);
        const bool pending = _pending;
        _pending = false;
        portEXIT_CRITICAL(&_mux);
        return pending;
    }

    // Edges since boot (diagnostics and tests)
    uint32_t count() const { return _count; }

private:
    static void IRAM_ATTR onEdge(void* arg) {
        Esp32DataReady* self = static_cast<Esp32DataReady*>(arg);
        portENTER_CRITICAL_ISR(&self->_mux);
        self->_pending = true;
        self->_count = self->_count + 1;
        TaskHandle_t task = self->_task;
        portEXIT_CRITICAL_ISR(&self->_mux);

        if (task) {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(task, &woken);
            if (woken) portYIELD_FROM_ISR();  // Run the flight task right after the ISR
        }
    }

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    volatile bool _pending = false;
    volatile uint32_t _count = 0;
    TaskHandle_t _task = nullptr;
    uint8_t _pin = 0;
    bool _attached = false;
};
