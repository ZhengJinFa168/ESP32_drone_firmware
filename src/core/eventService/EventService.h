#pragma once
#include "InterfaceEventService.h"
#include "../CoreConfig.h"
#include <array>
#include <cstdint>

class EventService : public IEventService {
public:
    static constexpr int MAX_LISTENERS_PER_EVENT = CoreConfig::MAX_LISTENERS_PER_EVENT;

    static EventService& instance() {
        static EventService svc;
        return svc;
    }

    // Subscribe a callback to an event type.
    // Returns true on success, false if the listener slot is full.
    bool subscribe(EventType type, EventCallback cb) override {
        auto idx = static_cast<uint8_t>(type);
        if (idx >= static_cast<uint8_t>(EventType::_COUNT)) return false;
        auto& slot = _listeners[idx];
        if (slot.count >= MAX_LISTENERS_PER_EVENT) return false;
        slot.callbacks[slot.count++] = cb;
        return true;
    }

    // Publish an event — all subscribers are called synchronously
    // in registration order. Call from loop context only.
    void publish(EventType type, const void* data = nullptr) override {
        auto idx = static_cast<uint8_t>(type);
        if (idx >= static_cast<uint8_t>(EventType::_COUNT)) return;
        auto& slot = _listeners[idx];
        for (int i = 0; i < slot.count; i++) {
            if (slot.callbacks[i]) {
                slot.callbacks[i](data);
            }
        }
    }

    // Reset all subscriptions. Essential for test isolation.
    void reset() override {
        for (auto& slot : _listeners) {
            slot.count = 0;
            for (int i = 0; i < MAX_LISTENERS_PER_EVENT; i++) {
                slot.callbacks[i] = nullptr;
            }
        }
    }

private:
    struct ListenerSlot {
        EventCallback callbacks[MAX_LISTENERS_PER_EVENT];
        int count = 0;
    };

    std::array<ListenerSlot, static_cast<size_t>(EventType::_COUNT)> _listeners{};

    EventService() = default;
    EventService(const EventService&) = delete;
    EventService& operator=(const EventService&) = delete;
};
