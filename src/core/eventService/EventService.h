#pragma once
#include "EventType.h"
#include "../CoreConfig.h"
#include <array>
#include <cstdint>

// Zero-allocation, lightweight C-style callback with context pointer
using EventCallback = void (*)(void* context, const void* data);

struct EventListener {
    EventCallback callback = nullptr;
    void* context = nullptr;
};

class EventService {
public:
    static constexpr int MAX_LISTENERS_PER_EVENT = CoreConfig::MAX_LISTENERS_PER_EVENT;

    static EventService& instance() {
        static EventService svc;
        return svc;
    }

    // Subscribe a callback and context to an event type.
    // Returns true on success, false if the listener slot is full.
    bool subscribe(EventType type, EventCallback cb, void* context = nullptr) {
        auto idx = static_cast<uint8_t>(type);
        if (idx >= static_cast<uint8_t>(EventType::_COUNT)) return false;
        auto& slot = _listeners[idx];
        if (slot.count >= MAX_LISTENERS_PER_EVENT) return false;
        slot.listeners[slot.count].callback = cb;
        slot.listeners[slot.count].context = context;
        slot.count++;
        return true;
    }

    // Publish an event — all subscribers are called synchronously
    // in registration order. Call from loop context only.
    void publish(EventType type, const void* data = nullptr) {
        auto idx = static_cast<uint8_t>(type);
        if (idx >= static_cast<uint8_t>(EventType::_COUNT)) return;
        auto& slot = _listeners[idx];
        for (int i = 0; i < slot.count; i++) {
            if (slot.listeners[i].callback) {
                slot.listeners[i].callback(slot.listeners[i].context, data);
            }
        }
    }

    // Reset all subscriptions. Essential for test isolation.
    void reset() {
        for (auto& slot : _listeners) {
            slot.count = 0;
            for (int i = 0; i < MAX_LISTENERS_PER_EVENT; i++) {
                slot.listeners[i].callback = nullptr;
                slot.listeners[i].context = nullptr;
            }
        }
    }

private:
    struct ListenerSlot {
        EventListener listeners[MAX_LISTENERS_PER_EVENT];
        int count = 0;
    };

    std::array<ListenerSlot, static_cast<size_t>(EventType::_COUNT)> _listeners{};

    EventService() = default;
    EventService(const EventService&) = delete;
    EventService& operator=(const EventService&) = delete;
};
