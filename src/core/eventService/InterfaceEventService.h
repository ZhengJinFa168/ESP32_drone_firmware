#pragma once
#include "EventType.h"
#include <functional>

using EventCallback = std::function<void(const void*)>;

// Abstract interface for the event service.
// Use this for dependency injection and test mocking.
class IEventService {
public:
    virtual ~IEventService() = default;
    virtual bool subscribe(EventType type, EventCallback cb) = 0;
    virtual void publish(EventType type, const void* data = nullptr) = 0;
    virtual void reset() = 0;
};
