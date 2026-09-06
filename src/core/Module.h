#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include "eventService/EventService.h"

class Module {
protected:
    String name;
    bool enabled;

    // Convenience methods for event communication
    void publish(EventType type, const void* data = nullptr) {
        EventService::instance().publish(type, data);
    }

    bool subscribe(EventType type, EventCallback cb) {
        return EventService::instance().subscribe(type, cb);
    }

public:
    Module(const String& n) : name(n), enabled(true) {}
    virtual ~Module() {}

    // Pure virtual – every module must implement these
    virtual void setup() = 0;
    virtual void loop() = 0;

    // Utility methods
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() const { return enabled; }
    String getName() const { return name; }
};

#endif