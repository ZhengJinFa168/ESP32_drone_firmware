#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include "eventService/EventService.h"

class Module {
protected:
    const char* name;
    bool enabled;

    // Convenience wrappers: automatically passes 'this' as context.
    void publish(EventType type, const void* data = nullptr) {
        EventService::instance().publish(type, data);
    }

    bool subscribe(EventType type, EventCallback cb) {
        return EventService::instance().subscribe(type, cb, this);
    }

public:
    explicit Module(const char* n) : name(n), enabled(true) {}
    virtual ~Module() {}

    // Pure virtual – every module must implement setup
    virtual void setup() = 0;

    // Default empty loop — event-driven modules don't need to override this
    virtual void loop() {}

    // Utility methods
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() const { return enabled; }
    const char* getName() const { return name; }
};

#endif