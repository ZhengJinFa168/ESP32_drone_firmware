#ifndef MODULE_H
#define MODULE_H

#include "eventService/EventService.h"
#include "Log.h"

class Module {
protected:
    const char* name;
    bool enabled;

    // Convenience wrappers: automatically passes 'this' as context.
    void publish(EventType type, const void* data = nullptr) {
        EventService::instance().publish(type, data);
    }

    // Logs an error if the event's listener slots are full (CORE_MAX_LISTENERS_PER_EVENT):
    // a module that silently misses its events (e.g. arming) would be unsafe.
    bool subscribe(EventType type, EventCallback cb) {
        const bool ok = EventService::instance().subscribe(type, cb, this);
        if (!ok) {
            LOG_E("MODULE", "%s: subscribe failed for event %d (listener limit reached)",
                  name, static_cast<int>(type));
        }
        return ok;
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