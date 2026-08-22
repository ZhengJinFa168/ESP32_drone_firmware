#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>

class Module {
protected:
    String name;
    bool enabled;

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