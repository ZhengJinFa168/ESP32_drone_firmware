#ifndef APP_H
#define APP_H

#include "Module.h"

class App {
public:
    static const int MAX_MODULES = 15;  // Public for tests

private:
    Module* modules[MAX_MODULES];
    int moduleCount;

public:
    App() : moduleCount(0) {}

    void registerModule(Module* module) {
        if (moduleCount < MAX_MODULES) {
            modules[moduleCount++] = module;
        }
    }

    void setup() {
        Serial.begin(115200);
        Serial.println("Initializing modules...");
        for (int i = 0; i < moduleCount; i++) {
            Serial.print("  ✓ ");
            Serial.println(modules[i]->getName());
            modules[i]->setup();
        }
        Serial.println("All modules ready.");
    }

    void loop() {
        for (int i = 0; i < moduleCount; i++) {
            if (modules[i]->isEnabled()) {
                modules[i]->loop();
            }
        }
        delay(1);
    }

    // --- Test helpers ---
    int getModuleCount() const { return moduleCount; }
    Module* getModule(int index) const {
        return (index >= 0 && index < moduleCount) ? modules[index] : nullptr;
    }
};

#endif