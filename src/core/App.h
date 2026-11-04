#pragma once
#include "Module.h"
#include "CoreConfig.h"
#include "Log.h"

class App {
public:
    static const int MAX_MODULES = CoreConfig::MAX_MODULES;  // Public for tests

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

    // Logging must be initialized (logInit()) before this is called.
    void setup() {
        LOG_I("APP", "Initializing modules...");
        for (int i = 0; i < moduleCount; i++) {
            LOG_I("APP", "  setup: %s", modules[i]->getName());
            modules[i]->setup();
        }
        LOG_I("APP", "All modules ready.");
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
