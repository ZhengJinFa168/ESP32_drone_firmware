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

    // Returns false (and logs an error) if the module list is full or module is null.
    // SAFETY: a module that silently fails to register (e.g. arming) would never run;
    // the caller must treat false as fatal.
    bool registerModule(Module* module) {
        if (!module) {
            LOG_E("APP", "registerModule: null module");
            return false;
        }
        if (moduleCount >= MAX_MODULES) {
            LOG_E("APP", "registerModule: list full (MAX_MODULES = %d), '%s' not registered",
                  MAX_MODULES, module->getName());
            return false;
        }
        modules[moduleCount++] = module;
        return true;
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

    // Runs every enabled module once. Never blocks: the caller decides how to yield.
    void loop() {
        for (int i = 0; i < moduleCount; i++) {
            if (modules[i]->isEnabled()) {
                modules[i]->loop();
            }
        }
    }

    // --- Test helpers ---
    int getModuleCount() const { return moduleCount; }
    Module* getModule(int index) const {
        return (index >= 0 && index < moduleCount) ? modules[index] : nullptr;
    }
};
