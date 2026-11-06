#pragma once
#include "Module.h"
#include "CoreConfig.h"
#include "Log.h"
#include "Clock.h"
#include "LoopStats.h"
#include "eventService/EventService.h"

class App {
public:
    static const int MAX_MODULES = CoreConfig::MAX_MODULES;  // Public for tests

private:
    Module* modules[MAX_MODULES];
    int moduleCount;
    LoopStats _loopStats;

public:
    // loopPeriodUs: deadline for one pass (0 = no overrun check), see FLIGHT_LOOP_PERIOD_US
    explicit App(uint32_t loopPeriodUs = 0)
        : moduleCount(0), _loopStats(loopPeriodUs, CoreConfig::LOOP_STATS_WINDOW_MS) {}

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
    // Every pass is timed; once per LOOP_STATS_WINDOW_MS the stats are published as
    // LOOP_STATS (from the loop's own task, like every other event).
    void loop() {
        const uint32_t startUs = clockUs();
        for (int i = 0; i < moduleCount; i++) {
            if (modules[i]->isEnabled()) {
                modules[i]->loop();
            }
        }
        const uint32_t endUs = clockUs();

        LoopStatsData stats;
        if (_loopStats.record(startUs, endUs, clockMs(), stats)) {
            // Loud but bounded: at most one line per window
            if (stats.overruns > 0) {
                LOG_W("APP", "loop overruns: %lu passes missed the deadline (longest %lu us)",
                      static_cast<unsigned long>(stats.overruns), static_cast<unsigned long>(stats.maxRunUs));
            }
            EventService::instance().publish(EventType::LOOP_STATS, &stats);
        }
    }

    // --- Test helpers ---
    int getModuleCount() const { return moduleCount; }
    Module* getModule(int index) const {
        return (index >= 0 && index < moduleCount) ? modules[index] : nullptr;
    }
};
