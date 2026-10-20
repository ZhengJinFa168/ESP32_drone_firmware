#pragma once
#include "core/Module.h"

// --- CONFIGURATION ---
// Swap these defines to change the RC input method at compile time
#define USE_WEBSOCKET_RC
//#define USE_CRSF_RC
// ---------------------

#ifdef USE_WEBSOCKET_RC
    #include "WebSocketRcProvider.h"
    using CurrentRcProvider = WebSocketRcProvider;
#elif defined(USE_CRSF_RC)
    // #include "CrsfRcProvider.h" (To be implemented later)
    // using CurrentRcProvider = CrsfRcProvider;
#endif


class RcInputModule : public Module {
private:
    CurrentRcProvider provider;
    unsigned long _lastDataTime = 0;
    const unsigned long FAILSAFE_TIMEOUT_MS = 500; // 0.5s without packet triggers failsafe
    bool _inFailsafe = false;

public:
    RcInputModule() : Module("RcInput") {}

    void setup() override {
        provider.setup();
    }

    void loop() override {
        if (!isEnabled()) return;

        RcInputData rcData;
        
        if (provider.getRcData(rcData)) {
            _lastDataTime = millis();
            
            if (_inFailsafe) {
                Serial.println("RC Link Recovered!");
                _inFailsafe = false;
            }
            
            publish(EventType::RC_INPUT, &rcData);
        } else {
            // Check for failsafe
            if (!_inFailsafe && (millis() - _lastDataTime > FAILSAFE_TIMEOUT_MS)) {
                _inFailsafe = true;
                Serial.println("RC SIGNAL LOST! Failsafe triggered.");
                
                // Publish failsafe state: zero throttle, neutral sticks
                RcInputData failsafeData = {0.0f, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}};
                publish(EventType::RC_INPUT, &failsafeData);
            }
        }
    }
};

