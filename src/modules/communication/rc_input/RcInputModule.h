#pragma once
#include "core/Module.h"
#include "core/Log.h"
#include "RcValidation.h"

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


// Reads packets from the compile-time selected provider, validates them and
// publishes RC_INPUT. Publishes a failsafe packet (linkOk = false) when no VALID
// packet arrived for FAILSAFE_TIMEOUT_MS.
class RcInputModule : public Module {
private:
    CurrentRcProvider provider;
    unsigned long _lastDataTime = 0;
    const unsigned long FAILSAFE_TIMEOUT_MS = 500; // 0.5s without a valid packet triggers failsafe
    bool _inFailsafe = false;

    // Rejected packets, reported at most once per REJECT_LOG_INTERVAL_MS
    static constexpr unsigned long REJECT_LOG_INTERVAL_MS = 1000;
    uint32_t _outOfRangePackets = 0;
    uint32_t _reportedOutOfRange = 0;
    uint32_t _reportedMalformed = 0;
    unsigned long _lastRejectLogMs = 0;

    void reportRejectedPackets(unsigned long now) {
        if (now - _lastRejectLogMs < REJECT_LOG_INTERVAL_MS) return;
        const uint32_t malformed = provider.malformedPackets();
        if (malformed == _reportedMalformed && _outOfRangePackets == _reportedOutOfRange) return;

        _lastRejectLogMs = now;
        LOG_E("RC", "rejected packets: %lu malformed, %lu out of range (since last report)",
              static_cast<unsigned long>(malformed - _reportedMalformed),
              static_cast<unsigned long>(_outOfRangePackets - _reportedOutOfRange));
        _reportedMalformed = malformed;
        _reportedOutOfRange = _outOfRangePackets;
    }

public:
    RcInputModule() : Module("RcInput") {}

    void setup() override {
        provider.setup();
    }

    void loop() override {
        if (!isEnabled()) return;

        const unsigned long now = millis();
        RcInputData rcData;

        if (provider.getRcData(rcData)) {
            // SAFETY: never publish out-of-range values (and never clamp them).
            // A rejected packet does not refresh the link, so garbage ends in failsafe.
            if (isValidRcInput(rcData)) {
                _lastDataTime = now;
                if (_inFailsafe) {
                    LOG_I("RC", "link recovered");
                    _inFailsafe = false;
                }
                publish(EventType::RC_INPUT, &rcData);
            } else {
                _outOfRangePackets++;
            }
        }

        // Checked on every loop, not only when nothing arrives: a stream of rejected
        // packets must trigger failsafe too.
        if (!_inFailsafe && (now - _lastDataTime > FAILSAFE_TIMEOUT_MS)) {
            _inFailsafe = true;
            LOG_E("RC", "SIGNAL LOST, failsafe packet published");

            // Publish failsafe state: zero throttle, neutral sticks, linkOk = false so
            // consumers (arming, flight logic) can tell it apart from real pilot input
            RcInputData failsafeData = {0.0f, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}, false};
            publish(EventType::RC_INPUT, &failsafeData);
        }

        reportRejectedPackets(now);
    }
};
