#pragma once
#include <cstddef>
#include <cstdio>
#include <cstring>
#include "core/eventService/EventType.h"
#include "config/ArmingConfig.h"

// ============================================
// WEBSOCKET RC PACKET PARSER (pure logic, no hardware)
// ============================================
// Packet format (text): "T:0.00,Y:0.00,P:0.00,R:0.00,A:0"   (A = arm switch, 0 or 1)
//
// Strict: the whole message must match. Trailing characters, a missing field or an
// arm value other than 0/1 reject the packet. Value ranges are checked separately
// (RcValidation.h), for every provider.

constexpr size_t WS_RC_MSG_MAX = 64;  // Longest accepted message (bytes, without terminator)

// Parses a message of `len` bytes (not null-terminated). Returns false and leaves
// `out` untouched if the message is not a valid packet.
inline bool parseWebSocketRcPacket(const char* data, size_t len, RcInputData& out) {
    if (!data || len == 0 || len > WS_RC_MSG_MAX) return false;

    // Copy into a bounded, terminated buffer: the received frame is not terminated,
    // and writing a terminator into it would write past its end.
    char msg[WS_RC_MSG_MAX + 1];
    std::memcpy(msg, data, len);
    msg[len] = '\0';

    RcInputData parsed = {0.0f, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}, false};
    int armSwitch = -1;
    int consumed = 0;
    const int fields = std::sscanf(msg, "T:%f,Y:%f,P:%f,R:%f,A:%d%n",
                                   &parsed.throttle, &parsed.yaw, &parsed.pitch, &parsed.roll,
                                   &armSwitch, &consumed);
    if (fields != 5 || consumed <= 0 || static_cast<size_t>(consumed) != len) return false;
    if (armSwitch != 0 && armSwitch != 1) return false;  // e.g. 256 would wrap to 0 in a uint8_t

    parsed.aux[ARM_AUX_CHANNEL] = static_cast<uint8_t>(armSwitch);
    parsed.linkOk = true;
    out = parsed;
    return true;
}
