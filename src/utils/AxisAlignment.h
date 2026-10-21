#pragma once
#include <cstdint>

// ============================================
// SENSOR → BODY AXIS ALIGNMENT
// ============================================
// Body frame is FLU: X forward, Y left, Z up (see .agents/rules/architecture.md).
// A sensor mounted square to the frame differs from the body only by a
// signed axis swap. Each body axis i is taken from one sensor axis:
//
//     body[i] = sign[i] * sensor[src[i]]
//
// The same mapping applies to every vector the sensor measures (accel, gyro).
// Only proper rotations (24 of the 48 signed permutations) are physically
// possible; the others are mirror images and indicate a config mistake.

struct AxisAlignment {
    uint8_t src[3];   // Sensor axis feeding body X, Y, Z (0 = x, 1 = y, 2 = z)
    int8_t sign[3];   // +1 or -1 for each body axis
};

// True if the alignment is a real rotation: src is a permutation of {0, 1, 2},
// every sign is ±1, and the determinant is +1 (not a mirror image).
constexpr bool isValidRotation(const AxisAlignment& a) {
    bool seen[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        if (a.src[i] > 2 || seen[a.src[i]]) return false;
        if (a.sign[i] != 1 && a.sign[i] != -1) return false;
        seen[a.src[i]] = true;
    }

    // Determinant of a signed permutation matrix = permutation parity × product of signs
    int inversions = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (a.src[i] > a.src[j]) inversions++;
        }
    }
    int parity = (inversions % 2 == 0) ? 1 : -1;
    return parity * a.sign[0] * a.sign[1] * a.sign[2] == 1;
}

// Rotate a sensor-frame vector into the body frame. Safe when in == out.
inline void applyAlignment(const AxisAlignment& a, const float in[3], float out[3]) {
    const float v[3] = {in[0], in[1], in[2]};  // Copy first so in and out may alias
    for (int i = 0; i < 3; i++) {
        out[i] = a.sign[i] * v[a.src[i]];
    }
}

// Common flat mountings, named like Betaflight's board alignment.
// CWn: chip facing up, sensor rotated n degrees clockwise (seen from above)
//      relative to the body.
// CWn_FLIP: chip facing down — the board is first flipped over the forward (X)
//      axis, then rotated n degrees clockwise (seen from above).
// Any of the other 16 square mountings can be written as an AxisAlignment directly.
namespace Align {
    constexpr AxisAlignment CW0       {{0, 1, 2}, { 1,  1,  1}};
    constexpr AxisAlignment CW90      {{1, 0, 2}, { 1, -1,  1}};
    constexpr AxisAlignment CW180     {{0, 1, 2}, {-1, -1,  1}};
    constexpr AxisAlignment CW270     {{1, 0, 2}, {-1,  1,  1}};
    constexpr AxisAlignment CW0_FLIP  {{0, 1, 2}, { 1, -1, -1}};
    constexpr AxisAlignment CW90_FLIP {{1, 0, 2}, {-1, -1, -1}};
    constexpr AxisAlignment CW180_FLIP{{0, 1, 2}, {-1,  1, -1}};
    constexpr AxisAlignment CW270_FLIP{{1, 0, 2}, { 1,  1, -1}};
}
