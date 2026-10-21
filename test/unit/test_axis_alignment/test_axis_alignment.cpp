#include <unity.h>
#include <cmath>
#include "utils/AxisAlignment.h"
#include "config/SensorConfig.h"

void setUp() {}
void tearDown() {}

// Gravity reading (g) on a still sensor: the axis pointing UP reads +1.
// Body frame FLU: X forward, Y left, Z up.
static void assertMaps(const AxisAlignment& a, const float sensor[3], float bx, float by, float bz) {
    float body[3];
    applyAlignment(a, sensor, body);
    TEST_ASSERT_EQUAL_FLOAT(bx, body[0]);
    TEST_ASSERT_EQUAL_FLOAT(by, body[1]);
    TEST_ASSERT_EQUAL_FLOAT(bz, body[2]);
}

void test_cw0_is_identity() {
    const float v[3] = {0.1f, -0.2f, 0.97f};
    assertMaps(Align::CW0, v, 0.1f, -0.2f, 0.97f);
}

void test_all_presets_are_valid_rotations() {
    const AxisAlignment presets[] = {
        Align::CW0, Align::CW90, Align::CW180, Align::CW270,
        Align::CW0_FLIP, Align::CW90_FLIP, Align::CW180_FLIP, Align::CW270_FLIP,
    };
    for (const auto& p : presets) {
        TEST_ASSERT_TRUE(isValidRotation(p));
    }
}

void test_presets_map_level_drone_to_body_up() {
    // Level drone: body Z (up) reads +1. Chip facing up: sensor z = +1. Chip facing down: sensor z = -1.
    const float chipUp[3] = {0.0f, 0.0f, 1.0f};
    const float chipDown[3] = {0.0f, 0.0f, -1.0f};
    assertMaps(Align::CW0, chipUp, 0, 0, 1);
    assertMaps(Align::CW90, chipUp, 0, 0, 1);
    assertMaps(Align::CW180, chipUp, 0, 0, 1);
    assertMaps(Align::CW270, chipUp, 0, 0, 1);
    assertMaps(Align::CW0_FLIP, chipDown, 0, 0, 1);
    assertMaps(Align::CW90_FLIP, chipDown, 0, 0, 1);
    assertMaps(Align::CW180_FLIP, chipDown, 0, 0, 1);
    assertMaps(Align::CW270_FLIP, chipDown, 0, 0, 1);
}

void test_cw90_sensor_y_points_forward() {
    // Sensor rotated 90 deg clockwise (from above): its +Y points forward, its +X points right.
    // Nose pointing up -> the forward-pointing sensor axis (+Y) reads +1.
    const float noseUp[3] = {0.0f, 1.0f, 0.0f};
    assertMaps(Align::CW90, noseUp, 1, 0, 0);
    // Right side up -> sensor +X (pointing right) reads +1 -> body left (Y) reads -1.
    const float rightUp[3] = {1.0f, 0.0f, 0.0f};
    assertMaps(Align::CW90, rightUp, 0, -1, 0);
}

void test_cw180_axes_reversed_in_plane() {
    // Sensor +X points backward: nose up -> sensor x reads -1.
    const float noseUp[3] = {-1.0f, 0.0f, 0.0f};
    assertMaps(Align::CW180, noseUp, 1, 0, 0);
}

void test_cw0_flip_left_and_up_reversed() {
    // Flipped over X: sensor +Y points right, sensor +Z points down.
    // Left side up -> sensor y reads -1 -> body Y reads +1.
    const float leftUp[3] = {0.0f, -1.0f, 0.0f};
    assertMaps(Align::CW0_FLIP, leftUp, 0, 1, 0);
}

void test_exactly_24_of_48_signed_permutations_are_rotations() {
    const uint8_t perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    int valid = 0;
    for (const auto& p : perms) {
        for (int s = 0; s < 8; s++) {
            AxisAlignment a{{p[0], p[1], p[2]},
                            {static_cast<int8_t>((s & 1) ? -1 : 1),
                             static_cast<int8_t>((s & 2) ? -1 : 1),
                             static_cast<int8_t>((s & 4) ? -1 : 1)}};
            if (isValidRotation(a)) valid++;
        }
    }
    TEST_ASSERT_EQUAL(24, valid);
}

void test_rejects_mirror_and_malformed() {
    TEST_ASSERT_FALSE(isValidRotation(AxisAlignment{{0, 1, 2}, {1, 1, -1}}));   // Mirror (det -1)
    TEST_ASSERT_FALSE(isValidRotation(AxisAlignment{{1, 0, 2}, {1, 1, 1}}));    // Axis swap without sign flip = mirror
    TEST_ASSERT_FALSE(isValidRotation(AxisAlignment{{0, 0, 2}, {1, 1, 1}}));    // Repeated axis
    TEST_ASSERT_FALSE(isValidRotation(AxisAlignment{{0, 1, 3}, {1, 1, 1}}));    // Axis out of range
    TEST_ASSERT_FALSE(isValidRotation(AxisAlignment{{0, 1, 2}, {1, 0, 1}}));    // Sign not ±1
}

void test_apply_preserves_magnitude() {
    const AxisAlignment presets[] = {
        Align::CW0, Align::CW90, Align::CW180, Align::CW270,
        Align::CW0_FLIP, Align::CW90_FLIP, Align::CW180_FLIP, Align::CW270_FLIP,
    };
    const float v[3] = {0.3f, -0.5f, 0.81f};
    const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    for (const auto& p : presets) {
        float out[3];
        applyAlignment(p, v, out);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, n, std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]));
    }
}

void test_apply_in_place() {
    float v[3] = {1.0f, 2.0f, 3.0f};
    applyAlignment(Align::CW90, v, v);  // in == out must be safe
    TEST_ASSERT_EQUAL_FLOAT(2.0f, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, v[2]);
}

void test_configured_alignment_is_valid() {
    TEST_ASSERT_TRUE(isValidRotation(IMU_ALIGNMENT));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_cw0_is_identity);
    RUN_TEST(test_all_presets_are_valid_rotations);
    RUN_TEST(test_presets_map_level_drone_to_body_up);
    RUN_TEST(test_cw90_sensor_y_points_forward);
    RUN_TEST(test_cw180_axes_reversed_in_plane);
    RUN_TEST(test_cw0_flip_left_and_up_reversed);
    RUN_TEST(test_exactly_24_of_48_signed_permutations_are_rotations);
    RUN_TEST(test_rejects_mirror_and_malformed);
    RUN_TEST(test_apply_preserves_magnitude);
    RUN_TEST(test_apply_in_place);
    RUN_TEST(test_configured_alignment_is_valid);

    return UNITY_END();
}
