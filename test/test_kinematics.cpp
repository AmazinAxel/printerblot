#include "test.h"
#include "../src/kinematics.h"

// CoreXY inverse with Y-inversion: a = x - y, b = -(x + y)
// (Blot's mechanical Y axis is flipped relative to the math convention.)
TEST_CASE("kinematics: inverse at origin-like inputs") {
    float a, b;
    cartesian_to_motor(0, 0, &a, &b); CHECK_NEAR(a,  0.0f, 1e-6); CHECK_NEAR(b,  0.0f, 1e-6);
    cartesian_to_motor(1, 0, &a, &b); CHECK_NEAR(a,  1.0f, 1e-6); CHECK_NEAR(b, -1.0f, 1e-6);
    cartesian_to_motor(0, 1, &a, &b); CHECK_NEAR(a, -1.0f, 1e-6); CHECK_NEAR(b, -1.0f, 1e-6);
    cartesian_to_motor(-1,0, &a, &b); CHECK_NEAR(a, -1.0f, 1e-6); CHECK_NEAR(b,  1.0f, 1e-6);
}

TEST_CASE("kinematics: inverse on diagonal") {
    float a, b;
    cartesian_to_motor(1, 1, &a, &b); CHECK_NEAR(a,  0.0f, 1e-6); CHECK_NEAR(b, -2.0f, 1e-6);
    cartesian_to_motor(1,-1, &a, &b); CHECK_NEAR(a,  2.0f, 1e-6); CHECK_NEAR(b,  0.0f, 1e-6);
    cartesian_to_motor(-1,1, &a, &b); CHECK_NEAR(a, -2.0f, 1e-6); CHECK_NEAR(b,  0.0f, 1e-6);
    cartesian_to_motor(-1,-1,&a, &b); CHECK_NEAR(a,  0.0f, 1e-6); CHECK_NEAR(b,  2.0f, 1e-6);
}

TEST_CASE("kinematics: forward = inverse^-1") {
    // Random-ish inputs, round-trip
    const float pts[][2] = {
        {0.0f, 0.0f}, {5.0f, 0.0f}, {0.0f, 7.3f}, {12.5f, -3.2f},
        {-100.0f, 50.0f}, {0.001f, 0.001f}, {200.0f, 199.0f},
    };
    for (auto &p : pts) {
        float a, b, x2, y2;
        cartesian_to_motor(p[0], p[1], &a, &b);
        motor_to_cartesian(a, b, &x2, &y2);
        CHECK_NEAR(x2, p[0], 1e-5);
        CHECK_NEAR(y2, p[1], 1e-5);
    }
}

TEST_CASE("kinematics: motor load factor for axis-aligned directions = 1") {
    // Both motors move at the same rate → load == 1
    CHECK_NEAR(motor_load_for_unit_dir(1.0f, 0.0f), 1.0f, 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(-1.0f, 0.0f), 1.0f, 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(0.0f, 1.0f), 1.0f, 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(0.0f, -1.0f), 1.0f, 1e-6);
}

TEST_CASE("kinematics: motor load factor for ±45° diagonals = sqrt(2)") {
    // Only one motor moves → at √2× the tip speed → load == √2
    const float inv = 1.0f / std::sqrt(2.0f);
    CHECK_NEAR(motor_load_for_unit_dir(inv, inv),    std::sqrt(2.0f), 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(-inv, inv),   std::sqrt(2.0f), 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(inv, -inv),   std::sqrt(2.0f), 1e-6);
    CHECK_NEAR(motor_load_for_unit_dir(-inv, -inv),  std::sqrt(2.0f), 1e-6);
}

TEST_CASE("kinematics: motor load is between 1 and sqrt(2) for all directions") {
    // Sweep 360° and verify the load always falls in [1, √2].
    for (int deg = 0; deg < 360; deg += 7) {
        float r = (float)deg * (float)M_PI / 180.0f;
        float ux = std::cos(r), uy = std::sin(r);
        float load = motor_load_for_unit_dir(ux, uy);
        CHECK(load >= 1.0f - 1e-5f);
        CHECK(load <= std::sqrt(2.0f) + 1e-5f);
    }
}

TEST_CASE("kinematics: motor load is symmetric across quadrants") {
    // f(-ux, -uy) == f(ux, uy) because we take absolute values.
    const float dirs[][2] = {{0.8f, 0.6f}, {0.3f, -0.9f}, {-0.5f, 0.5f}};
    for (auto &d : dirs) {
        float f1 = motor_load_for_unit_dir( d[0],  d[1]);
        float f2 = motor_load_for_unit_dir(-d[0], -d[1]);
        CHECK_NEAR(f1, f2, 1e-6);
    }
}
