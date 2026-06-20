#pragma once

#include <Arduino.h>

// CoreXY forward/inverse kinematics for Blot.
//
// Inverse (Cartesian → motor-space mm):
//     a =  x - y
//     b = -x - y
// Forward (motor-space mm → Cartesian mm):
//     x = (a - b) / 2
//     y = -(a + b) / 2
//
// Note the Y sign: the mechanical Y axis on Blot runs opposite to the math
// convention we want for G-code (positive G-code Y = upward on paper).
// Negating Y once here keeps GCODE semantics clean and the rest of the
// firmware doesn't need to know about it.
//
// Both motors use the same steps/mm, so conversion to steps is just
// multiplication by STEPS_PER_MM.
//
// motor_load_for_unit_dir() returns max(|a_per_L|, |b_per_L|) — the factor
// you multiply Cartesian tip velocity/accel by to get the peak motor
// velocity/accel. It's 1 for pure X/Y, √2 for ±45°.

inline void cartesian_to_motor(float x, float y, float *a, float *b) {
    float y_mot = -y;
    *a = x + y_mot;
    *b = y_mot - x;
}

inline void motor_to_cartesian(float a, float b, float *x, float *y) {
    *x =  0.5f * (a - b);
    *y = -0.5f * (a + b);
}

// Direction-dependent motor load. Absolute values make this invariant under
// Y-inversion — it's the same magnitude either way.
inline float motor_load_for_unit_dir(float ux, float uy) {
    float a = fabsf(ux - uy);
    float b = fabsf(uy + ux);
    return (a > b) ? a : b;
}
