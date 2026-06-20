#include "arc.h"
#include "planner.h"
#include "stepper.h"
#include "config.h"
#include "pico/time.h"
#include <Arduino.h>  // for yield() — services USB during tight loops

float planner_arc_tolerance_mm = DEFAULT_ARC_TOLERANCE;

bool arc_interpolate(float target_x, float target_y,
                     float offset_i, float offset_j,
                     bool clockwise,
                     float feedrate_mm_min) {
    float start_x, start_y;
    plan_get_position(&start_x, &start_y);

    float cx = start_x + offset_i;
    float cy = start_y + offset_j;
    float rsq = offset_i * offset_i + offset_j * offset_j;
    if (rsq < 1e-10f) return false;
    float radius = sqrtf(rsq);

    // Vector from center to start = -(I, J). Vector from center to end.
    float r_start_x = -offset_i;
    float r_start_y = -offset_j;
    float r_end_x = target_x - cx;
    float r_end_y = target_y - cy;

    // Angular travel. atan2 of the cross/dot gives the signed angle between
    // the two radius vectors in the range (-π, π].
    float theta_start = atan2f(r_start_y, r_start_x);
    float theta_end   = atan2f(r_end_y,   r_end_x);
    float sweep = theta_end - theta_start;

    // Normalize sweep into the correct sign for the commanded direction.
    if (clockwise) {
        if (sweep >= 0.0f) sweep -= 2.0f * (float)M_PI;
    } else {
        if (sweep <= 0.0f) sweep += 2.0f * (float)M_PI;
    }
    // If start == end (full circle requested with same point), sweep is 0;
    // interpret as a full turn in the commanded direction.
    if (fabsf(sweep) < 1e-6f) {
        sweep = clockwise ? -2.0f * (float)M_PI : 2.0f * (float)M_PI;
    }

    // Segment count from chord-error bound:
    // chord half-angle θ_h satisfies r·(1 − cos θ_h) = tolerance
    // ⇒ θ_h ≈ sqrt(2·tolerance/r) for small tolerance
    // n_seg = ceil(|sweep| / (2·θ_h))
    float tol = planner_arc_tolerance_mm;
    if (tol <= 0.0f) tol = 1e-4f;
    float half_chord_rad = sqrtf(2.0f * tol / radius);
    uint32_t n_seg = (uint32_t)ceilf(fabsf(sweep) / (2.0f * half_chord_rad));

    // Floor on segment length (Marlin-style). Tight chord tolerance on small
    // arcs generates dozens of sub-mm segments; the planner can't build a
    // speed ramp across them (v_reach per segment is sqrt(2·a·L)) and the
    // ISR pays a ~1 s first-step penalty whenever v_entry=0. For a pen
    // plotter, 0.5 mm chord is well below anything you can see drawn —
    // capping the count here keeps arcs fast without sacrificing accuracy.
    const float ARC_MIN_SEGMENT_MM = 0.5f;
    float arc_length = fabsf(sweep) * radius;
    uint32_t n_seg_by_length = (uint32_t)ceilf(arc_length / ARC_MIN_SEGMENT_MM);
    if (n_seg > n_seg_by_length) n_seg = n_seg_by_length;
    if (n_seg < 1) n_seg = 1;
    if (n_seg > 4096) n_seg = 4096;

    float delta = sweep / (float)n_seg;
    float cos_d = cosf(delta);
    float sin_d = sinf(delta);

    float rx = r_start_x;
    float ry = r_start_y;

    for (uint32_t i = 1; i < n_seg; i++) {
        float nrx = rx * cos_d - ry * sin_d;
        float nry = rx * sin_d + ry * cos_d;
        rx = nrx; ry = nry;
        float px = cx + rx;
        float py = cy + ry;

        while (plan_buffer_available() == 0) {
            stepper_wake();
            stepper_service_sync();
            yield();
        }
        plan_buffer_line(px, py, feedrate_mm_min, false);
    }
    // Final point — use the exact commanded target (not the accumulated
    // rotation) to avoid drift.
    while (plan_buffer_available() == 0) {
        stepper_wake();
        yield();
    }
    plan_buffer_line(target_x, target_y, feedrate_mm_min, false);
    return true;
}
