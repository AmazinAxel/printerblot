#include "planner.h"
#include "kinematics.h"
#include "stepper.h"

// Ring buffer of planned blocks. head = next slot to write. tail = block
// the stepper is currently executing (or about to). planned = oldest block
// whose entry_speed can still be modified by lookahead (≥ tail).
static plan_block_t block_buffer[PLANNER_BUFFER_SIZE];
static volatile uint8_t buffer_head;
static volatile uint8_t buffer_tail;
static volatile uint8_t buffer_planned;

// Last queued Cartesian position (the planner's virtual tool position).
static float last_pos[2] = {0.0f, 0.0f};
// Commanded motor-step position corresponding to last_pos. This lets us
// round every new block's motor deltas RELATIVE TO AN ABSOLUTE TARGET
// instead of per-segment, which keeps rounding error bounded at ±0.5 step
// everywhere on the path — no cumulative drift across many arc segments.
static int32_t last_machine_steps[2] = {0, 0};
// Unit vector of the previous non-zero block (for junction deviation with
// the next one). Zeroed when there's no previous block.
static float prev_unit_vec[2] = {0.0f, 0.0f};
static bool  prev_unit_valid  = false;

// Tunables (defaults; settings.cpp can overwrite)
float planner_max_feedrate_mm_min    = DEFAULT_MAX_FEEDRATE_MM_MIN;
float planner_max_rapid_mm_min       = DEFAULT_MAX_RAPID_MM_MIN;
float planner_max_accel_mm_s2        = DEFAULT_MAX_ACCEL_MM_S2;
float planner_motor_max_accel_mm_s2  = MOTOR_MAX_ACCEL_MM_S2;
float planner_motor_max_rate_mm_min  = MOTOR_MAX_RATE_MM_MIN;
float planner_junction_deviation_mm  = DEFAULT_JUNCTION_DEVIATION;

static inline uint8_t next_index(uint8_t i) {
    return (i + 1) & (PLANNER_BUFFER_SIZE - 1);
}
static inline uint8_t prev_index(uint8_t i) {
    return (i + PLANNER_BUFFER_SIZE - 1) & (PLANNER_BUFFER_SIZE - 1);
}

void plan_init() {
    plan_reset();
}

void plan_reset() {
    noInterrupts();
    buffer_head = 0;
    buffer_tail = 0;
    buffer_planned = 0;
    prev_unit_valid = false;
    prev_unit_vec[0] = prev_unit_vec[1] = 0.0f;
    last_machine_steps[0] = 0;
    last_machine_steps[1] = 0;
    interrupts();
}

bool plan_is_empty() {
    return buffer_head == buffer_tail;
}

uint8_t plan_buffer_available() {
    int8_t free = buffer_tail - buffer_head - 1;
    if (free < 0) free += PLANNER_BUFFER_SIZE;
    return (uint8_t)free;
}

plan_block_t *plan_get_current_block() {
    if (buffer_head == buffer_tail) return nullptr;
    return &block_buffer[buffer_tail];
}

void plan_discard_current_block() {
    if (buffer_head == buffer_tail) return;
    uint8_t old_tail = buffer_tail;
    buffer_tail = next_index(buffer_tail);
    // buffer_planned is the cursor the forward pass starts from. If it was
    // pointing at the block we just consumed, it's now BEHIND tail — the
    // slot is stale data (may contain leftover fields from a sync block,
    // or overwritten garbage). The next planner_recalculate would iterate
    // through that stale slot and corrupt the real new block's entry_speed
    // via the v_max calculation, producing rate=1 stepper crawl. Advance
    // planned so it always points to a valid, non-consumed block.
    if (buffer_planned == old_tail) {
        buffer_planned = buffer_tail;
    }
}

void plan_get_position(float *x, float *y) {
    *x = last_pos[0];
    *y = last_pos[1];
}

void plan_set_position(float x, float y) {
    // Can't change position mid-motion. Caller should flush first.
    last_pos[0] = x;
    last_pos[1] = y;
    // Re-anchor the absolute step counter so the next block's rounding is
    // exact relative to this new position.
    float a, b;
    cartesian_to_motor(x, y, &a, &b);
    last_machine_steps[0] = (int32_t)lroundf(a * STEPS_PER_MM);
    last_machine_steps[1] = (int32_t)lroundf(b * STEPS_PER_MM);
    prev_unit_valid = false;
}

// ───────── helpers ─────────

// Fill in step counts, direction bits, step_event_count, unit vector,
// and Cartesian length. Step deltas are computed by the caller from the
// absolute target (see plan_buffer_line) to avoid per-segment rounding drift.
static void populate_geometry(plan_block_t *b,
                              int32_t na, int32_t nb,
                              float dx, float dy, float length_mm) {
    b->steps[0] = na;
    b->steps[1] = nb;
    b->direction_bits = 0;
    if (na < 0) { b->direction_bits |= 0x01; b->steps[0] = -na; }
    if (nb < 0) { b->direction_bits |= 0x02; b->steps[1] = -nb; }

    uint32_t ena = (uint32_t)b->steps[0];
    uint32_t enb = (uint32_t)b->steps[1];
    b->step_event_count = (ena > enb) ? ena : enb;

    b->millimeters = length_mm;

    if (length_mm > 0.0f) {
        float inv_L = 1.0f / length_mm;
        b->unit_vec[0] = dx * inv_L;
        b->unit_vec[1] = dy * inv_L;
    } else {
        b->unit_vec[0] = 0.0f;
        b->unit_vec[1] = 0.0f;
    }
}

// Cartesian accel for this move, clamped so neither motor exceeds its
// physical accel limit. See README "CoreXY-aware acceleration".
static float clamp_accel_for_direction(const float unit_vec[2]) {
    float load = motor_load_for_unit_dir(unit_vec[0], unit_vec[1]);
    if (load < 1e-6f) return planner_max_accel_mm_s2;
    float motor_cap = planner_motor_max_accel_mm_s2 / load;
    return fminf(planner_max_accel_mm_s2, motor_cap);
}

// Same idea for top speed.
static float clamp_speed_for_direction(float desired_mm_s,
                                       const float unit_vec[2]) {
    float load = motor_load_for_unit_dir(unit_vec[0], unit_vec[1]);
    float cap_user  = planner_max_feedrate_mm_min / 60.0f;
    float cap_motor = (load > 1e-6f)
                          ? (planner_motor_max_rate_mm_min / 60.0f) / load
                          : cap_user;
    float cap = fminf(cap_user, cap_motor);
    return fminf(desired_mm_s, cap);
}

// GRBL-style junction deviation. Returns max tip speed (mm/s) that can
// transition from prev_unit_vec → this_unit_vec without exceeding
// acceleration limits at the corner.
static float junction_speed(const float v_in[2], const float v_out[2],
                            float accel_mm_s2) {
    // Cosine of the angle between incoming motion and outgoing motion.
    // Dot product of v_in with v_out; we want the "bend" angle, so:
    // cos_theta_junction = -(v_in · v_out)
    float cos_t = -(v_in[0] * v_out[0] + v_in[1] * v_out[1]);
    if (cos_t <= -0.999999f) {
        // Near-straight continuation; no junction limit.
        return 1.0e9f;
    }
    if (cos_t >= 0.999999f) {
        // Full reversal — must stop at corner.
        return 0.0f;
    }
    // sin(θ/2) via half-angle identity
    float sin_half = sqrtf(0.5f * (1.0f - cos_t));
    // GRBL formula: v² = a * δ * sin(θ/2) / (1 - sin(θ/2))
    float v_sq = accel_mm_s2 * planner_junction_deviation_mm
                 * sin_half / (1.0f - sin_half);
    return sqrtf(v_sq);
}

// Compute the trapezoid fields (initial/nominal/final rate, accel_st,
// accelerate_until, decelerate_after) for a block. The in-flight block
// can be recomputed too (e.g. to extend cruise when a new block arrives),
// so the final field-write burst is bracketed by noInterrupts/interrupts
// to keep the stepper from catching a half-updated state.
static void compute_trapezoid(plan_block_t *b) {
    if (b->step_event_count == 0) {
        noInterrupts();
        b->initial_rate = 0;
        b->nominal_rate = 0;
        b->final_rate = 0;
        b->accel_st = 0;
        b->accelerate_until = 0;
        b->decelerate_after = 0;
        b->recalculate_flag = false;
        interrupts();
        return;
    }

    float L = b->millimeters;
    float events_per_mm = (float)b->step_event_count / L;

    float v_entry = b->entry_speed;
    float v_nom   = b->nominal_speed;
    // v_exit = next block's entry speed (velocity is continuous across
    // block boundaries). 0 if this is the newest block — no next yet.
    float v_exit  = 0.0f;
    uint8_t next_idx = next_index((uint8_t)(b - block_buffer));
    if (next_idx != buffer_head) {
        v_exit = block_buffer[next_idx].entry_speed;
    }

    float a = b->acceleration;
    float inv_2a = 0.5f / a;
    float d_accel = (v_nom * v_nom - v_entry * v_entry) * inv_2a;
    float d_decel = (v_nom * v_nom - v_exit  * v_exit ) * inv_2a;
    if (d_accel < 0.0f) d_accel = 0.0f;
    if (d_decel < 0.0f) d_decel = 0.0f;
    float d_cruise = L - d_accel - d_decel;

    if (d_cruise < 0.0f) {
        float v_peak_sq =
            (2.0f * a * L + v_entry * v_entry + v_exit * v_exit) * 0.5f;
        if (v_peak_sq < v_entry * v_entry) v_peak_sq = v_entry * v_entry;
        if (v_peak_sq < v_exit  * v_exit ) v_peak_sq = v_exit  * v_exit;
        d_accel = (v_peak_sq - v_entry * v_entry) * inv_2a;
        if (d_accel < 0.0f) d_accel = 0.0f;
        if (d_accel > L)    d_accel = L;
        d_cruise = 0.0f;
        v_nom = sqrtf(v_peak_sq);
    }

    uint32_t new_initial_rate = (uint32_t)(v_entry * events_per_mm);
    uint32_t new_nominal_rate = (uint32_t)(v_nom   * events_per_mm);
    uint32_t new_final_rate   = (uint32_t)(v_exit  * events_per_mm);
    uint32_t new_accel_st     = (uint32_t)(a * events_per_mm);

    uint32_t accel_events = (uint32_t)(d_accel * events_per_mm);
    uint32_t decel_events = (uint32_t)(d_decel * events_per_mm);
    if (accel_events > b->step_event_count) accel_events = b->step_event_count;
    if (accel_events + decel_events > b->step_event_count) {
        decel_events = b->step_event_count - accel_events;
    }
    uint32_t new_accelerate_until = accel_events;
    uint32_t new_decelerate_after = b->step_event_count - decel_events;

    // Atomic swap — a running stepper ISR must never see a half-updated
    // mix of old rate fields and new phase breakpoints.
    noInterrupts();
    b->initial_rate       = new_initial_rate;
    b->nominal_rate       = new_nominal_rate;
    b->final_rate         = new_final_rate;
    b->accel_st           = new_accel_st;
    b->accelerate_until   = new_accelerate_until;
    b->decelerate_after   = new_decelerate_after;
    b->recalculate_flag   = false;
    interrupts();
}

// Backward pass: from newest block toward planned pointer, clamp each
// block's entry_speed so that we can still decelerate to the next block's
// entry speed within this block's length.
static void planner_recalculate() {
    if (buffer_head == buffer_tail) return;

    uint8_t last = prev_index(buffer_head);   // newest block
    if (last == buffer_tail) {
        // Only one block in buffer. Entry = max_entry, exit = 0.
        plan_block_t *b = &block_buffer[last];
        b->entry_speed = fminf(b->max_entry_speed, b->nominal_speed);
        compute_trapezoid(b);
        buffer_planned = last;
        return;
    }

    // Reverse pass. The newest block always plans to decelerate to zero,
    // so its entry starts at min(max_entry, nominal).
    uint8_t idx = last;
    block_buffer[last].entry_speed =
        fminf(block_buffer[last].max_entry_speed,
              block_buffer[last].nominal_speed);
    // The last block will decelerate from min(max_entry, nominal) down to 0.

    while (idx != buffer_planned) {
        uint8_t pidx = prev_index(idx);
        if (pidx == (uint8_t)(buffer_tail - 1 + PLANNER_BUFFER_SIZE) % PLANNER_BUFFER_SIZE)
            break;
        plan_block_t *cur  = &block_buffer[idx];
        plan_block_t *prev = &block_buffer[pidx];

        // Max entry into prev such that it can decelerate to cur->entry over prev->length.
        float v_reach_sq = cur->entry_speed * cur->entry_speed
                           + 2.0f * prev->acceleration * prev->millimeters;
        float v_max = sqrtf(v_reach_sq);
        float new_entry = fminf(prev->max_entry_speed, v_max);
        new_entry = fminf(new_entry, prev->nominal_speed);

        if (fabsf(new_entry - prev->entry_speed) > 1e-4f) {
            prev->entry_speed = new_entry;
            prev->recalculate_flag = true;
        } else {
            // Entry didn't change; deeper blocks won't change either.
            break;
        }

        idx = pidx;
        if (idx == buffer_tail) break;
    }

    // Forward pass: clamp next-block entry to what cur can accel up to,
    // and ALWAYS recompute cur's trapezoid. The trapezoid's exit speed
    // depends on the next block's entry; if we only re-run when cur's
    // entry changed, we miss the common case "block 2 arrived, so block 1
    // can now cruise through instead of decelerating to zero."
    idx = buffer_planned;
    while (idx != buffer_head) {
        uint8_t nidx = next_index(idx);
        plan_block_t *cur = &block_buffer[idx];

        if (nidx != buffer_head) {
            plan_block_t *nxt = &block_buffer[nidx];
            float v_reach_sq = cur->entry_speed * cur->entry_speed
                               + 2.0f * cur->acceleration * cur->millimeters;
            float v_max = sqrtf(v_reach_sq);
            if (nxt->entry_speed > v_max) {
                nxt->entry_speed = v_max;
            }
        }

        compute_trapezoid(cur);
        idx = nidx;
    }

    buffer_planned = buffer_tail;
}

// ───────── public queueing ─────────

bool plan_buffer_line(float x_target, float y_target,
                      float feedrate_mm_min, bool rapid) {
    // Back-pressure: caller should check plan_buffer_available() first.
    if (next_index(buffer_head) == buffer_tail) return false;

    float dx = x_target - last_pos[0];
    float dy = y_target - last_pos[1];
    float L = sqrtf(dx * dx + dy * dy);

    // Degenerate zero-length move — silently drop. Leave last_machine_steps
    // alone so the next block still rounds relative to the absolute target.
    if (L < 1e-6f) {
        last_pos[0] = x_target;
        last_pos[1] = y_target;
        return true;
    }

    // Motor step targets are rounded from the absolute commanded position,
    // not the per-segment delta. Over a long arc chain this bounds cumulative
    // drift at ±0.5 step rather than letting it random-walk with √N.
    float a_mm, b_mm;
    cartesian_to_motor(x_target, y_target, &a_mm, &b_mm);
    int32_t tgt_a = (int32_t)lroundf(a_mm * STEPS_PER_MM);
    int32_t tgt_b = (int32_t)lroundf(b_mm * STEPS_PER_MM);
    int32_t na = tgt_a - last_machine_steps[0];
    int32_t nb = tgt_b - last_machine_steps[1];

    plan_block_t *b = &block_buffer[buffer_head];
    populate_geometry(b, na, nb, dx, dy, L);

    float desired_mm_min = rapid ? planner_max_rapid_mm_min : feedrate_mm_min;
    if (desired_mm_min <= 0.0f) desired_mm_min = planner_max_feedrate_mm_min;
    float desired_mm_s = desired_mm_min / 60.0f;

    b->nominal_speed = clamp_speed_for_direction(desired_mm_s, b->unit_vec);
    b->acceleration  = clamp_accel_for_direction(b->unit_vec);

    // Junction deviation with previous block.
    if (prev_unit_valid) {
        b->max_entry_speed = junction_speed(prev_unit_vec, b->unit_vec,
                                            b->acceleration);
    } else {
        b->max_entry_speed = 0.0f;
    }
    if (b->max_entry_speed > b->nominal_speed)
        b->max_entry_speed = b->nominal_speed;

    b->entry_speed = 0.0f;         // will be set by recalculate()
    b->sync_cmd = SYNC_NONE;
    b->sync_arg = 0;
    b->recalculate_flag = true;

    // Commit
    prev_unit_vec[0] = b->unit_vec[0];
    prev_unit_vec[1] = b->unit_vec[1];
    prev_unit_valid  = true;
    last_pos[0] = x_target;
    last_pos[1] = y_target;
    last_machine_steps[0] = tgt_a;
    last_machine_steps[1] = tgt_b;

    buffer_head = next_index(buffer_head);
    planner_recalculate();
    return true;
}

#ifdef FIRMWARE_TEST
// Test-only accessor: returns a pointer to the (offset)-th queued block,
// counting from the tail (oldest). Returns nullptr if offset is past head.
extern "C" plan_block_t *block_buffer_get(uint8_t offset) {
    uint8_t idx = (buffer_tail + offset) & (PLANNER_BUFFER_SIZE - 1);
    // Walk from tail to head; if we reach head before offset, it's out of range.
    uint8_t walk = buffer_tail;
    for (uint8_t i = 0; i < offset; i++) {
        walk = next_index(walk);
        if (walk == buffer_head) return nullptr;
    }
    return &block_buffer[idx];
}
#endif

bool plan_sync_command(uint8_t cmd, int32_t arg) {
    if (next_index(buffer_head) == buffer_tail) return false;

    plan_block_t *b = &block_buffer[buffer_head];
    // Zero-distance block. Stepper treats this as "immediately fire the sync
    // command, then advance."
    b->steps[0] = b->steps[1] = 0;
    b->direction_bits = 0;
    b->step_event_count = 0;
    b->millimeters = 0.0f;
    b->unit_vec[0] = b->unit_vec[1] = 0.0f;
    b->nominal_speed = b->entry_speed = b->max_entry_speed = 0.0f;
    b->acceleration = planner_max_accel_mm_s2;
    b->initial_rate = b->nominal_rate = b->final_rate = 0;
    b->accel_st = 0;
    b->accelerate_until = b->decelerate_after = 0;
    b->sync_cmd = cmd;
    b->sync_arg = arg;
    b->recalculate_flag = false;

    // Sync blocks break the junction chain — there's no continuous motion
    // across them, so the next move starts from zero.
    prev_unit_valid = false;

    buffer_head = next_index(buffer_head);
    planner_recalculate();
    return true;
}
