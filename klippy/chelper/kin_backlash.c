// Backlash compensation stepper pulse time generation
//
// Copyright (C) 2025  Klipper Backlash Contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// Backlash compensation works by detecting direction changes at move
// boundaries and applying a position offset to compensate for mechanical
// play in the drivetrain.
//
// Without backlash compensation, the stepper position is:
//     stepper_position(t) = original_position(t)
//
// With backlash compensation enabled, when a direction change is detected:
//     compensated_position(t) = original_position(t) + backlash_offset
//
// The backlash_offset is +backlash/2 when moving in the positive direction
// and -backlash/2 when moving in the negative direction after a reversal.
// This ensures the motor "leads" the tool by half the backlash distance,
// so the mechanical play is absorbed during the direction change.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible, container_of
#include "itersolve.h" // struct stepper_kinematics
#include "list.h" // list_node, list_head
#include "trapq.h" // struct move, move_get_coord


/****************************************************************
 * Backlash parameter list (supports runtime changes via G-code)
 ****************************************************************/

struct backlash_params {
    double backlash[3]; // [0]=X, [1]=Y, [2]=Z in mm
    double active_print_time;
    struct list_node node;
};


/****************************************************************
 * Backlash per-axis state
 ****************************************************************/

struct backlash_axis_state {
    double previous_pos;       // Last known tool position for this axis
    struct move *previous_move; // Last move processed (for direction detection)
};


/****************************************************************
 * Backlash stepper wrapper struct
 ****************************************************************/

struct backlash_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct list_head backlash_list;
    struct backlash_axis_state axes[3]; // [0]=X, [1]=Y, [2]=Z
};


/****************************************************************
 * Direction detection and backlash offset calculation
 ****************************************************************/

// Determine net displacement direction of a move for a given axis.
// Returns +1.0 for positive displacement, -1.0 for negative, 0.0 for none.
static double
get_move_direction(struct move *m, int axis)
{
    double move_dist = (m->start_v + m->half_accel * m->move_t) * m->move_t;
    double axis_dist = m->axes_r.axis[axis] * move_dist;
    if (axis_dist > 0.)
        return 1.0;
    if (axis_dist < 0.)
        return -1.0;
    return 0.0;
}

// Calculate backlash offset for a given axis at a move boundary.
// Returns the position offset to apply, or 0.0 if no offset needed.
static double
get_backlash_offset(struct backlash_stepper *bs, struct move *m, int axis)
{
    struct backlash_axis_state *ba = &bs->axes[axis];

    // Determine current backlash value from parameter list
    double backlash = 0.;
    struct backlash_params *bp = list_last_entry(
            &bs->backlash_list, struct backlash_params, node);
    while (unlikely(bp->active_print_time > m->print_time) &&
            !list_is_first(&bp->node, &bs->backlash_list)) {
        bp = list_prev_entry(bp, node);
    }
    backlash = bp->backlash[axis];

    if (backlash <= 0. || !ba->previous_move)
        return 0.;

    // Detect new move boundary
    if (m != ba->previous_move) {
        double prev_dir = get_move_direction(ba->previous_move, axis);
        double curr_dir = get_move_direction(m, axis);
        ba->previous_move = m;

        // Check for direction change (both directions must be non-zero
        // and opposite sign)
        if (prev_dir != 0. && curr_dir != 0. && prev_dir != curr_dir) {
            // Direction changed - apply backlash offset
            // Motor leads by backlash/2 in the current direction
            return curr_dir * backlash * 0.5;
        }
    }

    return 0.;
}


/****************************************************************
 * Position calculation callbacks
 ****************************************************************/

// Generic callback that applies backlash on specified axes
static double
backlash_calc_position(struct stepper_kinematics *sk, struct move *m,
                       double move_time, int apply_x, int apply_y, int apply_z)
{
    struct backlash_stepper *bs = container_of(
            sk, struct backlash_stepper, sk);
    if (!bs->orig_sk)
        return 0.;

    // Calculate base position from original kinematics
    double base = bs->orig_sk->calc_position_cb(bs->orig_sk, m, move_time);

    // Apply backlash offsets if at move boundary (move_time == 0)
    if (move_time <= 0.) {
        double offset = 0.;
        if (apply_x)
            offset += get_backlash_offset(bs, m, 0);
        if (apply_y)
            offset += get_backlash_offset(bs, m, 1);
        if (apply_z)
            offset += get_backlash_offset(bs, m, 2);

        if (offset != 0.) {
            // Update previous position for direction tracking
            struct coord end = move_get_coord(m, m->move_t);
            if (apply_x)
                bs->axes[0].previous_pos = end.x;
            if (apply_y)
                bs->axes[1].previous_pos = end.y;
            if (apply_z)
                bs->axes[2].previous_pos = end.z;

            return base + offset;
        }
    }

    return base;
}

// Optimized callback for X-axis only
static double
backlash_x_calc_position(struct stepper_kinematics *sk, struct move *m,
                         double move_time)
{
    return backlash_calc_position(sk, m, move_time, 1, 0, 0);
}

// Optimized callback for Y-axis only
static double
backlash_y_calc_position(struct stepper_kinematics *sk, struct move *m,
                         double move_time)
{
    return backlash_calc_position(sk, m, move_time, 0, 1, 0);
}

// Optimized callback for Z-axis only
static double
backlash_z_calc_position(struct stepper_kinematics *sk, struct move *m,
                         double move_time)
{
    return backlash_calc_position(sk, m, move_time, 0, 0, 1);
}

// General callback for all axes
static double
backlash_xyz_calc_position(struct stepper_kinematics *sk, struct move *m,
                           double move_time)
{
    return backlash_calc_position(sk, m, move_time, 1, 1, 1);
}


/****************************************************************
 * Interface functions
 ****************************************************************/

// Select the appropriate callback based on which axes are active
static void
backlash_update_callback(struct backlash_stepper *bs)
{
    int flags = bs->orig_sk->active_flags;
    int has_x = flags & AF_X;
    int has_y = flags & AF_Y;
    int has_z = flags & AF_Z;

    if (has_x && !has_y && !has_z)
        bs->sk.calc_position_cb = backlash_x_calc_position;
    else if (!has_x && has_y && !has_z)
        bs->sk.calc_position_cb = backlash_y_calc_position;
    else if (!has_x && !has_y && has_z)
        bs->sk.calc_position_cb = backlash_z_calc_position;
    else
        bs->sk.calc_position_cb = backlash_xyz_calc_position;
}

// Connect the backlash wrapper to the original kinematics
int __visible
backlash_set_sk(struct stepper_kinematics *sk,
                struct stepper_kinematics *orig_sk)
{
    struct backlash_stepper *bs = container_of(
            sk, struct backlash_stepper, sk);
    bs->orig_sk = orig_sk;
    bs->sk.active_flags = orig_sk->active_flags;
    bs->sk.commanded_pos = orig_sk->commanded_pos;
    bs->sk.last_flush_time = orig_sk->last_flush_time;
    bs->sk.last_move_time = orig_sk->last_move_time;
    backlash_update_callback(bs);
    return 0;
}

// Set backlash value for a specific axis (0=X, 1=Y, 2=Z)
void __visible
backlash_set_backlash(struct stepper_kinematics *sk, int axis,
                      double backlash)
{
    struct backlash_stepper *bs = container_of(
            sk, struct backlash_stepper, sk);
    if (axis < 0 || axis > 2)
        return;

    // Check if value changed
    struct backlash_params *last = list_last_entry(
            &bs->backlash_list, struct backlash_params, node);
    if (last->backlash[axis] == backlash)
        return;

    // Add new parameter entry
    struct backlash_params *bp = malloc(sizeof(*bp));
    memset(bp, 0, sizeof(*bp));
    bp->backlash[0] = last->backlash[0];
    bp->backlash[1] = last->backlash[1];
    bp->backlash[2] = last->backlash[2];
    bp->backlash[axis] = backlash;
    bp->active_print_time = sk->last_flush_time;
    list_add_tail(&bp->node, &bs->backlash_list);

    // Cleanup old entries
    double cleanup_time = sk->last_flush_time - 1.;
    struct backlash_params *first = list_first_entry(
            &bs->backlash_list, struct backlash_params, node);
    while (!list_is_last(&first->node, &bs->backlash_list)) {
        struct backlash_params *next = list_next_entry(first, node);
        if (next->active_print_time >= cleanup_time)
            break;
        list_del(&first->node);
        free(first);
        first = next;
    }
}

// Reset backlash state (call after homing)
void __visible
backlash_set_position(struct stepper_kinematics *sk,
                      double x, double y, double z)
{
    struct backlash_stepper *bs = container_of(
            sk, struct backlash_stepper, sk);
    // Reset direction tracking - next move will be treated as first
    bs->axes[0].previous_move = NULL;
    bs->axes[1].previous_move = NULL;
    bs->axes[2].previous_move = NULL;
    bs->axes[0].previous_pos = x;
    bs->axes[1].previous_pos = y;
    bs->axes[2].previous_pos = z;
}

// Allocate a backlash stepper wrapper
struct stepper_kinematics * __visible
backlash_stepper_alloc(void)
{
    struct backlash_stepper *bs = malloc(sizeof(*bs));
    memset(bs, 0, sizeof(*bs));
    list_init(&bs->backlash_list);
    // Add default (zero) backlash parameters
    struct backlash_params *bp = malloc(sizeof(*bp));
    memset(bp, 0, sizeof(*bp));
    list_add_tail(&bp->node, &bs->backlash_list);
    return &bs->sk;
}

// Free a backlash stepper wrapper and its parameter list
void __visible
backlash_stepper_free(struct stepper_kinematics *sk)
{
    struct backlash_stepper *bs = container_of(
            sk, struct backlash_stepper, sk);
    while (!list_empty(&bs->backlash_list)) {
        struct backlash_params *bp = list_first_entry(
                &bs->backlash_list, struct backlash_params, node);
        list_del(&bp->node);
        free(bp);
    }
    free(sk);
}
