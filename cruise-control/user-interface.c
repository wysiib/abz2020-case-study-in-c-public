#include "user-interface.h"
#include "scs-state.h"
#include "sensors.h"

#define MIN_CRUISE_CONTROL_SPEED 200

void lever_forward(void) {

    scs_state scs = get_scs_state();

    if (scs.has_previous_desired_speed) { // SCS-2
        set_prev_desired_speed(scs.previous_desired_speed);
        set_cruise_control(true);
    } else if (scs.current_speed >= MIN_CRUISE_CONTROL_SPEED) {
        set_prev_desired_speed(scs.current_speed);
        set_cruise_control(true);
    }
}

void lever_up5(void) {
    scs_state scs = get_scs_state();

    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.previous_desired_speed;
        if (desired < speed_max) {
            set_prev_desired_speed(desired + 1);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_prev_desired_speed(curr);
    }
}

static inline vehicleSpeed getTensPlace(vehicleSpeed s) {
    // Remove the last two digits (0-9kmh).
    return (s / 100) * 100;
}

void lever_up7(void) {
    scs_state scs = get_scs_state();

    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.previous_desired_speed;
        if (desired < speed_max) {
            vehicleSpeed next = getTensPlace(desired) + 100;
            set_prev_desired_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_prev_desired_speed(curr);
    }
}

void lever_down5(void) {
    scs_state scs = get_scs_state();

    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.previous_desired_speed;
        if (desired > speed_min) {
            set_prev_desired_speed(desired - 1);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_prev_desired_speed(curr);
    }
}

void lever_down7(void) {
    scs_state scs = get_scs_state();

    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.previous_desired_speed;
        if (desired > speed_min) {
            vehicleSpeed next = getTensPlace(desired);
            if (desired == next) {
                next -= 100;
            }
            set_prev_desired_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_prev_desired_speed(curr);
    }
}

void lever_backward(void) { set_cruise_control(false); }
