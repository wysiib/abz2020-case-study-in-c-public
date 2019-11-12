#include "scs-state.h"
#include "actuators.h"
#include <assert.h>

#include "user-interface.h"

static scs_state scs;

scs_state get_scs_state(void) {
    return scs;
}

void reset(void **state) {
    scs = (scs_state){0};
}

void set_cruise_control(bool active) {
    if (!active && scs.cruise_control_active) {
        // When turning cruise control off, we save the previous speed.
        // TODO: Check if this is really supposed to happen this way.
        set_prev_desired_speed(scs.target_speed);
    }
    scs.cruise_control_active = active;
}

void set_target_speed(vehicleSpeed target) {
    assert((target >= speed_min) && (target <= speed_max));
    scs.target_speed = target;
}

void set_prev_desired_speed(vehicleSpeed prev) {
    assert((prev >= speed_min) && (prev <= speed_max));
    scs.has_previous_desired_speed = true;
    scs.previous_desired_speed = prev;
}

void reset_prev_desired_speed(void) {
    scs.has_previous_desired_speed = false;
    scs.previous_desired_speed = 0;
}

void set_current_speed(vehicleSpeed current) {
    assert((current >= speed_min) && (current <= speed_max));
    scs.current_speed = current;
}

void set_lever(SCSLever pos) {
    assert(pos != scs.lever_pos); // TODO: Do we want this assertion?
    scs.lever_prev_pos = scs.lever_pos;
    scs.lever_pos = pos;
    if (pos == scs_Neutral) {
        scs.lever_release_processed = false;
    }
}

void set_lever_last_tic(size_t time) {
    scs.lever_last_tic = time;
}

void set_lever_continuous(bool continuous) {
    scs.lever_continuous = continuous;
}

void set_lever_release(bool processed) {
    assert(processed); // Should actually never be called to set it to false, should it?
    scs.lever_release_processed = processed;
}

void lever_up5_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.target_speed;
        if (desired < speed_max) {
            set_target_speed(desired + (vehicleSpeed) 10);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_target_speed(curr);
    }
}

static inline vehicleSpeed getTensPlace(vehicleSpeed s) {
    // Remove the last two digits (0-9kmh).
    return (s / (vehicleSpeed) 100) * (vehicleSpeed) 100;
}

void lever_up7_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.target_speed;
        if (desired < speed_max) {
            vehicleSpeed next = getTensPlace(desired) + (vehicleSpeed) 100;
            set_target_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_target_speed(curr);
    }
}

void lever_down5_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.target_speed;
        if (desired > speed_min) {
            set_target_speed(desired - (vehicleSpeed) 10);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_target_speed(curr);
    }
}

void lever_down7_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.target_speed;
        if (desired > speed_min) {
            vehicleSpeed next = getTensPlace(desired);
            if (desired == next) {
                next -= (vehicleSpeed) 100;
            }
            set_target_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_target_speed(curr);
    }
}

// Actuators ---------------------------------------------------------------- //

void set_vehicle_speed(vehicleSpeed target) {
    assert((target >= speed_min) && (target <= speed_max));

    // SCS-12: setVehicleSpeed = 0 => no speed to maintain.
    if (target == speed_min) {
        set_cruise_control(false);
    }
}
