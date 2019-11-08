#include "scs-impl.h"

#include "actuators.h"
#include "scs-state.h"
#include "user-interface.h"

#include <assert.h>

/**
 * Adapts the desired speed based upon how long the lever was held.
 */
static inline void handle_held_lever(scs_state scs, size_t system_time) {
    assert(scs.lever_pos != scs_Neutral);
    assert(scs.lever_last_tic != 0);

    size_t time_passed = system_time - scs.lever_last_tic;

    if (!scs.lever_continuous) { // Only started to hold lever.
        if (scs.lever_pos == scs_Upward5 && time_passed >= lever5_auto_time) {
            set_lever_continuous(true);
            lever_up5_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Downward5 && time_passed >= lever5_auto_time) {
            set_lever_continuous(true);
            lever_down5_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Upward7 && time_passed >= lever7_auto_time) {
            set_lever_continuous(true);
            lever_up7_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Downward7 && time_passed >= lever7_auto_time) {
            set_lever_continuous(true);
            lever_down7_step();
            set_lever_last_tic(system_time);
        }
    } else {
        if (scs.lever_pos == scs_Upward5 && time_passed >= lever5_frequency) {
            lever_up5_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Downward5 && time_passed >= lever5_frequency) {
            lever_down5_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Upward7 && time_passed >= lever7_frequency) {
            lever_up7_step();
            set_lever_last_tic(system_time);
        } else if (scs.lever_pos == scs_Downward7 && time_passed >= lever7_frequency) {
            lever_down7_step();
            set_lever_last_tic(system_time);
        }
    }
}

static inline void handle_lever_release(scs_state scs, size_t system_time) {
    assert(scs.lever_pos == scs_Neutral);

    if (!scs.lever_continuous) { // Only do a step if lever was not hold continuously.
        if (scs.lever_prev_pos == scs_Upward5) {
            lever_up5_step();
        } else if (scs.lever_prev_pos == scs_Upward7) {
            lever_up7_step();
        } else if (scs.lever_prev_pos == scs_Downward5) {
            lever_down5_step();
        } else if (scs.lever_prev_pos == scs_Downward7) {
            lever_down7_step();
        }
    } else {
        set_lever_continuous(false);
    }

    set_lever_release(true);
    set_lever_last_tic(0);
}

void scs_do_step(void) {
    size_t time = get_time();
    scs_state last_scs = get_scs_state();

    // Mock calls are required.
    (void)get_brightness();
    (void)get_key_status();
    (void)get_all_doors_closed();
    (void)get_reverse_gear();
    (void)get_voltage_battery();
    (void)get_steering_angle();
    (void)get_oncoming_traffic();
    (void)get_camera_state();

    int engine_on = get_engine_status();
    if (!engine_on) {
        // Engine was turned off, SCS-1 requires the prev. desired speed to reset
        reset_prev_desired_speed();
        set_cruise_control(false);
    }

    // Note: Speed not actually a sensor in SCS specification.
    set_vehicle_speed(get_current_speed());

    // directional lever logic
    if (last_scs.lever_pos != scs_Neutral) {
        if (!last_scs.lever_last_tic) {
            /* This is the first step in which the lever is registered as
               "changed out of neutral position".
             */
            set_lever_last_tic(time);
        } else {
            handle_held_lever(last_scs, time);
        }
    } else if (!last_scs.lever_release_processed) {
        handle_lever_release(last_scs, time);
    }
}
