#include "scs-impl.h"

#include "actuators.h"
#include "scs-state.h"
#include "sensors.h"
#include "user-interface.h"

#include <assert.h>

/**
 * Adapts the desired speed based upon how long the lever was held.
 */
static inline void handle_held_lever(scs_state scs, size_t system_time) {
    assert(scs.lever_pos != scs_Neutral);
    assert(scs.lever_last_tic != (size_t)0);

    size_t time_passed = system_time - scs.lever_last_tic;

    if (!scs.lever_continuous) { // Only started to hold lever.
        if ((scs.lever_pos == scs_Upward5) && (time_passed >= lever5_auto_time)) {
            set_lever_continuous(true);
            lever_up5_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Downward5) && (time_passed >= lever5_auto_time)) {
            set_lever_continuous(true);
            lever_down5_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Upward7) && (time_passed >= lever7_auto_time)) {
            set_lever_continuous(true);
            lever_up7_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Downward7) && (time_passed >= lever7_auto_time)) {
            set_lever_continuous(true);
            lever_down7_step();
            set_lever_last_tic(system_time);
        } else {
            // do nothing
        }
    } else {
        if ((scs.lever_pos == scs_Upward5) && (time_passed >= lever5_frequency)) {
            lever_up5_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Downward5) && (time_passed >= lever5_frequency)) {
            lever_down5_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Upward7) && (time_passed >= lever7_frequency)) {
            lever_up7_step();
            set_lever_last_tic(system_time);
        } else if ((scs.lever_pos == scs_Downward7) && (time_passed >= lever7_frequency)) {
            lever_down7_step();
            set_lever_last_tic(system_time);
        } else {
            // do nothing
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
        } else {
            // do nothing?
        }
    } else {
        set_lever_continuous(false);
    }

    set_lever_release(true);
    set_lever_last_tic(0);
}

/** Returns the safety distance in milliseconds. */
static size_t safety_ms(safetyDistance dist) {
    size_t ms = 0;
    switch (dist) {
    case two_secs:
        ms = 2000;
        break;
    case two_point_five_secs:
        ms = 2500;
        break;
    case three_secs:
        ms = 3000;
        break;
    default:
        assert(0); // Cannot happen, enum is exhausted.
    }

    return ms;
}

static bool is_max_deceleration_insufficient(scs_state scs,
                                             rangeRadar collision_dist) {
    assert((collision_dist >= distance_min) && (collision_dist <= distance_max));

    float mps = (float)scs.current_speed / 36.f; // km/h -> m/s.
    float braking_dist = (mps * mps) / (2.f * -((float)VEHICLE_MAX_DECELERATION));

    bool insufficient = ((float)collision_dist < braking_dist);

    return insufficient;
}

static inline void handle_range_radar(scs_state scs, rangeRadar collision_dist) {
    assert(scs.mode == adaptive);
    assert((collision_dist >= distance_min) && (collision_dist <= distance_max));

    if (scs.safety_dist >= (size_t)collision_dist) {
        // Welcome to problem town!
        set_acceleration(VEHICLE_MAX_DECELERATION); // Could be smoother, but max works.

        if (is_max_deceleration_insufficient(scs, collision_dist) &&
            !(scs.acoustic_warning.is_on)) {
            acoustic_warning_on();
        }
    }
}

static inline void run_acoustic_signal(acousticSignal warning, size_t now) {
    assert(warning.is_on && warning.started_playing &&
           (warning.start_time != (size_t)0));
    /* First 0.1 sec: Sound on.
       0.1--0.3 sec: Sound off.
       0.3--0.4 sec: Sound on.
       >0.4 sec: Warning off.
     */

    size_t play_time_ms = now - warning.start_time; // Time the signal is playing already.

    if (play_time_ms < (size_t)100) {
        set_acoustic_warning_tone(true);
    } else if (play_time_ms < (size_t)300) {
        set_acoustic_warning_tone(false);
    } else if (play_time_ms < (size_t)400) {
        set_acoustic_warning_tone(true);
    } else {
        set_acoustic_signal(false);
    }
}

void scs_do_step(void) {
    size_t time = get_time();

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
    set_current_speed(get_current_speed());

    scs_state last_scs = get_scs_state();

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
    } else {
        // do nothing
    }

    // Check distance
    (void)get_range_radar_state();
    rangeRadar collision_dist = read_range_radar_sensor();
    if (last_scs.mode == adaptive) {
        if ((collision_dist >= distance_min) && (collision_dist <= distance_max)) {
            handle_range_radar(last_scs, collision_dist);
        }
    }

    // Acoustic signal
    if (last_scs.mode == adaptive) {
        // Might be the signal only started playing in this frame.
        // Query SCS again.
        scs_state current_scs = get_scs_state();
        if (current_scs.acoustic_warning.is_on &&
            !(current_scs.acoustic_warning.started_playing)) {
            start_acoustic_signal(time);
        } else if (last_scs.acoustic_warning.is_on) {
            run_acoustic_signal(last_scs.acoustic_warning, time);
        } else {
            // Nothing to play.
        }
    }
}
