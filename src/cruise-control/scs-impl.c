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

static inline float meters_per_sec(vehicleSpeed tens_kmh) {
    return (float)tens_kmh / 36.f;
}

static inline size_t meters_to_standstill(float mps) {
    assert(VEHICLE_MAX_DECELERATION == (vehicleAcceleration)-5);
    return (size_t)((mps * mps) / 10.f); // 2.f * MAX_DECELERATION (5.f)
}

static inline size_t seconds_to_standstill(float mps) {
    assert(VEHICLE_MAX_DECELERATION == (vehicleAcceleration)-5);
    return (size_t)(mps / 5.f); // 2.f * MAX_DECELERATION (5.f)
}

static inline size_t seconds_to_collision(float mps, size_t collision_dist) {
    return (size_t)((float)collision_dist / mps);
}

static inline void handle_range_radar(scs_state scs,
                                      rangeRadar collision_dist,
                                      size_t system_time) {
    assert(scs.mode == adaptive);
    assert((collision_dist >= distance_min) && (collision_dist <= distance_max));

    if (scs.safety_dist >= (size_t)collision_dist) {
        // Welcome to problem town!
        set_acceleration(VEHICLE_MAX_DECELERATION); // Could be smoother, but max works.

        // Check for warning signals.
        float mps = meters_per_sec(scs.current_speed);
        bool need_acoustic_warning = ((float)collision_dist < (mps * 0.8)); // SCS-26
        if (need_acoustic_warning && !(scs.acoustic_warning.is_on)) {
            start_acoustic_signal(system_time);
        }
        bool need_visual_warning = ((float)collision_dist < (mps * 1.5f)); // SCS-25
        if (need_visual_warning) {
            visual_warning(true);
        } else {
            visual_warning(false);
        }

        // Set new target speed.
        if (scs.vehicle_speed_infront < scs.desired_speed) {
            set_target_speed(scs.vehicle_speed_infront);
        } else {
            set_target_speed(scs.desired_speed);
        }
    } else {
        // Ensure target speed matches desired speed.
        set_target_speed(scs.desired_speed);
    }
}

static inline void run_acoustic_warning(acousticSignal warning, size_t now) {
    assert(warning.is_on && warning.started_playing &&
           (warning.start_time != (size_t)0) &&
           (!get_scs_state().brake_warning_playing));
    /* First 0.1 sec: Sound on.
       0.1--0.3 sec: Sound off.
       0.3--0.4 sec: Sound on.
       >0.4 sec: Warning off.
     */

    size_t play_time_ms = now - warning.start_time; // Time the signal is playing already.

    if (play_time_ms < (size_t)100) {
        acoustic_warning(true);
    } else if (play_time_ms < (size_t)300) {
        acoustic_warning(false);
    } else if (play_time_ms < (size_t)400) {
        acoustic_warning(true);
    } else {
        reset_acoustic_signal();
    }
}

static inline void run_brake_warning(acousticSignal warning, size_t now) {
    assert(warning.is_on && warning.started_playing &&
           (warning.start_time != (size_t)0) &&
           get_scs_state().brake_warning_playing);
    /* First 0.10 sec: Sound on.
       0.10--0.15 sec: Sound off.
       0.15--0.25 sec: Sound on.
       0.25--0.30 sec: Sound off.
       0.30--0.40 sec: Sound on.
       >0.4 sec: Sound off.
     */

    size_t play_time_ms = now - warning.start_time; // Time the signal is playing already.

    if (play_time_ms < (size_t)100) {
        acoustic_warning(true);
    } else if (play_time_ms < (size_t)150) {
        acoustic_warning(false);
    } else if (play_time_ms < (size_t)250) {
        acoustic_warning(true);
    } else if (play_time_ms < (size_t)300) {
        acoustic_warning(false);
    } else if (play_time_ms < (size_t)400) {
        acoustic_warning(true);
    } else {
        reset_acoustic_signal();
    }

    reset_acoustic_brake_warning();
}

static inline void handle_brake_assist(scs_state scs,
                                       size_t collision_dist,
                                       size_t system_time) {
    assert(scs.brake_assistant_available);
    float mps = meters_per_sec(scs.current_speed);
    size_t standstill_ms = seconds_to_standstill(mps) * (size_t)1000;
    size_t collision_ms = seconds_to_collision(mps, collision_dist) * (size_t)1000;

    if (collision_ms <= standstill_ms) { // exact seconds case
        brake_pressure(100);
        // start_acoustic_brake_warning(system_time);
    } else if (collision_ms <= (standstill_ms + (size_t)1500)) { // + 1.5 seconds case
        brake_pressure(60);
        // start_acoustic_brake_warning(system_time);
    } else if (collision_ms <= (standstill_ms + (size_t)3000)) { // + 3 seconds case
        brake_pressure(20);
        // start_acoustic_brake_warning(system_time);
    } else {
        // Do nothing.
        // TODO: release brake pressure?
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
    vehicleSpeed current_speed = get_current_speed();
    set_current_speed(current_speed);

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

    // Calculate safety distance
    if (last_scs.mode == adaptive) {
        float mps;
        float ms;
        if (last_scs.vehicle_speed_infront < (vehicleSpeed)200) {
            // SCS-23
            mps = meters_per_sec(last_scs.vehicle_speed_infront);
            if (last_scs.vehicle_acceleration_infront <= 0) { // Not accelerating.
                ms = 2500.f;
            } else {
                ms = 3000.f;
            }
        } else {
            mps = meters_per_sec(last_scs.current_speed);
            ms = (float)safety_ms(last_scs.safety_dist_time);
        }
        size_t dist = (size_t)((mps * ms) / 1000.f);
        set_safety_distance(dist);
        last_scs.safety_dist = dist; // Update for further calculations.
    }

    // Check distance and adjust target speed.
    (void)get_range_radar_state();
    rangeRadar collision_dist = read_range_radar_sensor();
    if (last_scs.mode == adaptive) {
        if ((collision_dist >= distance_min) && (collision_dist <= distance_max)) {
            handle_range_radar(last_scs, collision_dist, time);
        } else {
            set_target_speed(get_scs_state().desired_speed);
        }
    } else {
        // If the simple cruise control is used, desired_speed == target_speed.
        set_target_speed(get_scs_state().desired_speed);
    }

    // Acoustic signal
    if (last_scs.mode == adaptive) {
        // Might be the signal only started playing in this frame.
        // Query SCS again.
        scs_state current_scs = get_scs_state();
        if (current_scs.acoustic_warning.is_on &&
            !(current_scs.acoustic_warning.started_playing)) {
            start_acoustic_signal(time);
        } else if (last_scs.acoustic_warning.is_on && !last_scs.brake_warning_playing) {
            run_acoustic_warning(last_scs.acoustic_warning, time);
        } else {
            // Nothing to play.
        }
    }

    // Brake assistant.
    if ((collision_dist >= distance_min) && (collision_dist <= distance_max)) {
        // SCS-27
        if ((last_scs.vehicle_speed_infront == (vehicleSpeed)0) &&
            (current_speed <= (vehicleSpeed)600)) {
            brake_assistant_available(true);
        } else if ((last_scs.vehicle_speed_infront > (vehicleSpeed)0) &&
                   (current_speed <= (vehicleSpeed)1200)) {
            brake_assistant_available(true);
        } else {
            // NOTE: This case is actually not specified, and I do not see a reason
            // as of why the assistant should be unavailable besides testing the
            // boundaries.
            brake_assistant_available(false);
        }
    } else {
        // NOTE: It is also not a requirement for the brake assistant to detect
        // an object in the safety distance to be active, but otherwise
        // there should not be any vehicle infront which can be found to be
        // stationary or not.
        // Again, for testing reasons, the assistant is deactivated here.
        brake_assistant_available(false);
    }

    if (get_scs_state().brake_assistant_available) {
        handle_brake_assist(get_scs_state(), collision_dist, time);
    }
    if (last_scs.brake_warning_playing) {
        run_brake_warning(last_scs.acoustic_warning, time);
    }

    // Post conditions/invariants.
    scs_state new_scs = get_scs_state();
    // not(adaptive) => target_speed == desired_speed
    assert((new_scs.mode == adaptive) || (new_scs.target_speed == new_scs.desired_speed));
}
