#ifndef CRUISE_CONTROL_STATE
#define CRUISE_CONTROL_STATE

#include "../common/common.h"
#include "user-interface.h"
#include <sys/types.h>

typedef ssize_t vehicleAcceleration; // Given in m/s².
#define VEHICLE_MAX_DECELERATION ((vehicleAcceleration) -5)
#define VEHICLE_MAX_ACCELERATION ((vehicleAcceleration) 2)

typedef enum {
    simple = 1,
    adaptive = 2
} cruiseControlMode;

typedef struct {
    bool is_on;
    bool started_playing;
    bool playing_sound; // Indicates whether the acousic signal is played.
    size_t start_time;
} acousticSignal;

typedef struct {
    cruiseControlMode mode;

    bool cruise_control_active;
    vehicleSpeed desired_speed; // Speed the driver wants to go by.
    bool has_previous_desired_speed;
    vehicleSpeed previous_desired_speed; // Previously desired speed for cruise control.
    // TODO: Do we want to save the current speed in here?
    // It should always be equal to the sensor value, but having it saved here
    // eases test mock handling.
    vehicleSpeed target_speed; // Speed the adaptive cruise control aims for.
    vehicleSpeed current_speed; // Last seen value via sensor.

    vehicleAcceleration acceleration; // Desired vehicle acceleration in m/s².

    SCSLever lever_pos;
    SCSLever lever_prev_pos; // Previous lever position. Might be equal to lever_pos.
    size_t lever_last_tic; // Time in ms at which the lever was lastly evaluated. If 0, lever change was not yet registered.
    bool lever_release_processed; // Whether the releasing of the lever was yet processed
    bool lever_continuous; // If true, lever is held continuously -> alter speeds accordingly.

    vehicleSpeed vehicle_speed_infront;
    vehicleAcceleration vehicle_acceleration_infront;

    size_t safety_dist; // Safety distance to keep in meters. TODO: needs to be calculated
    safetyDistance safety_dist_time;

    bool visual_warning_on;
    acousticSignal acoustic_warning;

    bool brake_assistant_available; // SCS-27
    percentage brake_pressure;
    bool brake_warning_playing; // SCS-28
} scs_state;

bool scs_state_equals(scs_state s1, scs_state s2){
    return
    s1.cruise_control_active == s2.cruise_control_active &&
    s1.desired_speed == s2.desired_speed &&
    s1.has_previous_desired_speed == s2.has_previous_desired_speed &&
    s1.previous_desired_speed == s2.previous_desired_speed &&
    s1.target_speed == s2.target_speed &&
    s1.current_speed == s2.current_speed &&
    s1.acceleration == s2.acceleration &&
    s1.lever_pos == s2.lever_pos &&
    s1.lever_prev_pos == s2.lever_prev_pos &&
    s1.lever_last_tic == s2.lever_last_tic &&
    s1.lever_release_processed == s2.lever_release_processed &&
    s1.lever_continuous == s2.lever_continuous &&
    s1.vehicle_speed_infront == s2.vehicle_speed_infront &&
    s1.vehicle_acceleration_infront == s2.vehicle_acceleration_infront &&
    s1.safety_dist == s2.safety_dist &&
    s1.safety_dist_time == s2.safety_dist_time &&
    s1.visual_warning_on == s2.visual_warning_on &&

    s1.acoustic_warning.is_on == s2.acoustic_warning.is_on &&
    s1.acoustic_warning.start_time == s2.acoustic_warning.start_time &&
    s1.acoustic_warning.playing_sound == s2.acoustic_warning.playing_sound &&
    s1.acoustic_warning.started_playing == s2.acoustic_warning.started_playing &&

    s1.brake_assistant_available == s2.brake_assistant_available &&
    s1.brake_pressure == s2.brake_pressure &&
    s1.brake_warning_playing == s2.brake_warning_playing;
}

scs_state get_scs_state(void);

/** Zeroes the SCS state. */
void reset(void **state);

void set_scs_mode(cruiseControlMode mode);

void set_cruise_control(bool active);

void set_desired_speed(vehicleSpeed desired);

void set_prev_desired_speed(vehicleSpeed prev);

void reset_prev_desired_speed(void);

void set_target_speed(vehicleSpeed speed);

void set_current_speed(vehicleSpeed speed);

//
// Acceleration
//

void set_acceleration(vehicleAcceleration);

//
// Lever handling.
//

/*
    Note: Overview of the lever pulling logic.

    The user-interface only provides access to setting a lever into a
    specific position (up5, up7, down5, down7, foward, backward)
    and releasing it again (neutral).

    The scs-impl has to follow the semantics outlined here.

    set_lever is called to store the info which lever was pulled in the SCS
    state, whereby the state keeps track of which lever position was set before
    (lever_prev_pos).
    Upon start of the engine, lever_pos = lever_prev_pos = neutral position.

    The set_lever_continuous should be fired if one of the up or down positions
    is held by the driver for a specified amount of time (see lever5_auto_time
    and friends in scs-impl.h). The behaviour of the lever changes depending on
    whether this time was exceeded or not.

    If exceeded, a lever dependend update to the desired speed is made and
    repeated in a specific time frequency (see lever5_frequency and friends)
    as long as the lever is held. Releasing the lever between two such frequency
    tics does not impose an additional update.

    If not exceeded, an update to the desired speed is made once the lever is
    released (neutral position).
    As the update is dependend on the lever position before the release,
    lever_prev_pos has to be queried to decide for the appropriate update.

    In the state variable lever_last_tic, the system time is saved at which
    either
    * the lever was moved out of neutral position,
    * the desired speed was updated due to lever frequency.
    This time can be 0 if the lever was released and no timings need to be
    checked for.
    A zero value is equivalent to "no time is saved".

    As for releasing the lever, the state variable lever_release_processed
    is set to false, which denotes that the release of the lever is still
    pending to be processed.
    After processing of the release, it should be set back to true,
    so that the released lever is not processed more than once.

 */

void set_lever(SCSLever pos);

void set_lever_last_tic(size_t time);

void set_lever_continuous(bool continuous);

/** Set whether the lever release was processed by the system. */
void set_lever_release(bool processed);

/** Calculates a step of holding the lever up to the first resistance level (5°). */
void lever_up5_step(void);

/** Calculates a step of holding the lever beyond the first resistance level (7°). */
void lever_up7_step(void);

/** Calculates a step of holding the lever down to the first resistance level (5°). */
void lever_down5_step(void);

/** Calculates a step of holding the lever below the first resistance level (7°). */
void lever_down7_step(void);

//
// Safety distance.
//

void set_vehicle_speed_infront(vehicleSpeed speed);

void set_vehicle_acceleration_infront(vehicleAcceleration acceleration);

void set_safety_distance(size_t meters);

void set_safety_distance_time(safetyDistance distance);

void start_acoustic_signal(size_t start_time);

//
// Brake assistant.
//

void brake_assistant_available(bool available);

void start_acoustic_brake_warning(size_t start_time);

void reset_acoustic_brake_warning(void);

#endif
