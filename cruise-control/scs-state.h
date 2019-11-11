#ifndef CRUISE_CONTROL_STATE
#define CRUISE_CONTROL_STATE

#include "../common/common.h"
#include "user-interface.h"

typedef struct {
    bool cruise_control_active;
    bool has_previous_desired_speed;
    vehicleSpeed previous_desired_speed;
    // TODO: Do we want to save the current speed in here?
    // It should always be equal to the sensor value, but having it saved here
    // eases test mock handling.
    vehicleSpeed current_speed; // Last seen value via sensor.

    SCSLever lever_pos;
    SCSLever lever_prev_pos; // Previous lever position. Might be equal to lever_pos.
    size_t lever_last_tic; // Time in ms at which the lever was lastly evaluated. If 0, lever change was not yet registered.
    bool lever_release_processed; // Whether the releasing of the lever was yet processed
    bool lever_continuous; // If true, lever is held continuously -> alter speeds accordingly.
} scs_state;

scs_state get_scs_state(void);

/** Zeroes the SCS state. */
void reset(void **state);

void set_cruise_control(bool active);

void set_prev_desired_speed(vehicleSpeed prev);

void reset_prev_desired_speed();

void observe_current_speed(vehicleSpeed speed);

// Lever handling.

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
void lever_up5_step();

/** Calculates a step of holding the lever beyond the first resistance level (7°). */
void lever_up7_step();

/** Calculates a step of holding the lever down to the first resistance level (5°). */
void lever_down5_step();

/** Calculates a step of holding the lever below the first resistance level (7°). */
void lever_down7_step();

#endif