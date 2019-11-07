#ifndef CRUISE_CONTROL_STATE
#define CRUISE_CONTROL_STATE

#include "../common/common.h"

typedef struct {
    bool has_previous_desired_speed;
    vehicleSpeed previous_desired_speed;
    // TODO: Do we want to save the current speed in here?
    // It should always be equal to the sensor value, but having it saved here
    // eases test mock handling.
    vehicleSpeed current_speed; // Last seen value via sensor.
} scs_state;

scs_state get_scs_state(void);

/** Zeroes the SCS state. */
void reset(void **state);

void set_prev_desired_speed(vehicleSpeed prev);

void reset_prev_desired_speed();

void observe_current_speed(vehicleSpeed speed);

#endif
