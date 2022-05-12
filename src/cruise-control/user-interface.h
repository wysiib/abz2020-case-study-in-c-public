#ifndef CRUISE_CONTROL_USER_INTERFACE_H_INCLUDED
#define CRUISE_CONTROL_USER_INTERFACE_H_INCLUDED

#include "../common/common.h"

typedef enum {
    scs_Neutral,
    scs_Downward5,
    scs_Downward7,
    scs_Upward5,
    scs_Upward7,
    scs_Backward,
    scs_Forward
} SCSLever;

typedef enum {
    two_secs,
    two_point_five_secs,
    three_secs
} safetyDistance;

void lever_release(void);

void lever_forward(void);

/** Pushes the lever up to the first resistance level (5°). */
void lever_up5(void);

/** Pushes the lever above the first resistance level (7°). */
void lever_up7(void);

/** Pushes the lever down to the first resistance level (5°). */
void lever_down5(void);

/** Pushes the lever below the first resistance level (7°). */
void lever_down7(void);

void lever_backward(void);

// TODO: all values possible at all time?
void turn_lever(safetyDistance distance);

void press_button(void);

void brakePedal(Pedal deflection);

void gasPedal(Pedal deflection);

void toggle_traffic_sign_detection(void);

void toggle_adaptive_cruise_control(void);

#endif
