#ifndef __CRUISE_CONTROL_USER_INTERFACE_INCLUDED
#define __CRUISE_CONTROL_USER_INTERFACE_INCLUDED

typedef enum {scs_Neutral, scs_Downward5, scs_Downward7, scs_Upward5, scs_Upward7, scs_Backward, scs_Forward} SCSLever;

typedef enum {two-secs, two-point-five-secs, three-secs} safetyDistance;

void lever_forward(void);

void lever_up(void);

void lever_down(void);

void lever_backward(void);

// TODO: all values possible at all time?
void turn_lever(safetyDistance distance);

void press_button(void);

// TODO: not finished yet

#endif
