#include "user-interface.h"
#include "scs-state.h"
#include "sensors.h"

#define MIN_CRUISE_CONTROL_SPEED ((vehicleSpeed) 200)

void lever_release(void) {
    set_lever(scs_Neutral);
}

void lever_forward(void) {

    scs_state scs = get_scs_state();

    if (scs.has_previous_desired_speed) { // SCS-2
        set_prev_desired_speed(scs.previous_desired_speed);
        set_cruise_control(true);
    } else if (scs.current_speed >= MIN_CRUISE_CONTROL_SPEED) {
        set_prev_desired_speed(scs.current_speed);
        set_cruise_control(true);
    } else {
        // do nothing?
    }
}

void lever_up5(void) {
    set_lever(scs_Upward5);
}

void lever_up7(void) {
    set_lever(scs_Upward7);
}

void lever_down5(void) {
    set_lever(scs_Downward5);
}

void lever_down7(void) {
    set_lever(scs_Downward7);
}

void lever_backward(void) { set_cruise_control(false); }
