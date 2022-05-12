#include "user-interface.h"
#include "scs-state.h"
#include "sensors.h"

#define MIN_CRUISE_CONTROL_SPEED ((vehicleSpeed)200)

void lever_release(void) {
    set_lever(scs_Neutral);
}

void lever_forward(void) {

    scs_state scs = get_scs_state();

    if (scs.has_previous_desired_speed) { // SCS-2
        set_desired_speed(scs.previous_desired_speed);
        set_cruise_control(true);
    } else {
        if (scs.current_speed >= MIN_CRUISE_CONTROL_SPEED) {
            set_desired_speed(scs.current_speed);
            set_cruise_control(true);
        } // Else do nothing.
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


void turn_lever(safetyDistance distance) {
    set_safety_distance_time(distance);
}

void brakePedal(Pedal deflection) {
    set_cruise_control(false);
}

void toggle_adaptive_cruise_control(void) {
    cruiseControlMode current_mode = get_scs_state().mode;

    if (current_mode == simple) {
        set_scs_mode(adaptive);
    } else {
        set_scs_mode(simple);
    }
}
