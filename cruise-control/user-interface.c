#include "user-interface.h"
#include "scs-state.h"
#include "sensors.h"

#define MIN_CRUISE_CONTROL_SPEED 200

void lever_forward(void) {

    scs_state scs = get_scs_state();

    if (scs.has_previous_desired_speed) { // SCS-2
        set_prev_desired_speed(scs.previous_desired_speed);
        set_cruise_control(true);
    } else if (scs.current_speed >= MIN_CRUISE_CONTROL_SPEED) {
        set_prev_desired_speed(scs.current_speed);
        set_cruise_control(true);
    }
}

void lever_up5(void) {
    scs_state scs = get_scs_state();

    if (scs.cruise_control_active) {
        vehicleSpeed curr = scs.current_speed;
        if (curr < speed_max) {
            set_prev_desired_speed(curr + 1);
        }
    }
}
