#include "user-interface.h"
#include "scs-state.h"
#include "sensors.h"

void lever_forward(void) {

    scs_state scs = get_scs_state();

    if (scs.has_previous_desired_speed) { // SCS-2
        set_prev_desired_speed(scs.previous_desired_speed);
    } else {
        set_prev_desired_speed(scs.current_speed);
    }
}
