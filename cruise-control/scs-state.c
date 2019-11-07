#include "scs-state.h"
#include <assert.h>

static scs_state scs;

scs_state get_scs_state(void) {
    return scs;
}

void set_prev_desired_speed(vehicleSpeed prev) {
    assert(prev >= speed_min && prev <= speed_max);
    scs.has_previous_desired_speed = true;
    scs.previous_desired_speed = prev;
}

void reset_prev_desired_speed() {
    scs.has_previous_desired_speed = false;
    scs.previous_desired_speed = 0;
}
