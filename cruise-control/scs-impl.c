#include "scs-impl.h"
#include "scs-state.h"

void reset(void **state) {}

void scs_do_step(void) {
    // Mock calls are required.
    (void) get_brightness();
    (void) get_time();
    (void) get_key_status();
    (void) get_all_doors_closed();
    (void) get_reverse_gear();
    (void) get_voltage_battery();
    (void) get_steering_angle();
    (void) get_oncoming_traffic();

    int engine_on = get_engine_status();
    if (!engine_on) {
        // Engine was turned off, SCS-1 requires the prev. desired speed to reset
        reset_prev_desired_speed();
    }
}
