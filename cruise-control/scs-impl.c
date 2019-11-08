#include "scs-impl.h"
#include "scs-state.h"
#include "user-interface.h"
#include "actuators.h"

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
    (void) get_camera_state();

    int engine_on = get_engine_status();
    if (!engine_on) {
        // Engine was turned off, SCS-1 requires the prev. desired speed to reset
        reset_prev_desired_speed();
        set_cruise_control(false);
    }

    // Note: Speed not actually a sensor in SCS specification.
    set_vehicle_speed(get_current_speed());
}
