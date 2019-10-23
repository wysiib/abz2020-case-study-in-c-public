#include "sensors.h"
#include "user-interface.h"
#include "actuators.h"

static size_t when_light_on = 0;

void set_all_lights(percentage p) {
    set_low_beam_left(p);
    set_low_beam_right(p);
    set_tail_lamp_left(p);
    set_tail_lamp_right(p);
}

void light_do_step(void) {
    // TODO: read all sensors
    brightness bb = get_brightness();
    size_t tt = get_time();

    if (get_light_rotary_switch() == lrs_auto) {
        if (bb < 200 && when_light_on == 0) {
            when_light_on = tt;
            set_all_lights(100);
        }

        if (bb >= 250 && tt - when_light_on >= 3) { // TODO: check value 250?
            set_all_lights(0);
            when_light_on = 0;
        }
    }

    if (get_light_rotary_switch() == lrs_on) {
        // TODO: maybe manipulate when_light_on
        set_all_lights(100);
    }
}
