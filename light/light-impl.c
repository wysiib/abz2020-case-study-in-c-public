#include "sensors.h"
#include "user-interface.h"
#include "actuators.h"

static size_t when_light_on = 0;
static bool last_engine = 0;

void set_all_lights(percentage p) {
    set_low_beam_left(p);
    set_low_beam_right(p);
    set_tail_lamp_left(p);
    set_tail_lamp_right(p);
}

void light_do_step(void) {
    keyState ks = get_key_status();
    bool engine_on = get_engine_status();
    bool all_doors_closed = get_all_doors_closed();
    bool reverse_gear = get_reverse_gear();
    voltage voltage_battery = get_voltage_battery();
    steeringAngle angle = get_steering_angle();
    bool oncomming_trafic = get_oncoming_traffic();

    brightness bb = get_brightness();
    size_t tt = get_time();

    if (get_light_rotary_switch() == lrs_auto) {
        if (bb < 200 && when_light_on == 0) {
            when_light_on = tt;
            set_all_lights(100);
        }

        if (bb >= 250 && tt - when_light_on >= 3) {
            set_all_lights(0);
            when_light_on = 0;
        }
    }

    if (get_light_rotary_switch() == lrs_on) {
        // TODO: maybe manipulate when_light_on
        set_all_lights(100);
    }

    if (ks != KeyInIgnitionOnPosition) {
        if (last_engine == 0 && ks == KeyInserted && get_light_rotary_switch() == lrs_on) {
            set_all_lights(50);
        } else {
            set_all_lights(0);
            when_light_on = 0;
        }
    }

    last_engine = engine_on;
}
