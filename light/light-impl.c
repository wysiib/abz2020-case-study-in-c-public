#include "sensors.h"
#include "user-interface.h"
#include "actuators.h"

static size_t when_light_on = 0;

static bool last_engine = 0;
static lightRotarySwitch last_lrs = lrs_off;
static keyState last_key_state = NoKeyInserted;
static bool last_all_door_closed = 0;

static size_t ambi_light_timer = 0;

void reset(void) {
    reset_user_interface();
    when_light_on = 0;

    last_engine = 0;
    last_lrs = lrs_off;
    last_key_state = NoKeyInserted;
    last_all_door_closed = 0;

    ambi_light_timer = 0;
}

static void set_all_lights(percentage p) {
    set_low_beam_left(p);
    set_low_beam_right(p);
    set_tail_lamp_left(p);
    set_tail_lamp_right(p);
}

static void update_ambient_light_timer(keyState old, keyState new,
                                       bool doors_old, bool doors_new,
                                       size_t time) {
    if (old != new || doors_old != doors_new) {
        ambi_light_timer = time;
    }
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


    // engine turned off
    if (last_engine == 1 && engine_on == 0 && get_ambient_light() == 0) {
        set_all_lights(0);
    }

    update_ambient_light_timer(last_key_state, ks,
                               last_all_door_closed, all_doors_closed,
                               tt);

    if (engine_on == 0 && get_ambient_light()) {
        if (tt - ambi_light_timer >= 30) {
            set_all_lights(0);
        }
    }

    if (get_light_rotary_switch() == lrs_auto) {
        if (engine_on && bb < 200 && when_light_on == 0) {
            when_light_on = tt;
            set_all_lights(100);
        }

        if (bb >= 250 && tt - when_light_on >= 3) {
            set_all_lights(0);
            when_light_on = 0;
        }
    }

    if (engine_on && get_light_rotary_switch() == lrs_on) {
        // TODO: maybe manipulate when_light_on
        set_all_lights(100);
    }

    if (ks != KeyInIgnitionOnPosition && get_light_rotary_switch() != lrs_auto) {
        if (ks == KeyInserted && get_light_rotary_switch() == lrs_on && last_lrs != lrs_on) {
            set_all_lights(50);
        }  else {
            set_all_lights(0);
            when_light_on = 0;
        }
        if (get_pitman_vertical() == pa_Downward7) {
            set_low_beam_left(10);
            set_tail_lamp_left(10);
        } else if (get_pitman_vertical() == pa_Upward7) {
            set_low_beam_right(10);
            set_tail_lamp_right(10);
        }
    }

    last_lrs = get_light_rotary_switch();
    last_engine = engine_on;
    last_key_state = ks;
    last_all_door_closed = all_doors_closed;
}
