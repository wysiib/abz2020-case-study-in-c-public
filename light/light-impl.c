#include <assert.h>

#include "../system.h"

#include "sensors.h"
#include "user-interface.h"
#include "actuators.h"
#include "light-state.h"
#include "light-impl.h"

static size_t when_light_on = 0;

static bool last_engine = 0;
static lightRotarySwitch last_lrs = lrs_off;
static keyState last_key_state = NoKeyInserted;
static bool last_all_door_closed = 0;

static bool daytime_light_was_on = false;
static bool lrs_turned_on_while_key_inserted = false;

static size_t ambi_light_timer = 0;

static size_t blink_timer = 0;
static size_t remaining_blinks = 0;
static size_t pitman_arm_move_time = 0;
static pitmanArmUpDown last_pitman_arm = pa_ud_Neutral;
static bool blinking = false;
static blinkingDirection blinking_direction = none;

void reset(void **state) {
    (void) state;
    reset_user_interface();
    reset_lights();
    when_light_on = 0;

    last_engine = 0;
    last_lrs = lrs_off;
    last_key_state = NoKeyInserted;
    last_all_door_closed = 0;

    daytime_light_was_on = false;
    lrs_turned_on_while_key_inserted = false;

    ambi_light_timer = 0;

    blink_timer = 0;
    remaining_blinks = 0;
    pitman_arm_move_time = 0;
    last_pitman_arm = pa_ud_Neutral;
    blinking = false;
    blinking_direction = none;
}

static void set_all_lights(percentage p) {
    set_low_beam_left(p);
    set_low_beam_right(p);
    set_tail_lamp_left(p);
    set_tail_lamp_right(p);
}

static bool ambient_light_prevent_turnoff(size_t tt) {
    bool return_value = false;
    if(get_ambient_light()) {
        if ((tt - ambi_light_timer) < (size_t) 30000) { // only prolongs light, check for light rather than engine?
            return_value = true;
        }
    }
    return return_value;
}

static void update_ambient_light_status(keyState old, keyState new,
                                       bool doors_old, bool doors_new,
                                       size_t time, bool engine_on) {
    // ELS-19
    // only extend time if it has not yet passed
    if(ambient_light_prevent_turnoff(time)) {
        if ((old != new) || (doors_old != doors_new)) {
            ambi_light_timer = time;
        }
    }
}

static void set_blinkers_off(size_t time) {
    set_blink_left(0);
    set_blink_right(0);

    if((get_market_code() == USA) || (get_market_code() == Canada)) {
        switch (blinking_direction) {
            case blink_left:
                set_low_beam_left(50);
                set_tail_lamp_left(0);
                break;
            case blink_right:
                set_low_beam_right(50);
                set_tail_lamp_right(0);
                break;
            case hazard:
                set_low_beam_left(50);
                set_tail_lamp_left(0);
                set_low_beam_right(50);
                set_tail_lamp_right(0);
                break;
            case none:
                break;
            default:
                assert(0);
                break;
        }
    }

    blinking = false;
    blink_timer = time;
}

static void set_blinkers_on(size_t time) {
    // activate hazard warning blinks
    if(get_hazard_warning()) {
        blinking_direction = hazard;
        remaining_blinks = -1; // does not reset timings but keeps cycle
    }

    switch(blinking_direction) {
        case blink_left:
            set_blink_left(100);
            set_blink_right(0); // in case of fast switches
            break;
        case blink_right:
            set_blink_right(100);
            set_blink_left(0); // in case of fast switches
            break;
        case hazard:
            set_blink_left(100);
            set_blink_right(100);
            break;
        case none:
            break;
        default:
            assert(0);
            break;
    }
       
    if((get_market_code() == USA) || (get_market_code() == Canada)) {
        switch(blinking_direction) {
            case blink_left:
                set_low_beam_left(50);
                set_tail_lamp_left(100);
                break;
            case blink_right:
                set_low_beam_right(50);
                set_tail_lamp_right(100);
                break;
            case hazard:
                set_low_beam_left(50);
                set_tail_lamp_left(100);
                set_low_beam_right(50);
                set_tail_lamp_right(100);
                break;
            case none:
                break;
            default:
                assert(0);
                break;
        }
    }

    blink_timer = time;
    remaining_blinks--;
    blinking = true;
}

void light_loop(void) {
    // used as the verification target for CBMC
    while(true) {
        light_do_step();

        // assertions for cbmc to verify, i.e. invariants!
        // ELS-6: USA => light dimmed
        // assert(get_light_state().blinkLeft == 0);
    }
}

void light_do_step(void) {
    keyState ks = get_key_status();
    bool engine_on = get_engine_status();
    bool all_doors_closed = get_all_doors_closed();
    bool reverse_gear = get_reverse_gear();
    voltage voltage_battery = get_voltage_battery();
    bool undervoltage = (voltage_battery <= (voltage) 85);
    steeringAngle angle = get_steering_angle();
    bool oncomming_trafic = get_oncoming_traffic();

    brightness bb = get_brightness();
    size_t tt = get_time();

    sensorState camera = get_camera_state();
    (void) camera;
    vehicleSpeed speedo = get_current_speed();
    (void) speedo;

    update_ambient_light_status(last_key_state, ks,
                               last_all_door_closed, all_doors_closed,
                               tt, engine_on);

    // update flags
    if((ks == KeyInserted) && (get_light_rotary_switch() == lrs_on) && (last_lrs != lrs_on)) {
        lrs_turned_on_while_key_inserted = true;
    }
    if(ks != KeyInserted) {
        lrs_turned_on_while_key_inserted = false;
    }

    // engine turned off
    if (!engine_on && !ambient_light_prevent_turnoff(tt)) {
        set_all_lights(0);
    }

    if (!get_daytime_running_light() && (get_light_rotary_switch() == lrs_auto)) {
        if (engine_on && (bb < (brightness) 200) && (when_light_on == (size_t) 0)) {
            when_light_on = tt;
            set_all_lights(100);
        }

        if ((bb >= (brightness) 250) && ((tt - when_light_on) >= (size_t) 3000)) {
            set_all_lights(0);
            when_light_on = 0;
        }
    }

    if (engine_on && get_light_rotary_switch() == lrs_on) {
        // TODO: maybe manipulate when_light_on
        set_all_lights(100);
    }

    if ((ks != KeyInIgnitionOnPosition) && (get_light_rotary_switch() != lrs_auto) && !undervoltage) {
        if (get_pitman_vertical() == pa_Downward7) {
            set_low_beam_left(10);
            set_tail_lamp_left(10);
        }
        if (get_pitman_vertical() == pa_Upward7) {
            set_low_beam_right(10);
            set_tail_lamp_right(10);
        }
    }

    // ELS-17
    // activated after starting the engine
    if(get_daytime_running_light() && engine_on) {
        daytime_light_was_on = true;
    }
    // stay on as long as key is inserted
    if(daytime_light_was_on && (ks != NoKeyInserted)) {
        if((get_market_code() == USA) || (get_market_code() == Canada)) {
            // from szenario 7 but not from specification?
            set_low_beam_left(100);
            set_low_beam_right(100);
        } else {
            set_all_lights(100);
        }
    }

    // ELS-16 (has priority over ELS-17)
    if(!engine_on && (last_lrs != lrs_auto) && (get_light_rotary_switch() == lrs_auto)) {
        set_all_lights(0);
    }

    // ELS-15 (appears to have priority over ELS-16 and ELS-17 in test scenario 3)
    if((ks == KeyInserted) && !ambient_light_prevent_turnoff(tt) && lrs_turned_on_while_key_inserted) {
        set_all_lights(50);
    }

    // ELS-14
    if(engine_on && get_light_rotary_switch() == lrs_on) {
        set_all_lights(100);
    }

    // turn off hazard warning before new setting
    if((blinking_direction == hazard) && !get_hazard_warning() && !blinking && ((tt - blink_timer) >= (size_t) 500)) { // wait for dark cycle to complete
        blinking_direction=none;
        set_blinkers_off(tt);
    }

    // turn on hazard warning if not yet in mode
    if((blinking_direction == none) && get_hazard_warning()) {
        set_blinkers_on(tt);
    }

    // direction / blinking
    // blink as soon as arm is moved unless in dark cycle
    if((get_pitman_vertical() == pa_Downward5) || (get_pitman_vertical() == pa_Downward7)) {
        if(engine_on && ((tt - blink_timer) >= (size_t) 500) && !blinking) { // TODO: do we need to track the cycle instead of the timer?
            blinking = true;
            blinking_direction = blink_left;
            blink_timer = tt;
            remaining_blinks = -1;
            set_blinkers_on(tt);
        }
    }
    if((get_pitman_vertical() == pa_Upward5) || (get_pitman_vertical() == pa_Upward7)) {
        if(engine_on && ((tt - blink_timer) >= (size_t) 500) && !blinking) { // TODO: do we need to track the cycle instead of the timer?
            blinking = true;
            blinking_direction = blink_right;
            blink_timer = tt;
            remaining_blinks = -1;
            set_blinkers_on(tt);
        }
    }

    // ELS-2
    if((get_pitman_vertical() != last_pitman_arm) && ((tt - pitman_arm_move_time) < (size_t) 500)) {
        remaining_blinks = 2;
    }
    // otherwise check if arm was released later on -> stop blinking
    if((get_pitman_vertical() != last_pitman_arm) && (get_pitman_vertical() == pa_ud_Neutral) && ((tt - pitman_arm_move_time) >= (size_t) 500)) {
        remaining_blinks = 0;
        set_blink_left(0);
        set_blink_right(0);
        blinking = false;

        if((get_market_code() == USA) || (get_market_code() == Canada)) {
            set_tail_lamp_right(0);
            set_tail_lamp_left(0);
        }
    }

    // blinker still on -> keep usa specific stuff
    // another setting (i.e. daytime light) might have tried to turn them up again
    if(remaining_blinks && ((get_market_code() == USA) || (get_market_code() == Canada))) {
        switch(blinking_direction) {
            case blink_left:
                set_low_beam_left(50);
                break;
            case blink_right:
                set_low_beam_right(50);
                break;
            case hazard:
                set_low_beam_left(50);
                set_low_beam_right(50);
                break;
            case none:
                break;
            default:
                assert(0);
                break;
        }
    }

    // turn blinker off or on
    if(((tt - blink_timer) >= (size_t) 500) && blinking && (blinking_direction != hazard)) {
        set_blinkers_off(tt);
    }
    if(((tt - blink_timer) >= (size_t) 500) && blinking && (blinking_direction == hazard) && (ks != NoKeyInserted)) {
        set_blinkers_off(tt);
    }
    if(((tt - blink_timer) >= (size_t) 333) && blinking && (blinking_direction == hazard) && (ks == NoKeyInserted)) {
        set_blinkers_off(tt);
    }
    
    // default: 500ms pulse for non-hazard
    if(((tt - blink_timer) >= (size_t) 500) && remaining_blinks && !blinking && (blinking_direction != hazard)) {
        set_blinkers_on(tt);
    }
    // hazard: 500ms if key in lock, 667 else
    if(remaining_blinks && !blinking && (blinking_direction == hazard)) {
        if(((tt - blink_timer) >= (size_t) 500) && (ks != NoKeyInserted)) {
            set_blinkers_on(tt);
        }
        if(((tt - blink_timer) >= (size_t) 667) && (ks == NoKeyInserted)) {
            set_blinkers_on(tt);
        }
    }

    // remember last time the pitman arm was moved
    if(get_pitman_vertical() != last_pitman_arm) {
        pitman_arm_move_time = tt;
    }

    // ELS-30
    if(get_pitman_horizontal() == pa_Forward && !undervoltage) {
        set_high_beam(1);
    }
    if(get_pitman_horizontal() == pa_fb_Neutral) {
        set_high_beam(0);
    }

    // ELS-31
    if(get_pitman_horizontal() == pa_Backward) {
        set_high_beam(1);
        set_high_beam_motor(7);
        set_high_beam_range(100);
    }

    // ELS-41: reverse gear
    if(reverse_gear) {
        set_reverse_light(100);
    }
    if(!reverse_gear) {
        set_reverse_light(0);
    }
    
    last_lrs = get_light_rotary_switch();
    last_engine = engine_on;
    last_key_state = ks;
    last_all_door_closed = all_doors_closed;
    last_pitman_arm = get_pitman_vertical();
}
