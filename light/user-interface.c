#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>

#include "user-interface.h"

static lightRotarySwitch lrs_state = 0;
static pitmanArmUpDown pacman_vertical = pa_ud_Neutral;
static pitmanArmForthBack pacman_horizontal = pa_fb_Neutral;
static bool ambient_light = 0;
static bool daytime_runing_light = 0;
static bool darkness_mode = 0;
static bool hazard_warning = 0;

void reset_user_interface(void) {
    lrs_state = 0;
    pacman_vertical = pa_ud_Neutral;
    pacman_horizontal = pa_fb_Neutral;
    ambient_light = 0;
    daytime_runing_light = 0;
    darkness_mode = 0;
    hazard_warning = 0;
}

void set_light_rotary_switch(lightRotarySwitch val) {
    assert(abs((int) lrs_state - (int) val) == 1);
    lrs_state = val;
}

lightRotarySwitch get_light_rotary_switch(void) {
    return lrs_state;
}

void pitman_vertical(pitmanArmUpDown deflection) {
    pacman_vertical = deflection;
}

pitmanArmUpDown get_pitman_vertical(void) {
    return pacman_vertical;
}

void toggle_ambient_light(void) {
    ambient_light = !ambient_light;
}

bool get_ambient_light(void) {
    return ambient_light;
}


