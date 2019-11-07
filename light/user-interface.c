#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>

#include "user-interface.h"

static lightRotarySwitch lrs_state = 0;
static pitmanArmUpDown pacman_vertical = pa_ud_Neutral;
static pitmanArmForthBack pacman_horizontal = pa_fb_Neutral;
static bool darkness_mode = 0;
static bool hazard_warning = 0;

void reset_user_interface(void) {
    lrs_state = 0;
    pacman_vertical = pa_ud_Neutral;
    pacman_horizontal = pa_fb_Neutral;
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

void toggle_hazard_warning(void) {
    hazard_warning = !hazard_warning;
}

bool get_hazard_warning(void) {
    return hazard_warning;
}

void pitman_horizontal(pitmanArmForthBack deflection) {
    pacman_horizontal = deflection;
}

pitmanArmForthBack get_pitman_horizontal(void) {
    return pacman_horizontal;
}
