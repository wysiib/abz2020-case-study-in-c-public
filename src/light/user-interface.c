#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>

#include "user-interface.h"

static lightRotarySwitch lrs_state = lrs_off;
static pitmanArmUpDown pacman_vertical = pa_ud_Neutral;
static pitmanArmForthBack pacman_horizontal = pa_fb_Neutral;
static bool darkness_mode = false;
static bool hazard_warning = false;

void reset_user_interface(void) {
    lrs_state = lrs_off;
    pacman_vertical = pa_ud_Neutral;
    pacman_horizontal = pa_fb_Neutral;
    darkness_mode = false;
    hazard_warning = false;
}

void set_light_rotary_switch(lightRotarySwitch val) {
    assert(abs((int) lrs_state - (int) val) <= 1);
    lrs_state = val;
}

lightRotarySwitch nondet_get_light_rotary_switch(void) {
    return lrs_state;
}

void pitman_vertical(pitmanArmUpDown deflection) {
    pacman_vertical = deflection;
}

pitmanArmUpDown nondet_get_pitman_vertical(void) {
    return pacman_vertical;
}

void toggle_hazard_warning(void) {
    hazard_warning = !hazard_warning;
}

bool nondet_get_hazard_warning(void) {
    return hazard_warning;
}

void pitman_horizontal(pitmanArmForthBack deflection) {
    pacman_horizontal = deflection;
}

pitmanArmForthBack nondet_get_pitman_horizontal(void) {
    return pacman_horizontal;
}

void toggle_darkness_mode(void) {
    darkness_mode = !darkness_mode;
}

bool nondet_get_darkness_mode(void) {
    return darkness_mode;
}
