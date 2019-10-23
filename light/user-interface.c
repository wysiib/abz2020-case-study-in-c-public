#include <assert.h>
#include <stdlib.h>

#include "user-interface.h"

static lightRotarySwitch lrs_state = 0;
static pitmanArmUpDown pacman_vertical = pa_ud_Neutral;
static pitmanArmForthBack pacman_horizontal = pa_fb_Neutral;


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

