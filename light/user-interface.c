#include <assert.h>
#include <stdlib.h>

#include "user-interface.h"

static lightRotarySwitch lrs_state = 0;

void set_light_rotary_switch(lightRotarySwitch val) {
    assert(abs((int) lrs_state - (int) val) == 1);
    lrs_state = val;
}

lightRotarySwitch get_light_rotary_switch(void) {
    return lrs_state;
}
