#include "light-state.h"
#include "actuators.h"

static light_state state;

light_state get_light_state(void) {
    return state;
}

void set_low_beam_left(percentage p) {
    state.lowBeamLeft = p;
}

void set_low_beam_right(percentage p) {
    state.lowBeamRight = p;
}

void set_tail_lamp_left(percentage p) {
    state.tailLampLeft = p;
}

void set_tail_lamp_right(percentage p) {
    state.tailLampRight = p;
}
