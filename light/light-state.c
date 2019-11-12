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

void set_blink_left(percentage p) {
    state.blinkLeft = p;
}

void set_blink_right(percentage p) {
    state.blinkRight = p;
}

void set_high_beam(bool b) {
    state.highBeamOn = b;
}

void set_high_beam_range(highBeamRange b) {
    state.highBeamRange = b;
}

void set_high_beam_motor(highBeamMotor b) {
    state.highBeamMotor = b;
}

void reset_lights(void) {
    state.brakeLight = 0;
    state.blinkLeft = 0;
    state.blinkRight = 0;
    state.lowBeamLeft = 0;
    state.lowBeamRight = 0;
    state.tailLampLeft = 0;
    state.tailLampRight = 0;
    state.highBeamOn = 0;
    state.highBeamMotor = 0;
    state.highBeamRange = 0;
    state.corneringLightLeft = 0;
    state.corneringLightRight = 0;
    state.reverseLight = 0;
}
