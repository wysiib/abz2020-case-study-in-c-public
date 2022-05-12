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

void set_reverse_light(percentage p) {
    state.reverseLight = p;
}

void set_high_beam(bool b) {
    state.highBeamOn = b;
}

void set_high_beam_range(high_beam_range b) {
    state.highBeamRange = b;
}

void set_high_beam_motor(high_beam_motor b) {
    state.highBeamMotor = b;
}

void set_cornering_light_left(percentage p) {
    state.corneringLightLeft = p;
}

void set_cornering_light_right(percentage p) {
    state.corneringLightRight = p;
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

bool light_state_equal(light_state s1, light_state s2){
    return
    s1.brakeLight == s2.brakeLight &&
    s1.blinkLeft == s2.blinkLeft &&
    s1.blinkRight == s2.blinkRight &&
    s1.lowBeamLeft == s2.lowBeamLeft &&
    s1.lowBeamRight == s2.lowBeamRight &&
    s1.tailLampLeft == s2.tailLampLeft &&
    s1.tailLampRight == s2.tailLampRight &&
    s1.highBeamOn == s2.highBeamOn &&
    s1.highBeamMotor == s2.highBeamMotor &&
    s1.highBeamRange == s2.highBeamRange &&
    s1.corneringLightLeft == s2.corneringLightLeft &&
    s1.corneringLightRight == s2.corneringLightRight ;
}
