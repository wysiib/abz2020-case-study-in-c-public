#ifndef LIGHT_LIGHT_STATE_H_INCLUDED
#define LIGHT_LIGHT_STATE_H_INCLUDED

#include <stdbool.h>
#include "../common/common.h"

#include "actuators.h"

typedef struct light_state {
    percentage brakeLight;
    percentage blinkLeft;
    percentage blinkRight;
    percentage lowBeamLeft;
    percentage lowBeamRight;
    percentage tailLampLeft;
    percentage tailLampRight;
    bool highBeamOn;
    high_beam_motor highBeamMotor;
    high_beam_range highBeamRange;
    percentage corneringLightLeft;
    percentage corneringLightRight;
    percentage reverseLight;
} light_state;

light_state get_light_state(void);

bool light_state_equal(light_state s1, light_state s2);

void reset_lights(void);

#endif
