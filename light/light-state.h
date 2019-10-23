#ifndef LIGHT_LIGHT_STATE_H_INCLUDED
#define LIGHT_LIGHT_STATE_H_INCLUDED

#include <stdbool.h>
#include "../common/common.h"

typedef enum {
    hbm_min = 0,
    hbm_max = 14
} high_beam_motor;

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
    percentage corneringLightLeft;
    percentage corneringLightRight;
    percentage reverseLight;
} light_state;

light_state get_light_state(void);

#endif
