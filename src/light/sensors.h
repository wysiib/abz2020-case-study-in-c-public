#ifndef LIGHT_SENSORS_H_INCLUDED
#define LIGHT_SENSORS_H_INCLUDED

#include "../common/common.h"

extern keyState nondet_get_key_status(void);

extern bool nondet_get_engine_status(void);

extern Pedal nondet_get_brake_pedal_deflection(void);

vehicleSpeed nondet_get_current_speed(void);

#endif
