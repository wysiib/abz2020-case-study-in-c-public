#ifndef CRUISE_CONTROL_ACTUATORS_H_INCLUDED
#define CRUISE_CONTROL_ACTUATORS_H_INCLUDED

#include "../common/common.h"

/** Sets the target speed for the adaptive cruise control. */
void set_vehicle_speed(vehicleSpeed speedo);

void set_brake_pressure(percentage p);

void acoustic_warning_on(void);

void set_visual_warning(bool on);

#endif
