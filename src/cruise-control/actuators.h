#ifndef CRUISE_CONTROL_ACTUATORS_H_INCLUDED
#define CRUISE_CONTROL_ACTUATORS_H_INCLUDED

#include "../common/common.h"

/** Sets the target speed for the adaptive cruise control. */
void set_vehicle_speed(vehicleSpeed speedo);

void brake_pressure(percentage p);

void acoustic_warning(bool on);

void visual_warning(bool on);

#endif
