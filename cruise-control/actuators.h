#ifndef __CRUISE_CONTROL_ACTUATORS_H_INCLUDED
#define __CRUISE_CONTROL_ACTUATORS_H_INCLUDED

#include "../common/common.h"

void set_vehicle_speed(vehicleSpeed speedo);

void set_brake_pressure(percentage p);

void set_acoustic_warning(bool on);

void set_visual_warning(bool on);

#endif
