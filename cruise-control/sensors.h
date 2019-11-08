#ifndef __CRUISE_CONTROL_SENSORS_H_INCLUDED
#define __CRUISE_CONTROL_SENSORS_H_INCLUDED

#include "../common/common.h"

typedef enum {
    nothing_detected = 0,
    distance_min = 1,
    // in meters, 1 m resolution
    distance_max = 200,
    faulty = 255
} rangeRadar;

extern keyState get_key_status(void);

extern bool get_engine_status(void);

extern Pedal get_brake_pedal_deflection(void);

sensorState get_range_radar_state(void);

rangeRadar read_range_radar_sensor(void);

// Note: Below are sensors we assume to exists, which however are not specified
// in Section 5 of the case study description as of version 1.9.
vehicleSpeed get_current_speed(void);

#endif
