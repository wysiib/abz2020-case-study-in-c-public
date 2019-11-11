#ifndef COMMON_COMMON_H_INCLUDED
#define COMMON_COMMON_H_INCLUDED

#include <stdlib.h>
#include <stdbool.h>

typedef enum {
    pedal_deflection_min = 0,
    // resolution: 0.2 degree
    pedal_deflection_max = 225 // 45 degree
} Pedal;

typedef enum {
    Ready,
    Dirty,
    NotReady
} sensorState;

typedef enum {NoKeyInserted, KeyInserted, KeyInIgnitionOnPosition} keyState;

typedef enum {
    speed_min = 0,
    // resolution: 0.1 km/h
    speed_max = 5000
} vehicleSpeed;

typedef enum {
    percentage_low = 0,
    percentage_high = 100
} percentage;



keyState get_key_status(void);

bool get_engine_status(void);

Pedal get_brake_pedal_deflection(void);

size_t get_time(void);

#endif
