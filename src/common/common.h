#ifndef COMMON_COMMON_H_INCLUDED
#define COMMON_COMMON_H_INCLUDED

#include <stdlib.h>
#include <stdbool.h>

typedef enum {
    voltage_min = 0,
    // resolution: 0.1 V
    voltage_max = 500 // 50V
} voltage;

typedef enum {
    st_calibrating = 0,
    st_hard_left_max = 1,
    // resolution: 1 degree
    st_hard_left_min = 410,
    st_soft_left_max = 411,
    // resolution: 0.1 degree
    st_soft_left_min = 510,
    st_neutral_maxl = 511,
    st_neutral = 512,
    st_neutral_maxr = 513,
    st_soft_right_min = 514,
    // resolution: 0.1 degree
    st_soft_right_max = 613,
    st_hard_right_min = 614,
    // resolution: 1 degree
    st_hard_right_max = 1022
} steeringAngle;

typedef enum {brightness_min = 0, brightness_max = 100000} brightness;

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

bool nondet_get_oncoming_traffic(void);

sensorState nondet_get_camera_state(void);

keyState nondet_get_key_status(void);

bool nondet_get_engine_status(void);

Pedal nondet_get_brake_pedal_deflection(void);

size_t nondet_get_time(void);

brightness nondet_get_brightness(void);

bool nondet_get_all_doors_closed(void);

bool nondet_get_reverse_gear(void);

voltage nondet_get_voltage_battery(void);

steeringAngle nondet_get_steering_angle(void);

#endif
