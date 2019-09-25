#ifndef __LIGHT_SENSORS_H_INCLUDED
#define __LIGHT_SENSORS_H_INCLUDED

#include "../common/common.h"

typedef enum {brightness_min = 0, brightness_max = 100000} brightness;

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

extern keyState get_key_status(void);

extern bool get_engine_status(void);

brightness get_brightness(void);

extern Pedal get_brake_pedal_deflection(void);

voltage get_voltage_battery(void);

steeringAngle get_steering_angle(void);

bool get_all_doors_closed(void);

bool get_oncoming_traffic(void);

sensorState get_camera_state(void);

vehicleSpeed get_current_speed(void);

bool get_reverse_gear(void);

#endif
