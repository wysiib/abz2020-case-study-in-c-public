#ifndef EXTERNAL_SENSOR_DRIVER_H_INCLUDED
#define EXTERNAL_SENSOR_DRIVER_H_INCLUDED

#include <stdlib.h>

#include "light/light-state.h" 
#include "light/sensors.h"

#include "cruise-control/sensors.h"

extern brightness nondet_get_brightness(void);

keyState nondet_get_key_status(void);

bool nondet_get_engine_status(void);

bool nondet_get_all_doors_closed(void);

bool nondet_get_reverse_gear(void);

voltage nondet_get_voltage_battery(void);

steeringAngle nondet_get_steering_angle(void);

bool nondet_get_oncoming_traffic(void);

size_t nondet_get_time(void);

// setters


void set_brightness(brightness val);

void set_key_status(keyState val);

void set_engine_status(bool val);

void set_all_doors_closed(bool val);

void set_reverse_gear(bool val);

void set_voltage_battery(voltage val);

void set_steering_angle(steeringAngle val);

void set_oncoming_traffic(bool val);

void set_time(size_t val);

void set_camera_state(sensorState val);

void set_sensor_current_speed(vehicleSpeed val);

void set_range_radar_state(sensorState val);

void set_range_radar_sensor(rangeRadar val);


typedef enum sensors_and_time_key {
    sensorKeyState,
    sensorEngineOn,
    sensorAllDoorsClosed,
    sensorBrightnessSensor,
    sensorReverseGear,
    sensorVoltageBattery,
    sensorSteeringAngle,
    sensorOncommingTraffic,
    sensorTime,
    sensorCameraState,
    sensorCurrentSpeed,
    sensorRangedRadar,
    sensorRangedRadarState
} sensors_and_time_key;

typedef struct sensors_and_time {
    keyState key_state;
    bool engine_on;
    bool all_doors_closed;
    brightness brightness_sensor;
    bool reverse_gear;
    voltage voltage_battery;
    steeringAngle steering_angle;
    bool oncomming_trafic;
    size_t time;
    sensorState camera_state;
    vehicleSpeed current_speed;
    sensorState range_radar_state;
    rangeRadar range_radar_distance;
} sensors_and_time;

void set_sensor_data(sensors_and_time data);

#endif
