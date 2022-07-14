#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdio.h>

#include <assert.h>

#include "light/user-interface.h"
#include "light/light-state.h"
#include "light/light-impl.h"
#include "light/sensors.h"

#include "cruise-control/scs-impl.h"

#include "cruise-control/user-interface.h"
#include "cruise-control/sensors.h"

#include "external_sensor_driver.h"

void nativehelloworld(void){
    printf("Hello World from abz library!");
}

static sensors_and_time state;

brightness nondet_get_brightness(void) {
    return state.brightness_sensor;
}

keyState nondet_get_key_status(void) {
    return state.key_state;
}

bool nondet_get_engine_status(void) {
    return state.engine_on;
}

bool nondet_get_all_doors_closed(void) {
    return state.all_doors_closed;
}

bool nondet_get_reverse_gear(void) {
    return state.reverse_gear;
}

voltage nondet_get_voltage_battery(void) {
    return state.voltage_battery;
}

steeringAngle nondet_get_steering_angle(void) {
    return state.steering_angle;
}

bool nondet_get_oncoming_traffic(void) {
    return state.oncomming_trafic;
}

size_t nondet_get_time(void) {
    return state.time;
}

sensorState nondet_get_camera_state(void) {
    return state.camera_state;
}

vehicleSpeed nondet_get_current_speed(void) {
    return state.current_speed;
}

sensorState nondet_get_range_radar_state(void) {
    return state.range_radar_state;
}

rangeRadar read_range_radar_sensor(void) {
    return state.range_radar_distance;
}

// setters


void set_brightness(brightness val) {
    state.brightness_sensor = val;
}

void set_key_status(keyState val) {
    state.key_state = val;
}

void set_engine_status(bool val) {
    state.engine_on = val;
}

void set_all_doors_closed(bool val) {
    state.all_doors_closed = val;
}

void set_reverse_gear(bool val) {
    state.reverse_gear = val;
}

void set_voltage_battery(voltage val) {
    state.voltage_battery = val;
}

void set_steering_angle(steeringAngle val) {
    state.steering_angle = val;
}

void set_oncoming_traffic(bool val) {
    state.oncomming_trafic = val;
}

void set_time(size_t val) {
    state.time = val;
}

void set_camera_state(sensorState val) {
    state.camera_state = val;
}

void set_sensor_current_speed(vehicleSpeed val) {
    state.current_speed = val;
}

void set_range_radar_state(sensorState val) {
    state.range_radar_state = val;
}

void set_range_radar_sensor(rangeRadar val) {
    state.range_radar_distance = val;
}
